/*
 * Water pump remote control - minimal version
 * WiFi -> HiveMQ Cloud (TLS) -> ON / OFF / RESTART -> relay
 *
 * Credentials live in secrets.h, which is gitignored. Copy
 * secrets_example.h to secrets.h and fill in your own values before
 * building.
 *
 * Relay wiring (fail-safe, NC): the relay's COM and NC contacts feed the
 * "power to motor controller" line. De-energized (idle / ESP32 dead or
 * rebooting) -> COM-NC closed -> motor controller stays powered, which is
 * our normal state, since the motor/DMAX side is meant to be powered
 * around the clock anyway. Energizing the relay opens COM-NC and cuts
 * that power. So the logical pump command and the physical relay state
 * are inverted from each other:
 *   pump ON  (power flowing) -> relay de-energized -> relayWrite(false)
 *   pump OFF (power cut)     -> relay energized     -> relayWrite(true)
 *
 * IMPORTANT: PIN_RELAY floats during the ESP32's own boot/reset, before
 * this code ever runs - add a physical 10k pull-up from PIN_RELAY to
 * 3.3V so it reads HIGH (de-energized / power-on) during that window
 * too, otherwise the fail-safe guarantee only holds once setup() has
 * actually executed.
 *
 * Relay feedback (bench test): read back a spare relay contact to
 * confirm the relay contact really actuated when commanded.
 *   ESP32 GND -> relay COM, relay NO -> PIN_FEEDBACK.
 *   Uses only the ESP32's own GND through a spare/unused contact - no
 *   coil voltage or mains involved, so no isolation is needed for this
 *   test wiring. Note: a 1-channel relay has only one COM terminal, so
 *   this test loopback and the NC production wiring above can't be
 *   connected at the same time - test first, then move COM/NC to the
 *   real circuit.
 *
 * WiFi: tries WIFI_SSID_1 first, falls back to WIFI_SSID_2 if it can't
 * connect within WIFI_TIMEOUT_MS. This runs at boot AND is re-run from
 * loop() any time WiFi drops - the built-in ESP32 auto-reconnect only
 * ever retries the last SSID, so it's turned off and we cycle between
 * both networks ourselves until one answers.
 *
 * Logging: every line that goes to Serial also goes out on T_LOG
 * ("home/pump/log") so a subscriber (e.g. the mobile app) sees the same
 * system log live, for direct debugging. Lines produced before WiFi/MQTT
 * are up (e.g. "failed to connect to internet") would otherwise be lost,
 * so they're held in a small RAM ring buffer (LOG_BUF_SIZE bytes) and
 * flushed out in order the moment MQTT connects. This buffer is RAM, not
 * flash: it's meant to bridge a WiFi/MQTT outage, not survive a power
 * loss - if a full reboot happens before it's flushed, that backlog is
 * gone. If it fills up before a connection comes back, the OLDEST lines
 * are dropped (overwritten) to make room for new ones.
 *
 * Library: PubSubClient (Nick O'Leary)
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include "secrets.h"   // WIFI_SSID_1/2, WIFI_PASS_1/2, MQTT_HOST/PORT/USER/PASS

const uint32_t WIFI_TIMEOUT_MS = 15000;   // how long to try each network before moving on

const char *T_CMD   = "home/pump/cmd";
const char *T_STATE = "home/pump/state";   // relay feedback, retained
const char *T_LOG   = "home/pump/log";     // system log, not retained

const int  PIN_RELAY        = 26;
// Hardware property of this opto module - it triggers (energizes the
// relay) on a LOW input. This is fixed by the module's own circuit, not
// a software choice - don't flip it without changing the module itself.
const bool RELAY_ACTIVE_LOW = true;

// --- feedback (bench test) ---
// Internal pull-up idles PIN_FEEDBACK HIGH. Wiring GND through the relay's
// spare contact pulls it LOW only while that contact is actually closed -
// i.e. only while the relay has really actuated, not just while we've
// commanded it to.
const int  PIN_FEEDBACK        = 27;
const bool FEEDBACK_ACTIVE_LOW = true;
const uint32_t FB_DEBOUNCE_MS  = 150;

const uint32_t RESTART_DELAY_MS = 6000;   // how long power stays cut during a RESTART

// --- offline log buffer (see header note) ---
const size_t LOG_BUF_SIZE = 4096;   // ~ enough for 60-100 typical log lines
char     logBuf[LOG_BUF_SIZE];
size_t   logLen        = 0;   // bytes currently held
size_t   logSentOffset = 0;   // how many of those bytes are already published

WiFiClientSecure net;
PubSubClient     mqtt(net);

bool     restartPending = false;
uint32_t restartAtMs    = 0;
uint32_t lastTryMs      = 0;

bool     fbStable   = false;   // debounced feedback reading
bool     fbLastRaw  = false;
uint32_t fbChangeMs = 0;

void relayWrite(bool on) {
  digitalWrite(PIN_RELAY, (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void publishFeedback(bool on) {
  if (!mqtt.connected()) return;
  mqtt.publish(T_STATE, on ? "ON" : "OFF", true);   // retained
}

// Appends one line to the ring buffer, dropping the oldest complete
// line(s) first if there isn't room.
void logAppend(const char *line) {
  size_t n = strlen(line);
  size_t need = n + 1;   // + trailing '\n'
  if (need > LOG_BUF_SIZE) return;   // a single line bigger than the whole buffer - drop it

  if (logLen + need > LOG_BUF_SIZE) {
    size_t toDrop = (logLen + need) - LOG_BUF_SIZE;
    size_t dropped = 0;
    while (dropped < toDrop && dropped < logLen) {
      char *nl = (char *)memchr(logBuf + dropped, '\n', logLen - dropped);
      if (!nl) { dropped = logLen; break; }
      dropped = (nl - logBuf) + 1;
    }
    memmove(logBuf, logBuf + dropped, logLen - dropped);
    logLen -= dropped;
    logSentOffset = (logSentOffset > dropped) ? (logSentOffset - dropped) : 0;
  }

  memcpy(logBuf + logLen, line, n);
  logBuf[logLen + n] = '\n';
  logLen += need;
}

// Publishes whatever hasn't been sent yet, one line per MQTT message.
void flushLogToMqtt() {
  if (!mqtt.connected()) return;
  while (logSentOffset < logLen) {
    char *nl = (char *)memchr(logBuf + logSentOffset, '\n', logLen - logSentOffset);
    if (!nl) break;
    size_t lineLen = nl - (logBuf + logSentOffset);
    char line[128];
    if (lineLen >= sizeof(line)) lineLen = sizeof(line) - 1;
    memcpy(line, logBuf + logSentOffset, lineLen);
    line[lineLen] = 0;
    mqtt.publish(T_LOG, line);
    logSentOffset = (nl - logBuf) + 1;
  }
  if (logSentOffset == logLen) { logLen = 0; logSentOffset = 0; }   // fully drained
}

// Every meaningful log line goes through here: Serial always gets it,
// the buffer/MQTT path follows the same content.
void logLine(const char *msg) {
  Serial.println(msg);
  logAppend(msg);
  flushLogToMqtt();
}

// Tries one network for up to timeoutMs. The "." progress dots stay
// Serial-only (cosmetic); the meaningful milestones go through logLine()
// once the attempt is finished, so we don't flood T_LOG with one message
// per dot.
bool connectToWiFi(const char *ssid, const char *pass, uint32_t timeoutMs) {
  char msg[96];
  snprintf(msg, sizeof(msg), "connecting to WiFi: %s", ssid);
  logLine(msg);

  WiFi.begin(ssid, pass);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

// Primary network first, then the backup, over and over until one
// answers. Called at boot and again from loop() whenever WiFi drops.
void connectWiFiWithFallback() {
  for (;;) {
    if (connectToWiFi(WIFI_SSID_1, WIFI_PASS_1, WIFI_TIMEOUT_MS)) {
      char msg[96];
      snprintf(msg, sizeof(msg), "connected to: %s", WIFI_SSID_1);
      logLine(msg);
      return;
    }
    if (connectToWiFi(WIFI_SSID_2, WIFI_PASS_2, WIFI_TIMEOUT_MS)) {
      char msg[96];
      snprintf(msg, sizeof(msg), "connected to: %s", WIFI_SSID_2);
      logLine(msg);
      return;
    }
    logLine("failed to connect to internet");
  }
}

// Debounce PIN_FEEDBACK and publish only on a real, settled change - this
// is what actually confirms the relay contact moved, as opposed to just
// trusting the command we sent it.
void readFeedback() {
  bool raw = digitalRead(PIN_FEEDBACK);
  if (FEEDBACK_ACTIVE_LOW) raw = !raw;

  if (raw != fbLastRaw) {
    fbLastRaw = raw;
    fbChangeMs = millis();
  } else if (raw != fbStable && (millis() - fbChangeMs) >= FB_DEBOUNCE_MS) {
    fbStable = raw;
    char msg[32];
    snprintf(msg, sizeof(msg), "feedback: %s", fbStable ? "ON" : "OFF");
    logLine(msg);
    publishFeedback(fbStable);
  }
}

void onMessage(char *topic, byte *payload, unsigned int len) {
  char cmd[16] = {0};
  unsigned int n = (len < 15) ? len : 15;
  for (unsigned int i = 0; i < n; i++) cmd[i] = toupper((char)payload[i]);
  while (n > 0 && (cmd[n-1] == '\n' || cmd[n-1] == '\r' || cmd[n-1] == ' '))
    cmd[--n] = 0;

  char msg[32];
  snprintf(msg, sizeof(msg), "cmd: [%s]", cmd);
  logLine(msg);

  if (!strcmp(cmd, "ON")) {
    restartPending = false;
    relayWrite(false);   // de-energize -> COM/NC closed -> power to motor
  } else if (!strcmp(cmd, "OFF")) {
    restartPending = false;
    relayWrite(true);    // energize -> COM/NC open -> power cut
  } else if (!strcmp(cmd, "RESTART")) {
    relayWrite(true);    // cut power first
    restartPending = true;
    restartAtMs = millis() + RESTART_DELAY_MS;
  }
}

void setup() {
  // Boot-safe default: de-energized -> COM/NC closed -> motor controller
  // stays powered even if the ESP32 never gets further than this line.
  relayWrite(false);            // set the level BEFORE the pin becomes an output
  pinMode(PIN_RELAY, OUTPUT);
  relayWrite(false);

  pinMode(PIN_FEEDBACK, INPUT_PULLUP);

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  // We handle reconnection ourselves (cycling both networks from loop()),
  // so the built-in single-SSID auto-reconnect would only get in the way.
  WiFi.setAutoReconnect(false);

  net.setInsecure();            // no certificate check - see note
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);

  connectWiFiWithFallback();
  char msg[48];
  snprintf(msg, sizeof(msg), "IP: %s", WiFi.localIP().toString().c_str());
  logLine(msg);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    logLine("WiFi disconnected - reconnecting");
    connectWiFiWithFallback();
  }

  if (WiFi.status() == WL_CONNECTED && !mqtt.connected()
      && millis() - lastTryMs > 5000) {
    lastTryMs = millis();
    if (mqtt.connect("pump-esp32", MQTT_USER, MQTT_PASS)) {
      mqtt.subscribe(T_CMD, 1);
      logLine("mqtt connected");
    }
  }
  mqtt.loop();
  flushLogToMqtt();

  readFeedback();

  if (restartPending && (int32_t)(millis() - restartAtMs) >= 0) {
    restartPending = false;
    relayWrite(false);   // restore power
  }
}
