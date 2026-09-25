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
 * State feedback: PIN_FEEDBACK reads the contactor's NO auxiliary
 * contact - terminals 13/14 on the Schneider LC1E0910M5 - so what we
 * report is what the contactor actually did, not merely what we asked
 * it to do.
 *   ESP32 GND -> terminal 13, terminal 14 -> PIN_FEEDBACK (GPIO 13).
 *   The aux contact is a dry contact: it carries only the voltage we
 *   feed into it, so this loop sits at 3.3 V throughout and needs no
 *   isolation - PROVIDED 13/14 are wired to nothing else.
 *   Use an EXTERNAL 1k pull-up from PIN_FEEDBACK to 3.3 V instead of
 *   relying on the internal one. Schneider rates this aux contact for a
 *   minimum of 17 V / 5 mA; the ESP32's ~45k internal pull-up passes
 *   only ~70 uA, far too little to break through the oxide film that
 *   forms on the contact surface, which shows up as intermittent false
 *   readings. 1k gives ~3.3 mA.
 *   Until 13/14 are physically wired, this pin just sits at the pull-up
 *   and reports OFF permanently.
 *
 * Topics: cmd (in, not retained - retaining it would make the ESP32
 * re-execute the last command on every reconnect, which would defeat
 * the fail-safe above), state (out, RETAINED - the real contactor
 * reading), log (out, not retained), avail (out, retained, "online" /
 * "offline" via MQTT's last will), hb (out, NOT retained - proof of
 * life every HEARTBEAT_MS, payload = uptime in seconds), status (out,
 * NOT retained - the reply to a STATUS command).
 *
 * Commands accepted on cmd: ON, OFF, RESTART, STATUS. STATUS asks the
 * board to report which WiFi network it is on, its signal strength, IP,
 * uptime, current contactor state, whether a restart is in progress and
 * who issued the last command. It is the on-demand counterpart to the
 * heartbeat: instead of the board announcing itself constantly whether
 * or not anyone is listening, the app asks when it actually wants to
 * know - on open, or behind a "check controller" button. Hear no reply
 * within a second or two and the controller is not there.
 *
 * Command payload format: "CMD" or "CMD:user", e.g. "ON" or
 * "ON:Mohamed". The user part is optional so a plain MQTT client can
 * still drive the pump by hand, and it is recorded in the log and in
 * the STATUS reply so it is always clear who changed the motor. Both
 * halves travel in ONE message on purpose - sending the actor on a
 * separate topic could not be correlated reliably once two people act
 * at nearly the same time.
 *
 * RESTART is refused while a restart is already running. A UI can grey
 * the button out, but only the board can actually enforce this: a UI
 * lock is a convention that any other MQTT client ignores, and without
 * the guard a second RESTART would simply extend the power cut by
 * another RESTART_DELAY_MS, so repeated presses could keep the pump off
 * indefinitely.
 *
 * CPU throttling: the board runs at 240 MHz while it is being used and
 * drops to 80 MHz after CPU_IDLE_MS with no command, returning to full
 * speed the instant one arrives. 80 MHz is the floor that keeps this
 * safe - at 240/160/80 MHz the CPU is clocked from the PLL and the APB
 * bus stays at 80 MHz, so UART baud rates and peripheral timing are
 * untouched. Below 80 MHz the CPU switches to the crystal, APB follows
 * it down, and the serial output turns to garbage. This is about heat
 * as much as power: the enclosure lives under the stairs with poor
 * ventilation and hot summers.
 *
 * How a subscriber should judge freshness: the retained state arrives
 * the instant it subscribes, so the button can be drawn immediately -
 * but a retained message carries no timestamp, so on its own it cannot
 * say whether that value is current or was left behind hours ago by a
 * board that has since died. avail answers that using the broker's own
 * disconnect detection, which only fires after the keepalive expires
 * (~22 s with this library's 15 s default). hb is the independent
 * check: it is deliberately NOT retained, so it can only ever arrive
 * from a board that is alive right now. Hear nothing on hb for about
 * two and a half intervals and treat the state as stale, whatever
 * avail happens to say. The uptime payload also makes a crash-loop
 * obvious - it keeps restarting from a small number.
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
 * OTA updates (ArduinoOTA, LAN only): the board advertises itself over
 * mDNS as OTA_HOSTNAME, so the Arduino IDE lists it under Tools > Port as
 * a network port and Upload flashes it over WiFi - no cable, no trip to
 * the enclosure. No internet is involved: this is local-network only, so
 * it keeps working during the months the internet subscription has
 * lapsed. It is password protected, and that is not optional - without a
 * password anyone on the LAN could reflash the board that switches mains
 * power to the pump.
 *
 * If the board never appears in the IDE's port list, that is mDNS
 * discovery being filtered somewhere on the network, not a fault in the
 * board - the upload still works when addressed by IP directly. Ask the
 * board for its current address with a STATUS command.
 *
 * Safety during an update: onStart deliberately releases the relay before
 * the flash is overwritten. An upload that fails half way leaves a dead
 * board, and a dead board must never be the thing holding the motor's
 * power off - the same fail-safe reasoning as the rest of this design.
 * An update therefore always ends with power flowing, whatever was
 * commanded before it.
 *
 * State reporting: home/pump/state is published on three occasions, and
 * they are deliberately separate. (1) On every settled change of the
 * feedback pin - the live signal. (2) Once when MQTT connects, to refresh
 * the retained value. (3) STATE_CONFIRM_MS after any command that changes
 * the pump, whether or not the reading moved.
 *
 * The third one exists because silence is ambiguous: if a command
 * produced no change of state, the app could not tell whether the board
 * ignored it, the relay failed, or the contactor never moved. Publishing
 * regardless turns that silence into an answer. It is deliberately
 * DELAYED rather than immediate - publishing the instant the command
 * arrives would report the pin's value from before the contactor had
 * time to move, which is worse than not publishing at all. The log line
 * that goes with it prints measured and commanded side by side and marks
 * a MISMATCH, which is what a failed actuation actually looks like.
 *
 * Pin map: GPIO 26 drives the relay, GPIO 13 reads the contactor's
 * auxiliary contact, GPIO 33 is reserved as the fan output and is held
 * low (fan off) until the fan logic exists. GPIO 33 is deliberately the
 * reserved one: it sits on ADC1, which still works while WiFi is on,
 * so if the fan plan later turns into an analog temperature sensor
 * instead, that pin can take it directly. GPIO 13 is ADC2, which the
 * radio makes unusable for analog anyway, so the permanently-digital
 * feedback input goes there and nothing is wasted.
 *
 * Firmware version: FW_VERSION is bumped by 0.1 for every change that
 * gets flashed. It is printed in the boot log and returned in every
 * STATUS reply, so "is the board actually running my new code?" has a
 * definite answer instead of depending on remembering which log line was
 * new - which matters now that updates go over the air to a board that
 * is awkward to reach. FW_BUILD is filled in by the compiler and covers
 * the case the version number was forgotten, which is exactly the case
 * where the question is hardest to answer.
 *
 * Library: PubSubClient (Nick O'Leary)
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

#include "secrets.h"   // WIFI_SSID_1/2, WIFI_PASS_1/2, MQTT_HOST/PORT/USER/PASS

const uint32_t WIFI_TIMEOUT_MS = 15000;   // how long to try each network before moving on

// Bump by 0.1 on every change that gets flashed. See the header note.
const char *FW_VERSION = "1.2";
const char *FW_BUILD   = __DATE__ " " __TIME__;

// Name the board announces over mDNS; this is what appears in the Arduino
// IDE's port list. The OTA password lives in secrets.h.
const char *OTA_HOSTNAME = "pump-esp32";

const char *T_CMD   = "home/pump/cmd";
const char *T_STATE = "home/pump/state";   // relay feedback, retained
const char *T_LOG   = "home/pump/log";     // system log, not retained
const char *T_AVAIL = "home/pump/avail";   // "online" / "offline" (LWT), retained
const char *T_HB    = "home/pump/hb";      // proof of life, NOT retained
const char *T_STATUS= "home/pump/status";  // reply to a STATUS command, NOT retained

const int  PIN_RELAY        = 26;
// Hardware property of this opto module - it triggers (energizes the
// relay) on a LOW input. This is fixed by the module's own circuit, not
// a software choice - don't flip it without changing the module itself.
const bool RELAY_ACTIVE_LOW = true;

// --- contactor state feedback (see header for the wiring) ---
// The pull-up idles PIN_FEEDBACK HIGH. The contactor's aux contact 13/14
// pulls it to GND only while the contactor has actually pulled in, so
// this reports what the hardware did rather than what we asked for.
const int  PIN_FEEDBACK        = 13;
const bool FEEDBACK_ACTIVE_LOW = true;
const uint32_t FB_DEBOUNCE_MS  = 150;

// Reserved for the enclosure fan. Held LOW = fan off. No logic yet; this
// only claims the pin and stops it floating. The assumption is a low-side
// MOSFET with an active-high gate, which also wants its own ~10k gate
// pull-down so the fan cannot twitch during the ESP32's boot, before this
// line runs.
const int  PIN_FAN = 33;

const uint32_t RESTART_DELAY_MS = 6000;   // how long power stays cut during a RESTART

// How long to wait after a command before publishing the resulting state.
// Must outlast the contactor's mechanical travel plus FB_DEBOUNCE_MS.
const uint32_t STATE_CONFIRM_MS = 1000;

// Heartbeat period. This is now a slow background sanity signal, not the
// primary liveness check - avail answers that within ~22 s for free, and
// STATUS answers it on demand and instantly. What the heartbeat still
// earns its place for is the uptime payload: watch it reset to a small
// number over and over and you are looking at a crash loop, which is
// otherwise very hard to spot. At 60 s it costs a few MB a month.
const uint32_t HEARTBEAT_MS = 60000;

// --- idle CPU throttling (see header note) ---
const uint32_t CPU_IDLE_MS   = 3600000UL;   // 1 hour of no commands -> throttle
const uint32_t CPU_FREQ_FULL = 240;
const uint32_t CPU_FREQ_IDLE = 80;          // do NOT go below 80: APB and UART follow

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

uint32_t lastHeartbeatMs = 0;

bool     confirmPending = false;  // a command is awaiting its state report
uint32_t confirmAtMs    = 0;
bool     cmdWantsOn     = false;  // what that command intended

uint32_t lastCmdMs    = 0;       // when a command last arrived
bool     cpuThrottled = false;
char     lastUser[24] = "none";  // who issued the last command

void relayWrite(bool on) {
  digitalWrite(PIN_RELAY, (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void publishFeedback(bool on) {
  if (!mqtt.connected()) return;
  mqtt.publish(T_STATE, on ? "ON" : "OFF", true);   // retained
}

// Proof of life. Deliberately NOT retained: the broker keeps no copy, so
// this message can only ever reach a subscriber from a board that is
// alive at that moment. That is what lets the app decide for itself that
// the data has gone stale, instead of waiting on the broker's keepalive
// timeout. Payload is uptime in seconds.
void publishHeartbeat() {
  if (!mqtt.connected()) return;
  char msg[16];
  snprintf(msg, sizeof(msg), "%lu", (unsigned long)(millis() / 1000));
  mqtt.publish(T_HB, msg);
  lastHeartbeatMs = millis();
}

// Answers a STATUS command. NOT retained, for the same reason as the
// heartbeat: a reply the broker could hand out later would prove nothing
// about whether this board is alive now. Publish STATUS, hear nothing
// back within a second or two, and the controller is offline.
void publishStatus() {
  if (!mqtt.connected()) return;
  char buf[360];
  snprintf(buf, sizeof(buf),
    "{\"fw\":\"%s\",\"build\":\"%s\","
    "\"ssid\":\"%s\",\"rssi\":%d,\"ip\":\"%s\",\"uptime_s\":%lu,"
    "\"state\":\"%s\",\"restarting\":%s,\"last_user\":\"%s\",\"cpu_mhz\":%u}",
    FW_VERSION,
    FW_BUILD,
    WiFi.SSID().c_str(),
    (int)WiFi.RSSI(),
    WiFi.localIP().toString().c_str(),
    (unsigned long)(millis() / 1000),
    fbStable ? "ON" : "OFF",
    restartPending ? "true" : "false",
    lastUser,
    (unsigned)getCpuFrequencyMhz());
  mqtt.publish(T_STATUS, buf);
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

// 240 MHz while in use, 80 MHz when idle. Switching is immediate and
// costs nothing functionally - WiFi, MQTT and the feedback pin all carry
// on exactly as before, only slower per instruction, which this workload
// never notices.
void cpuFull() {
  if (!cpuThrottled) return;
  setCpuFrequencyMhz(CPU_FREQ_FULL);
  cpuThrottled = false;
  logLine("cpu: 240 MHz (command received)");
}

void cpuIdle() {
  if (cpuThrottled) return;
  setCpuFrequencyMhz(CPU_FREQ_IDLE);
  cpuThrottled = true;
  logLine("cpu: 80 MHz (idle)");
}

// LAN-only firmware updates over WiFi. See the header note for why the
// relay is released before the flash is overwritten.
void setupOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    // Full speed for the flash write, and put the pump back into the safe
    // state first: from here until the reboot completes, this board is not
    // in control of anything, so it must not be holding power off.
    cpuFull();
    restartPending = false;
    relayWrite(false);
    logLine("ota: update starting - relay released, pump powered");
  });

  ArduinoOTA.onEnd([]() {
    logLine("ota: image written, rebooting");
  });

  ArduinoOTA.onError([](ota_error_t error) {
    char msg[48];
    snprintf(msg, sizeof(msg), "ota: FAILED (error %u)", (unsigned)error);
    logLine(msg);
  });

  ArduinoOTA.begin();

  char msg[80];
  snprintf(msg, sizeof(msg), "ota: ready as %s at %s",
           OTA_HOSTNAME, WiFi.localIP().toString().c_str());
  logLine(msg);
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
  // Payload is "CMD" or "CMD:user".
  char raw[64] = {0};
  unsigned int n = (len < sizeof(raw) - 1) ? len : sizeof(raw) - 1;
  memcpy(raw, payload, n);
  raw[n] = 0;
  while (n > 0 && (raw[n-1] == '\n' || raw[n-1] == '\r' || raw[n-1] == ' '))
    raw[--n] = 0;

  // Split on the first ':' - left is the command, right is who sent it.
  char *who = strchr(raw, ':');
  if (who) { *who = 0; who++; }
  if (!who || !*who) who = (char *)"unknown";
  snprintf(lastUser, sizeof(lastUser), "%s", who);

  // Only the command is case-folded; a person's name keeps its own case.
  char cmd[16] = {0};
  for (unsigned int i = 0; i < sizeof(cmd) - 1 && raw[i]; i++)
    cmd[i] = toupper(raw[i]);

  // Any command counts as activity: wake the CPU before acting on it.
  lastCmdMs = millis();
  cpuFull();

  char msg[96];
  snprintf(msg, sizeof(msg), "cmd: [%s] from %s", cmd, lastUser);
  logLine(msg);

  if (!strcmp(cmd, "ON")) {
    restartPending = false;
    relayWrite(false);   // de-energize -> COM/NC closed -> power to motor
    cmdWantsOn = true;
    confirmPending = true;
    confirmAtMs = millis() + STATE_CONFIRM_MS;
  } else if (!strcmp(cmd, "OFF")) {
    restartPending = false;
    relayWrite(true);    // energize -> COM/NC open -> power cut
    cmdWantsOn = false;
    confirmPending = true;
    confirmAtMs = millis() + STATE_CONFIRM_MS;
  } else if (!strcmp(cmd, "RESTART")) {
    // Enforced here, not in the UI: without this a second RESTART would
    // just push restartAtMs further out and keep the pump off.
    if (restartPending) {
      logLine("restart already in progress - ignored");
    } else {
      relayWrite(true);    // cut power first
      restartPending = true;
      restartAtMs = millis() + RESTART_DELAY_MS;
      // A restart ends with power flowing, so confirm after it finishes.
      cmdWantsOn = true;
      confirmPending = true;
      confirmAtMs = restartAtMs + STATE_CONFIRM_MS;
    }
  } else if (!strcmp(cmd, "STATUS")) {
    publishStatus();
  }
}

void setup() {
  // Boot-safe default: de-energized -> COM/NC closed -> motor controller
  // stays powered even if the ESP32 never gets further than this line.
  relayWrite(false);            // set the level BEFORE the pin becomes an output
  pinMode(PIN_RELAY, OUTPUT);
  relayWrite(false);

  // Fan output: drive the safe level before the pin becomes an output, so
  // it never twitches on the way up.
  digitalWrite(PIN_FAN, LOW);
  pinMode(PIN_FAN, OUTPUT);
  digitalWrite(PIN_FAN, LOW);

  // The external 1k pull-up does the real work here (see header); the
  // internal one costs nothing and keeps the pin defined if that
  // resistor is ever missing.
  pinMode(PIN_FEEDBACK, INPUT_PULLUP);

  // Seed the feedback state from an actual reading, so the first thing
  // we publish reflects reality rather than a guessed default.
  delay(10);
  fbLastRaw = digitalRead(PIN_FEEDBACK);
  if (FEEDBACK_ACTIVE_LOW) fbLastRaw = !fbLastRaw;
  fbStable   = fbLastRaw;
  fbChangeMs = millis();

  Serial.begin(115200);

  // First line of every boot, so the log always opens by saying which
  // build is talking. It buffers until MQTT is up, then goes out with the
  // rest of the backlog.
  {
    char v[72];
    snprintf(v, sizeof(v), "fw %s (built %s)", FW_VERSION, FW_BUILD);
    logLine(v);
  }

  WiFi.mode(WIFI_STA);
  // We handle reconnection ourselves (cycling both networks from loop()),
  // so the built-in single-SSID auto-reconnect would only get in the way.
  WiFi.setAutoReconnect(false);

  net.setInsecure();            // no certificate check - see note
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);
  mqtt.setBufferSize(512);   // the STATUS reply does not fit the 256-byte default

  lastCmdMs = millis();   // start the idle countdown from boot

  connectWiFiWithFallback();
  char msg[48];
  snprintf(msg, sizeof(msg), "IP: %s", WiFi.localIP().toString().c_str());
  logLine(msg);

  setupOTA();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    logLine("WiFi disconnected - reconnecting");
    connectWiFiWithFallback();
  }

  ArduinoOTA.handle();

  if (WiFi.status() == WL_CONNECTED && !mqtt.connected()
      && millis() - lastTryMs > 5000) {
    lastTryMs = millis();
    // The last will lets the broker announce "offline" for us if we drop
    // without saying goodbye - otherwise the retained state below would
    // keep looking authoritative long after this board had died.
    if (mqtt.connect("pump-esp32", MQTT_USER, MQTT_PASS,
                     T_AVAIL, 1, true, "offline")) {
      mqtt.publish(T_AVAIL, "online", true);
      mqtt.subscribe(T_CMD, 1);
      publishFeedback(fbStable);   // refresh the retained state right away
      publishHeartbeat();          // and prove we're alive without the wait
      logLine("mqtt connected");
    }
  }
  mqtt.loop();
  flushLogToMqtt();

  if (millis() - lastHeartbeatMs >= HEARTBEAT_MS) publishHeartbeat();

  readFeedback();

  // Report the state a command produced, changed or not. This is the
  // answer to "did that command actually do anything?".
  if (confirmPending && (int32_t)(millis() - confirmAtMs) >= 0) {
    confirmPending = false;
    publishFeedback(fbStable);
    char msg[80];
    snprintf(msg, sizeof(msg), "state after cmd: measured=%s commanded=%s%s",
             fbStable   ? "ON" : "OFF",
             cmdWantsOn ? "ON" : "OFF",
             (fbStable == cmdWantsOn) ? "" : "   <-- MISMATCH");
    logLine(msg);
  }

  if (restartPending && (int32_t)(millis() - restartAtMs) >= 0) {
    restartPending = false;
    relayWrite(false);   // restore power
    logLine("restart complete");
  }

  // Drop to 80 MHz once nothing has been asked of us for a while.
  if (!cpuThrottled && millis() - lastCmdMs >= CPU_IDLE_MS) cpuIdle();
}
