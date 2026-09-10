/*
 * Water pump remote control - ESP32
 *
 * ESP32 -> opto relay module -> contactor coil (A1/A2) -> pump circuit
 * Contactor aux NO (13/14) -> PC817 -> ESP32 input  (real state feedback)
 *
 * Libraries required:
 *   PubSubClient  (Nick O'Leary)  - Library Manager
 *
 * Topics:
 *   home/pump/cmd    <- ON | OFF | RESTART
 *   home/pump/state  -> retained JSON
 *   home/pump/avail  -> "online" / "offline" (LWT, retained)
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// ============================ CONFIG ============================

const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";

const char     *MQTT_HOST      = "xxxxxxxx.s1.eu.hivemq.cloud";
const uint16_t  MQTT_PORT      = 8883;
const char     *MQTT_USER      = "YOUR_HIVEMQ_USERNAME";
const char     *MQTT_PASS      = "YOUR_HIVEMQ_PASSWORD";
const char     *MQTT_CLIENT_ID = "pump-esp32";

const char *T_CMD   = "home/pump/cmd";
const char *T_STATE = "home/pump/state";
const char *T_AVAIL = "home/pump/avail";

// --- pins (avoid strapping pins 0, 2, 12, 15) ---
const int PIN_RELAY    = 26;
const int PIN_FEEDBACK = 27;

// Most opto relay modules trigger on LOW. Verify yours before wiring the coil.
const bool RELAY_ACTIVE_LOW    = true;
// PC817 output pulls the pin LOW when the aux contact is closed.
const bool FEEDBACK_ACTIVE_LOW = true;

// --- timing ---
const uint32_t MIN_OFF_MS       = 60000UL;            // anti short-cycle lockout
const uint32_t MAX_RUN_MS       = 30UL * 60 * 1000;   // auto-off safety limit
const uint32_t FB_DEBOUNCE_MS   = 150;
const uint32_t MISMATCH_GRACE_MS = 2500;
const uint32_t HEARTBEAT_MS     = 10000;
const uint32_t MQTT_RETRY_MS    = 5000;

// Root CA for HiveMQ Cloud (Let's Encrypt ISRG Root X1).
// Download from https://letsencrypt.org/certs/isrgrootx1.pem and paste here.
// Leave empty to fall back to setInsecure() - BENCH TESTING ONLY.
const char *ROOT_CA = R"EOF(
-----BEGIN CERTIFICATE-----
PASTE_ISRG_ROOT_X1_HERE
-----END CERTIFICATE-----
)EOF";

// ============================ STATE ============================

WiFiClientSecure netClient;
PubSubClient     mqtt(netClient);

bool     relayOn    = false;   // what we commanded
bool     pendingOn  = false;   // an ON is queued, waiting for the lockout
uint32_t lastOffMs  = 0;       // when the relay last went OFF
uint32_t onSinceMs  = 0;       // when the relay last went ON

bool     fbStable   = false;   // debounced aux contact reading
bool     fbLastRaw  = false;
uint32_t fbChangeMs = 0;

uint32_t lastStateChangeMs = 0;
uint32_t lastHeartbeatMs   = 0;
uint32_t lastMqttTryMs     = 0;

const char *faultCode = "none";   // none | no_feedback | max_runtime

// ============================ RELAY ============================

inline int relayLevel(bool on) {
  return (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW;
}

void relayWrite(bool on) {
  digitalWrite(PIN_RELAY, relayLevel(on));
}

void publishState();

void setRelay(bool on) {
  if (on == relayOn) return;
  relayOn = on;
  relayWrite(on);
  uint32_t now = millis();
  if (on) onSinceMs = now; else lastOffMs = now;
  lastStateChangeMs = now;
  faultCode = "none";
  publishState();
}

uint32_t lockoutRemainingMs() {
  uint32_t elapsed = millis() - lastOffMs;
  return (elapsed >= MIN_OFF_MS) ? 0 : (MIN_OFF_MS - elapsed);
}

// ============================ COMMANDS ============================

void cmdOn()  { pendingOn = true;  faultCode = "none"; publishState(); }
void cmdOff() { pendingOn = false; setRelay(false); publishState(); }

void cmdRestart() {
  setRelay(false);      // this restarts the lockout timer
  pendingOn = true;     // and queues the ON that follows it
  publishState();
}

// ============================ FEEDBACK ============================

void updateFeedback() {
  bool raw = digitalRead(PIN_FEEDBACK);
  if (FEEDBACK_ACTIVE_LOW) raw = !raw;

  if (raw != fbLastRaw) {
    fbLastRaw = raw;
    fbChangeMs = millis();
  } else if (raw != fbStable && (millis() - fbChangeMs) >= FB_DEBOUNCE_MS) {
    fbStable = raw;
    publishState();
  }
}

// Contactor closed while we commanded OFF is NOT a fault - it means somebody
// used the manual override switch. The other direction is a real problem.
const char *currentMode() {
  if (!relayOn && fbStable) return "manual";
  return "remote";
}

void checkMismatch() {
  if (millis() - lastStateChangeMs < MISMATCH_GRACE_MS) return;
  if (relayOn && !fbStable) {
    if (strcmp(faultCode, "no_feedback") != 0) {
      faultCode = "no_feedback";
      publishState();
    }
  }
}

// ============================ MQTT ============================

void publishState() {
  if (!mqtt.connected()) return;

  char buf[360];
  snprintf(buf, sizeof(buf),
    "{\"relay\":%s,\"feedback\":%s,\"pending_on\":%s,\"mode\":\"%s\","
    "\"fault\":\"%s\",\"lockout_s\":%lu,\"run_s\":%lu,"
    "\"rssi\":%d,\"uptime_s\":%lu,\"ip\":\"%s\"}",
    relayOn   ? "true" : "false",
    fbStable  ? "true" : "false",
    pendingOn ? "true" : "false",
    currentMode(),
    faultCode,
    (unsigned long)(lockoutRemainingMs() / 1000),
    (unsigned long)(relayOn ? (millis() - onSinceMs) / 1000 : 0),
    WiFi.RSSI(),
    (unsigned long)(millis() / 1000),
    WiFi.localIP().toString().c_str());

  mqtt.publish(T_STATE, buf, true);   // retained
  lastHeartbeatMs = millis();
}

void onMessage(char *topic, byte *payload, unsigned int len) {
  char cmd[16] = {0};
  unsigned int n = (len < sizeof(cmd) - 1) ? len : sizeof(cmd) - 1;
  for (unsigned int i = 0; i < n; i++) cmd[i] = toupper((char)payload[i]);

  if      (!strcmp(cmd, "ON"))      cmdOn();
  else if (!strcmp(cmd, "OFF"))     cmdOff();
  else if (!strcmp(cmd, "RESTART")) cmdRestart();
  else if (!strcmp(cmd, "STATE"))   publishState();
}

void mqttConnect() {
  if (millis() - lastMqttTryMs < MQTT_RETRY_MS) return;
  lastMqttTryMs = millis();

  bool ok = mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS,
                         T_AVAIL, 1, true, "offline");
  if (ok) {
    mqtt.publish(T_AVAIL, "online", true);
    mqtt.subscribe(T_CMD, 1);
    publishState();
  }
}

// ============================ SETUP / LOOP ============================

void setup() {
  // Drive the pin to the OFF level BEFORE making it an output, so the relay
  // never twitches during boot. Add a physical 10k pull-up (active-low module)
  // or pull-down (active-high) on this pin - the ESP32 floats it during reset.
  relayWrite(false);
  pinMode(PIN_RELAY, OUTPUT);
  relayWrite(false);

  pinMode(PIN_FEEDBACK, INPUT_PULLUP);

  Serial.begin(115200);

  // Start locked out: if we just booted, mains may have only now come back.
  lastOffMs = millis();
  lastStateChangeMs = millis();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  if (strstr(ROOT_CA, "PASTE_ISRG_ROOT_X1_HERE")) {
    Serial.println("WARNING: no root CA set, TLS is unverified");
    netClient.setInsecure();
  } else {
    netClient.setCACert(ROOT_CA);
  }

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(30);
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqtt.connected()) mqttConnect();
    else                   mqtt.loop();
  }

  updateFeedback();

  // Release a queued ON once the lockout has expired.
  if (pendingOn && !relayOn && lockoutRemainingMs() == 0) {
    pendingOn = false;
    setRelay(true);
  }

  // Safety: never let the pump run longer than MAX_RUN_MS.
  if (relayOn && (millis() - onSinceMs) >= MAX_RUN_MS) {
    pendingOn = false;
    setRelay(false);
    faultCode = "max_runtime";
    publishState();
  }

  checkMismatch();

  if (millis() - lastHeartbeatMs >= HEARTBEAT_MS) publishState();
}
