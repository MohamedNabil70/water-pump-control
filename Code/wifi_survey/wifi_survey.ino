/*
 * wifi_survey.ino
 * ---------------------------------------------------------------
 * ESP32 WiFi site survey for the water pump remote control project.
 *
 * Purpose: measure whether the ESP32 can actually see and reach the
 * home WiFi at the pump location, and how strong the signal is.
 *
 * NOTE: The ESP32 radio is 2.4 GHz ONLY. If your phone sees a network
 * here but this scan never lists it, that network is almost certainly
 * on the 5 GHz band, or hidden, or the ESP32 is being blocked by the
 * router's band/security settings.
 *
 * Serial Monitor: 115200 baud
 * ---------------------------------------------------------------
 */

#include <WiFi.h>

// ---------------- CONFIG ----------------

// Networks you care about. Must match the SSID EXACTLY (case sensitive).
const char* TARGETS[] = { "Nabil", "JOY" };
const int   NUM_TARGETS = sizeof(TARGETS) / sizeof(TARGETS[0]);

// Seconds to wait between scan rounds
const uint32_t ROUND_DELAY_MS = 3000;

// Onboard LED blink feedback, so you can survey with a power bank only
// (no laptop). Most ESP32 DevKit boards use GPIO2. Set to -1 to disable.
#define STATUS_LED_PIN 2

// Set to 1 and fill in the password to also test a real connection
// after every 5 scan rounds.
#define DO_CONNECT_TEST 0
const char* TEST_SSID = "Nabil";
const char* TEST_PASS = "#Pop.Rt8@@#";

// ---------------- STATE ----------------

struct TargetStats {
  uint32_t seen;
  int32_t  sum;
  int32_t  best;
  int32_t  worst;
  int32_t  last;
  uint8_t  channel;
  bool     everSeen;
};

TargetStats stats[8];
uint32_t roundNum = 0;

// ---------------- HELPERS ----------------

const char* encToStr(wifi_auth_mode_t e) {
  switch (e) {
    case WIFI_AUTH_OPEN:            return "OPEN";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-ENT";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3";
    default:                        return "UNKNOWN";
  }
}

// Quality tier: 0 = dead, 1 = very weak, 2 = marginal, 3 = good
int qualityTier(int32_t rssi) {
  if (rssi >= -70) return 3;
  if (rssi >= -80) return 2;
  if (rssi >= -88) return 1;
  return 0;
}

const char* tierLabel(int t) {
  switch (t) {
    case 3: return "GOOD      -> usable, MQTT should hold";
    case 2: return "MARGINAL  -> works sometimes, will drop";
    case 1: return "VERY WEAK -> unreliable, needs help";
    default: return "DEAD      -> do not rely on WiFi here";
  }
}

void blinkTier(int tier) {
#if STATUS_LED_PIN >= 0
  // 3 blinks = good, 2 = marginal, 1 = very weak, 0 = one long blink
  if (tier == 0) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(1200);
    digitalWrite(STATUS_LED_PIN, LOW);
    return;
  }
  for (int i = 0; i < tier; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(150);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(250);
  }
#endif
}

int targetIndex(const String& ssid) {
  for (int i = 0; i < NUM_TARGETS; i++) {
    if (ssid.equals(TARGETS[i])) return i;
  }
  return -1;
}

// ---------------- CONNECT TEST ----------------

#if DO_CONNECT_TEST
void connectTest() {
  Serial.println();
  Serial.println("=== CONNECT TEST ===");
  Serial.printf("Trying: %s\n", TEST_SSID);

  WiFi.begin(TEST_SSID, TEST_PASS);
  uint32_t start = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("CONNECTED in %lu ms\n", millis() - start);
    Serial.printf("  IP   : %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  RSSI : %d dBm\n", WiFi.RSSI());
    Serial.printf("  BSSID: %s (ch %d)\n",
                  WiFi.BSSIDstr().c_str(), WiFi.channel());
  } else {
    Serial.printf("FAILED after %lu ms (status code %d)\n",
                  millis() - start, WiFi.status());
    Serial.println("  Visible in scan but cannot connect usually means:");
    Serial.println("  wrong password, WPA3-only, or signal too weak to");
    Serial.println("  complete the handshake.");
  }

  WiFi.disconnect(true);
  delay(500);
  WiFi.mode(WIFI_STA);
  Serial.println("=== END CONNECT TEST ===");
  Serial.println();
}
#endif

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);
  delay(800);

#if STATUS_LED_PIN >= 0
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
#endif

  for (int i = 0; i < NUM_TARGETS; i++) {
    stats[i] = { 0, 0, -127, 0, 0, 0, false };
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(300);

  Serial.println();
  Serial.println("==============================================");
  Serial.println("  ESP32 WiFi SITE SURVEY");
  Serial.println("==============================================");
  Serial.printf("Chip     : %s rev %d\n",
                ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("MAC      : %s\n", WiFi.macAddress().c_str());
  Serial.printf("Band     : 2.4 GHz only\n");
  Serial.print ("Targets  : ");
  for (int i = 0; i < NUM_TARGETS; i++) {
    Serial.print(TARGETS[i]);
    if (i < NUM_TARGETS - 1) Serial.print(", ");
  }
  Serial.println();
  Serial.println("----------------------------------------------");
  Serial.println("RSSI reference:");
  Serial.println("  >= -70 dBm  GOOD");
  Serial.println("  -70..-80    MARGINAL");
  Serial.println("  -80..-88    VERY WEAK");
  Serial.println("  <  -88 dBm  DEAD");
  Serial.println("==============================================");
  Serial.println();
}

// ---------------- LOOP ----------------

void loop() {
  roundNum++;
  Serial.printf("\n----- ROUND %lu  (uptime %lu s) -----\n",
                roundNum, millis() / 1000);

  // async=false, showHidden=true
  int n = WiFi.scanNetworks(false, true);

  if (n <= 0) {
    Serial.println("No networks found at all.");
    Serial.println("If your phone sees networks here, the ESP32 antenna");
    Serial.println("or the board itself is the problem, not the distance.");
    blinkTier(0);
  } else {
    Serial.printf("Found %d networks:\n", n);
    Serial.println("  RSSI  CH  ENC        SSID");
    Serial.println("  ----  --  ---------  ----------------------");

    for (int i = 0; i < n; i++) {
      String ssid = WiFi.SSID(i);
      int32_t rssi = WiFi.RSSI(i);
      int idx = targetIndex(ssid);

      Serial.printf("  %4d  %2d  %-9s  %s%s\n",
                    rssi,
                    WiFi.channel(i),
                    encToStr(WiFi.encryptionType(i)),
                    ssid.length() ? ssid.c_str() : "<hidden>",
                    idx >= 0 ? "   <<< TARGET" : "");

      if (idx >= 0) {
        TargetStats& s = stats[idx];
        s.seen++;
        s.sum += rssi;
        s.last = rssi;
        s.channel = WiFi.channel(i);
        s.everSeen = true;
        if (rssi > s.best)  s.best  = rssi;
        if (rssi < s.worst || s.worst == 0) s.worst = rssi;
      }
    }
  }

  WiFi.scanDelete();

  // ---- summary for the target networks ----
  Serial.println();
  Serial.println("  TARGET SUMMARY (cumulative):");
  int bestTier = 0;

  for (int i = 0; i < NUM_TARGETS; i++) {
    TargetStats& s = stats[i];
    if (!s.everSeen) {
      Serial.printf("   %-12s NEVER SEEN in %lu rounds\n",
                    TARGETS[i], roundNum);
      continue;
    }
    int32_t avg = s.sum / (int32_t)s.seen;
    int tier = qualityTier(avg);
    if (tier > bestTier) bestTier = tier;

    Serial.printf("   %-12s last %4d | avg %4d | best %4d | worst %4d"
                  " | ch %2d | seen %lu/%lu\n",
                  TARGETS[i], s.last, avg, s.best, s.worst,
                  s.channel, s.seen, roundNum);
    Serial.printf("   %-12s %s\n", "", tierLabel(tier));
  }

  blinkTier(bestTier);
  delay(ROUND_DELAY_MS);

#if DO_CONNECT_TEST
  if (roundNum % 5 == 0) connectTest();
#endif
}
