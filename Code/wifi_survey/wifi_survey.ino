/*
 * wifi_survey.ino  —  v2
 * ---------------------------------------------------------------
 * ESP32 WiFi site survey for the water pump remote control project.
 *
 * v2 changes:
 *   - added "m70" (ground-floor reception hotspot) as a scan target
 *   - connect test ENABLED and pointed at m70
 *   - added an internet reachability probe after association, so a
 *     good RSSI that carries no traffic is not mistaken for success
 *
 * NOTE: The ESP32 radio is 2.4 GHz ONLY. If your phone shows a network
 * here but this scan never lists it, that network is on 5 GHz, or
 * hidden, or blocked by the router's band/security settings.
 *
 * Serial Monitor: 115200 baud
 * ---------------------------------------------------------------
 */

#include <WiFi.h>
#include <WiFiClient.h>

// ---------------- CONFIG ----------------

// Networks to track. SSIDs must match EXACTLY (case sensitive).
const char* TARGETS[] = { "Damasy", "Nabil", "JOY" };
const int   NUM_TARGETS = sizeof(TARGETS) / sizeof(TARGETS[0]);

// Seconds to wait between scan rounds
const uint32_t ROUND_DELAY_MS = 3000;

// Onboard LED blink feedback, so you can survey with a power bank only
// (no laptop). Most ESP32 DevKit boards use GPIO2. Set to -1 to disable.
#define STATUS_LED_PIN 2

// ---- connection test ----
// Runs every CONNECT_TEST_EVERY rounds. Set to 0 to disable.
#define DO_CONNECT_TEST     1
#define CONNECT_TEST_EVERY  4
const char* TEST_SSID = "Damasy";
const char* TEST_PASS = "#Pop.Rt8@@#";

// ---- internet reachability probe ----
// Association alone does not prove traffic flows. This resolves a
// hostname and opens a TCP socket to it.
#define DO_INTERNET_TEST 1
const char* PROBE_HOST  = "www.google.com";
const uint16_t PROBE_PORT = 443;

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
void internetProbe() {
#if DO_INTERNET_TEST
  Serial.printf("  probe : resolving %s ... ", PROBE_HOST);

  IPAddress ip;
  uint32_t t0 = millis();
  if (!WiFi.hostByName(PROBE_HOST, ip)) {
    Serial.println("DNS FAILED");
    Serial.println("  -> associated but no working internet path.");
    return;
  }
  Serial.printf("%s (%lu ms)\n", ip.toString().c_str(), millis() - t0);

  WiFiClient client;
  client.setTimeout(5000);
  t0 = millis();
  if (client.connect(ip, PROBE_PORT)) {
    Serial.printf("  probe : TCP %u OK (%lu ms) -> internet reachable\n",
                  PROBE_PORT, millis() - t0);
    client.stop();
  } else {
    Serial.printf("  probe : TCP %u FAILED -> DNS works, traffic does not\n",
                  PROBE_PORT);
  }
#endif
}

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
    Serial.printf("  IP    : %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  RSSI  : %d dBm\n", WiFi.RSSI());
    Serial.printf("  BSSID : %s (ch %d)\n",
                  WiFi.BSSIDstr().c_str(), WiFi.channel());
    internetProbe();
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
  Serial.println("  ESP32 WiFi SITE SURVEY  v2");
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
#if DO_CONNECT_TEST
  Serial.printf("Connect  : %s, every %d rounds\n",
                TEST_SSID, CONNECT_TEST_EVERY);
#endif
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
  if (roundNum % CONNECT_TEST_EVERY == 0) connectTest();
#endif
}
