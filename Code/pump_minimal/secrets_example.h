#pragma once

// Template for secrets.h - this file IS committed, the real secrets.h is not.
//
// Setup: copy this file to secrets.h in the same sketch folder and fill in
// your own values. secrets.h is listed in .gitignore, so your credentials
// stay on your machine.
//
//     cp secrets_example.h secrets.h

#define WIFI_SSID_1 "YOUR_PRIMARY_WIFI_SSID"
#define WIFI_PASS_1 "YOUR_PRIMARY_WIFI_PASSWORD"

#define WIFI_SSID_2 "YOUR_BACKUP_WIFI_SSID"
#define WIFI_PASS_2 "YOUR_BACKUP_WIFI_PASSWORD"

#define MQTT_HOST "your-cluster-id.s1.eu.hivemq.cloud"
#define MQTT_PORT 8883
#define MQTT_USER "YOUR_MQTT_USERNAME"
#define MQTT_PASS "YOUR_MQTT_PASSWORD"
