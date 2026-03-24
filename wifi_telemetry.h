#pragma once

// ============================================================
//  wifi_telemetry.h  —  ESP32 Access Point + UDP broadcast
//  The ESP32 creates its own hotspot. Connect your laptop to it.
// ============================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include "telemetry.h"

// ---- Hotspot credentials — change these if you like ----
#define AP_SSID        "RocketTelemetry"
#define AP_PASSWORD    "launch123"       // min 8 chars, or "" for open network
#define AP_CHANNEL     1
#define AP_MAX_CLIENTS 1                 // only your laptop needs to connect

// ---- UDP — broadcast to the entire AP subnet ----
// ESP32 AP always uses 192.168.4.x — broadcast is 192.168.4.255
#define UDP_BROADCAST_IP  "192.168.4.255"
#define UDP_PORT          5005

// ---- Throttle: send every N samples over WiFi ----
// 1 = every sample (100 Hz), 2 = 50 Hz, 5 = 20 Hz
#define WIFI_SEND_EVERY_N  1

#define JSON_BUF_SIZE  256

static WiFiUDP udp;
static bool wifiReady = false;
static uint32_t wifiSendCount = 0;

// ----------------------------------------------------------
//  wifi_init()  — starts the AP and opens the UDP socket
//  Call once from setup()
// ----------------------------------------------------------
bool wifi_init() {
    Serial.printf("[WIFI] Starting hotspot \"%s\" ...\n", AP_SSID);

    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, AP_MAX_CLIENTS);

    if (!ok) {
        Serial.println(F("[WIFI] softAP() failed!"));
        wifiReady = false;
        return false;
    }

    delay(100);  // give AP a moment to settle
    Serial.printf("[WIFI] Hotspot up!  Connect to: \"%s\"\n", AP_SSID);
    Serial.printf("[WIFI] Password: \"%s\"\n", AP_PASSWORD);
    Serial.printf("[WIFI] ESP32 IP: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("[WIFI] Broadcasting telemetry on UDP port %d\n", UDP_PORT);

    udp.begin(UDP_PORT);
    wifiReady = true;
    return true;
}

// ----------------------------------------------------------
//  wifi_send()  — serialise packet to compact JSON and broadcast
//  Call every loop iteration after sensors_read()
// ----------------------------------------------------------
void wifi_send(const TelemetryPacket *pkt) {
    if (!wifiReady) return;

    wifiSendCount++;
    if (wifiSendCount % WIFI_SEND_EVERY_N != 0) return;

    char buf[JSON_BUF_SIZE];
    int len = snprintf(buf, sizeof(buf),
        "{"
        "\"t\":%u,"
        "\"temp\":%.2f,"
        "\"pres\":%.1f,"
        "\"alt\":%.2f,"
        "\"qw\":%.5f,\"qx\":%.5f,\"qy\":%.5f,\"qz\":%.5f,"
        "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
        "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f,"
        "\"va\":%.3f"
        "}",
        pkt->timestamp_ms,
        pkt->temperature_c,
        pkt->pressure_pa,
        pkt->altitude_m,
        pkt->quat_w, pkt->quat_x, pkt->quat_y, pkt->quat_z,
        pkt->accel_x, pkt->accel_y, pkt->accel_z,
        pkt->gyro_x,  pkt->gyro_y,  pkt->gyro_z,
        pkt->vertical_accel
    );

    udp.beginPacket(UDP_BROADCAST_IP, UDP_PORT);
    udp.write((const uint8_t *)buf, len);
    udp.endPacket();
}
