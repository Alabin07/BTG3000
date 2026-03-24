#pragma once

// ============================================================
//  buzzer.h  —  Passive buzzer tones via ESP32 LEDC
//
//  Wiring:
//    BUZZER_PIN → buzzer positive leg
//    GND        → buzzer negative leg
//
//  Use a PASSIVE buzzer (bare metal disc or small speaker).
//  Active buzzers (with built-in oscillator) will only click
//  and won't produce different tones.
// ============================================================

#include <Arduino.h>
#include "telemetry.h"

static bool buzzerReady = false;

// ----------------------------------------------------------
//  Internal helpers
// ----------------------------------------------------------
static void _tone(uint32_t freq, uint32_t dur_ms) {
    if (!buzzerReady) return;
    ledcWriteTone(BUZZER_PIN, freq);
    delay(dur_ms);
    ledcWriteTone(BUZZER_PIN, 0);   // silence
}

static void _gap(uint32_t ms) {
    delay(ms);
}

// ----------------------------------------------------------
//  buzzer_init()  —  call once at the very start of setup(),
//  before any sensor init so you hear if the MCU booted at all
// ----------------------------------------------------------
void buzzer_init() {
    // ESP32 Arduino core v3.x — ledcAttach replaces ledcSetup+ledcAttachPin
    ledcAttach(BUZZER_PIN, 2000, 8);
    buzzerReady = true;

    // Single short blip — "MCU alive"
    _tone(1000, 80);
}

// ----------------------------------------------------------
//  buzzer_ok()  —  all sensors + WiFi started successfully
//  Two rising beeps — cheerful "ready" sound
// ----------------------------------------------------------
void buzzer_ok() {
    _tone(1200, 100); _gap(60);
    _tone(1800, 180);
}

// ----------------------------------------------------------
//  buzzer_sensor_fail()  —  BMP390 or BNO085 not found
//  Three low descending blips — "something is wrong"
// ----------------------------------------------------------
void buzzer_sensor_fail() {
    _tone(600, 200); _gap(80);
    _tone(450, 200); _gap(80);
    _tone(300, 400);
}

// ----------------------------------------------------------
//  buzzer_wifi_fail()  —  WiFi / AP failed to start
//  Two low beeps — less severe than sensor fail
// ----------------------------------------------------------
void buzzer_wifi_fail() {
    _tone(500, 200); _gap(100);
    _tone(500, 200);
}

// ----------------------------------------------------------
//  buzzer_fatal()  —  unrecoverable error, system halted
//  Repeating SOS-like pattern — hard to ignore
//  Call this inside your while(true) halt loop.
// ----------------------------------------------------------
void buzzer_fatal() {
    // Three short
    for (int i = 0; i < 3; i++) { _tone(800, 120); _gap(80); }
    _gap(150);
    // Three long
    for (int i = 0; i < 3; i++) { _tone(800, 350); _gap(100); }
    _gap(150);
    // Three short
    for (int i = 0; i < 3; i++) { _tone(800, 120); _gap(80); }
    _gap(600);
}