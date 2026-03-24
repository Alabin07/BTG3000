// ============================================================
//  Rocket Telemetry  —  WiFi-only build (no SD card)
//  Hardware : ESP32 + BMP390 + BNO085
//  Libraries: Adafruit BMP3XX, Adafruit BNO08x
//             (install both via Arduino Library Manager)
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include "telemetry.h"
#include "buzzer.h"           // before wifi so boot blip fires ASAP
#include "wifi_telemetry.h"

// ---- Sensor objects ----
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO08x.h>

Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno085(BNO085_RESET);

static sh2_SensorValue_t sensorVal;
static uint32_t sampleCount = 0;

// ---- Carry-forward: last known good BNO085 values ----
// Avoids broadcasting zero-quaternion packets when the BNO085
// report isn't ready within the drain window.
static float last_quat_w = 1.0f, last_quat_x = 0.0f,
             last_quat_y = 0.0f, last_quat_z = 0.0f;
static float last_accel_x = 0.0f, last_accel_y = 0.0f, last_accel_z = 0.0f;
static float last_gyro_x  = 0.0f, last_gyro_y  = 0.0f, last_gyro_z  = 0.0f;

// ---- Sea-level pressure reference ----
// Update this to your local QNH (from a weather app) for accurate altitude.
// Standard atmosphere = 101325 Pa. Typical range: 97000–104000 Pa.
static float seaLevelPressure_Pa = 102425.0f;


// ==============================================================
//  SETUP
// ==============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    delay(100);
    Serial.println(F("\n============================="));
    Serial.println(F("  Rocket Telemetry  (WiFi AP)"));
    Serial.println(F("=============================\n"));

    // Buzzer first — single blip confirms MCU is alive before anything else
    buzzer_init();

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_FREQ);

    if (!sensors_init()) {
        Serial.println(F("\n[FATAL] Sensor init failed — check wiring. Halting."));
        buzzer_sensor_fail();
        delay(500);
        while (true) { buzzer_fatal(); }
    }

    if (!wifi_init()) {
        Serial.println(F("[WARN] WiFi AP failed — no live telemetry."));
        buzzer_wifi_fail();
        delay(1000);
        // Not fatal — carry on without WiFi
    }

    buzzer_ok();
    Serial.println(F("\n[OK] Ready — streaming telemetry.\n"));
}


// ==============================================================
//  MAIN LOOP
// ==============================================================
void loop() {
    static uint32_t lastSample = 0;
    uint32_t now = millis();

    if ((now - lastSample) < SAMPLE_INTERVAL_MS) return;
    lastSample = now;

    TelemetryPacket pkt = {};
    pkt.timestamp_ms = now;

    if (!sensors_read(&pkt)) {
        Serial.println(F("[WARN] Sensor read failed, skipping"));
        return;
    }

    wifi_send(&pkt);
    sampleCount++;

    // Print a summary to Serial every 10 samples (10 Hz) so the monitor
    // stays readable without flooding.
    if (sampleCount % 10 == 0) {
        Serial.printf(
            "[%7.2f s]  Alt=%6.1f m  T=%5.1f°C  P=%7.0f Pa  "
            "a=(%5.2f, %5.2f, %5.2f) m/s²  "
            "q=(%6.4f, %6.4f, %6.4f, %6.4f)  "
            "va=%5.2f m/s²\n",
            now / 1000.0f,
            pkt.altitude_m, pkt.temperature_c, pkt.pressure_pa,
            pkt.accel_x, pkt.accel_y, pkt.accel_z,
            pkt.quat_w, pkt.quat_x, pkt.quat_y, pkt.quat_z,
            pkt.vertical_accel
        );
    }
}


// ==============================================================
//  SENSOR INITIALISATION
// ==============================================================
bool sensors_init() {

    // ---- BMP390 ----
    Serial.print(F("[INIT] BMP390 ... "));
    if (!bmp.begin_I2C(BMP390_I2C_ADDR)) {
        Serial.println(F("NOT FOUND"));
        Serial.println(F("  → Check SDA/SCL wiring and I2C address (SDO pin)"));
        return false;
    }
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_100_HZ);
    Serial.println(F("OK"));

    // ---- BNO085 ----
    Serial.print(F("[INIT] BNO085 ... "));
    if (!bno085.begin_I2C(BNO085_I2C_ADDR)) {
        Serial.println(F("NOT FOUND"));
        Serial.println(F("  → Check SDA/SCL wiring and I2C address (SA0 pin)"));
        return false;
    }
    if (!bno085.enableReport(SH2_ROTATION_VECTOR,      10000)) {
        Serial.println(F("FAILED: rotation vector report")); return false;
    }
    if (!bno085.enableReport(SH2_LINEAR_ACCELERATION,  10000)) {
        Serial.println(F("FAILED: linear accel report")); return false;
    }
    if (!bno085.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) {
        Serial.println(F("FAILED: gyro report")); return false;
    }
    Serial.println(F("OK"));

    return true;
}


// ==============================================================
//  SENSOR READ  —  fills one TelemetryPacket
// ==============================================================
bool sensors_read(TelemetryPacket *pkt) {

    // ---- BMP390 ----
    if (!bmp.performReading()) {
        Serial.println(F("[WARN] BMP390 read timeout"));
        return false;
    }
    pkt->temperature_c = (float)bmp.temperature;
    pkt->pressure_pa   = (float)bmp.pressure;
    pkt->altitude_m    = pressure_to_altitude(pkt->pressure_pa, seaLevelPressure_Pa);

    // ---- BNO085 — drain the report queue (up to 5 ms) ----
    bool gotQuat = false, gotAccel = false, gotGyro = false;
    uint32_t deadline = millis() + 5;

    while (millis() < deadline && !(gotQuat && gotAccel && gotGyro)) {
        if (!bno085.getSensorEvent(&sensorVal)) continue;

        switch (sensorVal.sensorId) {
            case SH2_ROTATION_VECTOR:
                last_quat_w = sensorVal.un.rotationVector.real;
                last_quat_x = sensorVal.un.rotationVector.i;
                last_quat_y = sensorVal.un.rotationVector.j;
                last_quat_z = sensorVal.un.rotationVector.k;
                gotQuat = true;
                break;
            case SH2_LINEAR_ACCELERATION:
                last_accel_x = sensorVal.un.linearAcceleration.x;
                last_accel_y = sensorVal.un.linearAcceleration.y;
                last_accel_z = sensorVal.un.linearAcceleration.z;
                gotAccel = true;
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                last_gyro_x = sensorVal.un.gyroscope.x;
                last_gyro_y = sensorVal.un.gyroscope.y;
                last_gyro_z = sensorVal.un.gyroscope.z;
                gotGyro = true;
                break;
        }
    }

    // ---- Always use last known good values ----
    // If a report didn't arrive this cycle, the previous frame's data is
    // far better than zeros. Quaternion starts as identity (1,0,0,0).
    pkt->quat_w  = last_quat_w;  pkt->quat_x = last_quat_x;
    pkt->quat_y  = last_quat_y;  pkt->quat_z = last_quat_z;
    pkt->accel_x = last_accel_x; pkt->accel_y = last_accel_y;
    pkt->accel_z = last_accel_z;
    pkt->gyro_x  = last_gyro_x;  pkt->gyro_y  = last_gyro_y;
    pkt->gyro_z  = last_gyro_z;

    // ---- Vertical acceleration (project linear accel onto world up-axis) ----
    float w = pkt->quat_w, x = pkt->quat_x,
          y = pkt->quat_y, z = pkt->quat_z;
    float upX = 2.0f*(x*z + w*y);
    float upY = 2.0f*(y*z - w*x);
    float upZ = 1.0f - 2.0f*(x*x + y*y);
    pkt->vertical_accel = pkt->accel_x*upX + pkt->accel_y*upY + pkt->accel_z*upZ;

    return true;
}


// ==============================================================
//  BAROMETRIC ALTITUDE  (hypsometric formula)
// ==============================================================
float pressure_to_altitude(float pressure_pa, float sea_level_pa) {
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1902949f));
}
