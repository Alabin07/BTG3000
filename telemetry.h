#pragma once

// ============================================================
//  telemetry.h  —  Pin definitions, config, shared struct
// ============================================================

// ---- I2C (BMP390 + BNO085 daisy-chain via StemmaQT) ----
#define I2C_SDA       21
#define I2C_SCL       22
#define I2C_FREQ      400000   // 400 kHz

// ---- Buzzer ----
#define BUZZER_PIN    25     // change to whatever GPIO you wire it to
#define BUZZER_CHAN   0      // LEDC channel (0–15, any free one)

// ---- I2C Addresses ----
#define BMP390_I2C_ADDR   0x77  // SDO → 3V3 (default); 0x76 if SDO → GND
#define BNO085_I2C_ADDR   0x4A  // SA0 → GND (default); 0x4B if SA0 → 3V3

// ---- BNO085 reset pin (-1 = not wired) ----
#define BNO085_RESET  -1

// ---- Sampling ----
#define SAMPLE_RATE_HZ     100
#define SAMPLE_INTERVAL_MS (1000 / SAMPLE_RATE_HZ)

// ---- Telemetry packet ----
typedef struct {
    uint32_t timestamp_ms;

    // BMP390
    float temperature_c;
    float pressure_pa;
    float altitude_m;

    // BNO085 - Rotation Vector (fused quaternion, body→world)
    float quat_w, quat_x, quat_y, quat_z;

    // BNO085 - Linear Acceleration (gravity removed, m/s²)
    float accel_x, accel_y, accel_z;

    // BNO085 - Calibrated Gyroscope (rad/s)
    float gyro_x, gyro_y, gyro_z;

    // Derived: acceleration projected onto world-frame up axis
    float vertical_accel;
} TelemetryPacket;

// ---- Function prototypes ----
bool sensors_init();
bool sensors_read(TelemetryPacket *pkt);
float pressure_to_altitude(float pressure_pa, float sea_level_pa);
