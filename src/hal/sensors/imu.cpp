#include "hal/sensors/imu.h"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "config.h"
#include "hal/sensors/sensor_bus.h"

Adafruit_MPU6050 mpu;

namespace {

constexpr uint8_t kIMUAddress = MPU6050_I2CADDR_DEFAULT;

uint32_t g_last_time_us = 0;
uint32_t g_last_init_attempt_ms = 0;
bool g_imu_ready = false;
bool g_imu_missing_logged = false;
bool g_orientation_seeded = false;

void UpdateIMUAvailability(bool available) {
    if (available) {
        if (!g_imu_ready && SENSOR_STATUS_LOGGING_ENABLED) {
            Serial.println("MPU6050 available.");
        }

        g_imu_ready = true;
        g_imu_missing_logged = false;
        return;
    }

    if ((g_imu_ready || !g_imu_missing_logged) && SENSOR_STATUS_LOGGING_ENABLED) {
        Serial.println("MPU6050 unavailable. Continuing without IMU data.");
    }

    g_imu_ready = false;
    g_imu_missing_logged = true;
    g_orientation_seeded = false;
    g_last_time_us = 0;
}

bool TryInitializeIMU() {
    const uint32_t now_ms = millis();
    if (g_last_init_attempt_ms != 0 &&
        (now_ms - g_last_init_attempt_ms) < SENSOR_RECONNECT_INTERVAL_MS) {
        return g_imu_ready;
    }

    g_last_init_attempt_ms = now_ms;

    if (!SensorBus_Init()) {
        UpdateIMUAvailability(false);
        return false;
    }

    if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS))) {
        return false;
    }

    const bool begin_ok = mpu.begin(kIMUAddress, &Wire);
    if (begin_ok) {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // 8G helps with flight vibration spikes
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // Smooth motor noise before the complementary filter
    }

    SensorBus_Unlock();

    if (!begin_ok) {
        UpdateIMUAvailability(false);
        return false;
    }

    g_last_time_us = micros();
    g_orientation_seeded = false;
    UpdateIMUAvailability(true);
    return true;
}

bool ProbeIMULocked() {
    Wire.beginTransmission(kIMUAddress);
    return Wire.endTransmission(true) == 0;
}

} // namespace

void IMU_Init() {
    (void)TryInitializeIMU();
}

void IMU_Read(IMUData_raw &data) {
    if (!g_imu_ready && !TryInitializeIMU()) {
        data.healthy = false;
        return;
    }

    if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS))) {
        data.healthy = false;
        return;
    }

    if (!ProbeIMULocked()) {
        SensorBus_Unlock();
        UpdateIMUAvailability(false);
        data.healthy = false;
        return;
    }

    sensors_event_t a = {};
    sensors_event_t g = {};
    sensors_event_t temp = {};
    mpu.getEvent(&a, &g, &temp);
    SensorBus_Unlock();

    // 1. Read Raw Data
    data.accel_x = a.acceleration.x;
    data.accel_y = a.acceleration.y;
    data.accel_z = a.acceleration.z;
    data.gyro_x = g.gyro.x;
    data.gyro_y = g.gyro.y;
    data.gyro_z = g.gyro.z;

    // 2. Calculate time elapsed since last read (dt)
    const uint32_t now_us = micros();
    float dt = 0.0f;
    if (g_last_time_us != 0) {
        dt = static_cast<float>(now_us - g_last_time_us) / 1000000.0f;
    }
    g_last_time_us = now_us;

    // 3. Calculate absolute angles from Accelerometer (prone to vibration noise)
    float accel_roll = atan2(data.accel_y, data.accel_z) * 180 / PI;
    float accel_pitch = atan2(-data.accel_x, sqrt(data.accel_y * data.accel_y + data.accel_z * data.accel_z)) * 180 / PI;

    // 4. Complementary Filter: Trust the Gyro for short-term movement, Trust Accel for long-term gravity reference
    if (!g_orientation_seeded || dt <= 0.0f || dt > 0.5f) {
        data.roll = accel_roll;
        data.pitch = accel_pitch;
        g_orientation_seeded = true;
    } else {
        data.roll = 0.98f * (data.roll + data.gyro_x * dt) + 0.02f * accel_roll;
        data.pitch = 0.98f * (data.pitch + data.gyro_y * dt) + 0.02f * accel_pitch;
    }

    data.healthy = true;
    UpdateIMUAvailability(true);
}
