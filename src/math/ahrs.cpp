////////// raw imu data to AHRS
// --- src/math/ahrs.cpp ---
#include "math/ahrs.h"
#include <Arduino.h>
#include <cmath>
#include "config.h"
#include "hal/sensors/imu.h"
AHRS::AHRS()
{
    last_time = 0;
    is_initialized = false;
}

void AHRS::init()
{
    last_time = micros();
    is_initialized = true;
}

void AHRS::update(const IMUData_raw &raw, IMUData_filtered &filtered)
{
    if (!raw.healthy)
        return;

    // 1. Calculate precise delta time (dt) in seconds
    unsigned long current_time = micros();
    float dt = (current_time - last_time) / 1000000.0f;
    last_time = current_time;

    // Safety check: Prevent massive dt jumps if task was suspended
    if (dt > 0.1f)
        dt = 0.01f;

    // 2. Calculate absolute angles from Accelerometer (Gravity reference)
    // Formula: atan2 gives the angle based on the gravity vector on remaining axes
    float accel_roll = atan2(raw.accel_y, raw.accel_z) * 180.0f / PI;
    float accel_pitch = atan2(-raw.accel_x, sqrt(raw.accel_y * raw.accel_y + raw.accel_z * raw.accel_z)) * 180.0f / PI;

    // // 3. The Complementary Filter
    // 98% trust in Gyro (fast, but drifts over time)
    // 2% trust in Accel (noisy from motor vibrations, but absolute over time)
    filtered.roll = 0.98f * (filtered.roll + raw.gyro_x * dt) + 0.02f * accel_roll;
    filtered.pitch = 0.98f * (filtered.pitch + raw.gyro_y * dt) + 0.02f * accel_pitch;
    // filtered.roll -= LEVEL_ROLL_OFFSET;
    // filtered.pitch -= LEVEL_PITCH_OFFSET;

    // 4. Yaw (Placeholder integration)
    // Note: To implement full 3D tilt-compensated compass yaw, we use the magnetometer here.
    // For now, we integrate the Z-gyro, which will be overwritten by GPS heading in main.cpp
    filtered.yaw = filtered.yaw + (raw.gyro_z * dt);
}