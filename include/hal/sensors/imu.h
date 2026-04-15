// #pragma once
// #include "datatypes.h"

// void IMU_Init();

// void IMU_Read(IMUData_raw &data);
#pragma once

#include <Arduino.h>

// ============================================================================
// RAW IMU SAMPLE TYPE (BODY FRAME)
// ============================================================================
// One full measurement of accelerometer, gyroscope, and magnetometer data.
//
// All values are expressed in the AIRCRAFT BODY FRAME using the mounting
// conventions documented for the ICM-20948 + AK09916 on this aircraft.
// ============================================================================
struct imu_sample_t {
    // Accelerometer in g (9.81 m/s^2 = 1 g).
    float ax_g;
    float ay_g;
    float az_g;

    // Gyroscope in degrees per second.
    float gx_dps;
    float gy_dps;
    float gz_dps;

    // Magnetometer in microtesla.
    float mx_uT;
    float my_uT;
    float mz_uT;

    // Timestamp (ms) when this sample was captured.
    uint32_t timestamp_ms;
};

// ============================================================================
// IMU PUBLIC API
// ============================================================================
//
// The rest of the project should talk to the IMU ONLY through this interface.
// No other files should touch I2C, the Adafruit driver, or chip registers.
// ============================================================================

// Initialize the ICM-20948 + AK09916 IMU hardware.
//
// Returns:
//   true  -> IMU initialized and is ready to use.
//   false -> initialization failed (wiring, address, etc.).
bool imu_init();

// Read a single raw IMU sample in AIRCRAFT BODY FRAME.
//
// Parameters:
//   sample (out) -> filled with accel / gyro / mag data on success.
//
// Returns:
//   true  -> sample filled with fresh data.
//   false -> read failed (IMU offline, bus error, etc.).
bool imu_read_sample(imu_sample_t &sample);

// Quick health checks for higher-level code (optional).
bool imu_is_online();
bool imu_mag_is_online();
