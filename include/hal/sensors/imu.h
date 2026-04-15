#pragma once
#include "datatypes.h"
#include <Arduino.h>

// ============================================================================
// RAW IMU SAMPLE TYPE
// ============================================================================
// One full measurement of accelerometer, gyroscope, and magnetometer data.
//
// All values are expressed in the AIRCRAFT BODY FRAME.
// If the sensor board is mounted with some rotation relative to the aircraft,
// we correct for that in src/hal/sensors/imu.cpp before filling this struct.
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

    // Timestamp (in milliseconds) when this sample was captured.
    uint32_t timestamp_ms;
};

// ============================================================================
// PUBLIC IMU API
// ============================================================================
// This is the ONLY interface the rest of the codebase should use to interact
// with the physical IMU hardware.
// ============================================================================

// Initialize the IMU hardware and verify that it is present.
//
// Returns:
//   true  -> IMU initialized correctly and is ready for use.
//   false -> IMU could not be initialized (hardware problem, wiring issue, etc.).
bool imu_init();

// Read a single raw IMU sample in AIRCRAFT BODY FRAME.
//
// Parameters:
//   sample (out) -> filled with the latest accel / gyro / mag values.
//
// Returns:
//   true  -> a fresh sample was read successfully.
//   false -> read failed (sensor not responding, bus error, etc.).
bool imu_read_sample(imu_sample_t &sample);

// Quick health checks (optional convenience for higher-level code).
bool imu_is_online();
bool imu_mag_is_online();
