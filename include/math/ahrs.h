#pragma once

#include "hal/sensors/imu.h"

// ============================================================================
// AHRS (ATTITUDE AND HEADING REFERENCE SYSTEM)
// ============================================================================
// This module sits ABOVE the IMU driver.
//
// Responsibilities:
//   - pull raw imu_sample_t from the IMU layer
//   - run attitude estimation (currently a complementary filter)
//   - expose fused roll / pitch / yaw in degrees
//   - optionally expose the last raw sample used by the filter
//
// The rest of the system should not need to know which exact filter is used
// internally; you can swap this for Mahony/Madgwick/EKF later.
// ============================================================================


// Fused attitude representation in degrees.
//
// Convention:
//   roll  -> rotation about body X (right wing down positive)
//   pitch -> rotation about body Y (nose up positive)
//   yaw   -> rotation about body Z (nose right / heading)
struct attitude_rpy_t {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
};

// Initialize AHRS internal state.
//
// Must be called once after imu_init() and before the first ahrs_update().
void ahrs_init();

// Run one AHRS update step.
//
// Parameters:
//   dt_s = time step in seconds since the previous update.
//
// Internally this will:
//   - read one imu_sample_t from imu_read_sample()
//   - compute roll/pitch from accelerometer
//   - integrate gyro rates
//   - compute tilt-compensated yaw from magnetometer
//   - fuse everything using complementary filter constants from config.h
//
// Returns:
//   true  -> new IMU data was used and the estimate was updated.
//   false -> IMU read failed or AHRS not initialized.
bool ahrs_update(float dt_s);

// Get the latest fused attitude estimate (degrees).
attitude_rpy_t ahrs_get_attitude();

// Get the latest raw IMU sample used by the filter.
//
// This is useful if you want:
//   - fused attitude for control
//   - raw accel/gyro/mag for logging or debug
imu_sample_t ahrs_get_last_raw_sample();




// #pragma once
// #include "datatypes.h"

// class AHRS
// {
// private:
//     float raw_accel_x, raw_accel_y, raw_accel_z;
//     float raw_gyro_x, raw_gyro_y, raw_gyro_z;

//     void read_imu();
//     void compute_attitude();
// public:
//      AHRS();//constructor

//      void init();
//      void ahrs_update();

// };
