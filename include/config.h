//////// config file for all pin assignments, tuning parameters and other constants

#pragma once
#include "datatypes.h"

///pin definintions

///Motor and servo pins

#define esc_pin 0
#define aileron_pin 1
#define elevator_pin 2
#define rudder_pin 3

//channels

const int esc_channel = 0;
const int aileron_channel = 1;
const int elevator_channel = 2;    
const int rudder_channel = 3;


// ============================================================
// IMU hardware settings

const int imu_sda_pin = 22;
const int imu_scl_pin = 20;
const uint32_t imu_i2c_freq_hz = 400000;

// IMU sensor configuration

const int imu_accel_fs_g = 4;
const int imu_gyro_fs_dps = 250;
const float imu_accel_rate_hz = 125.0f;
const float imu_gyro_rate_hz = 100.0f;
const uint8_t imu_accel_dlpf_mode = 5;
const uint8_t imu_gyro_dlpf_mode = 5;

// IMU calibration constants in AIRCRAFT BODY FRAME

// accel bias
const float ax_bias_g = 0.0f;
const float ay_bias_g = 0.0f;
const float az_bias_g = 0.0f;

// gyro bias
const float gx_bias_dps = 0.0f;
const float gy_bias_dps = 0.0f;
const float gz_bias_dps = 0.0f;

// mag bias
const float mx_bias_uT = 0.0f;
const float my_bias_uT = 0.0f;
const float mz_bias_uT = 0.0f;

// accel scale
const float ax_scale = 1.0f;
const float ay_scale = 1.0f;
const float az_scale = 1.0f;

// gyro scale
const float gx_scale = 1.0f;
const float gy_scale = 1.0f;
const float gz_scale = 1.0f;

// mag scale
const float mx_scale = 1.0f;
const float my_scale = 1.0f;
const float mz_scale = 1.0f;

// AHRS complementary filter tuning

const float roll_pitch_alpha = 0.98f;
const float yaw_alpha = 0.97f;


///tuning parameters
const float roll_kp = 1.0f;
const float roll_ki = 0.0f;
const float roll_kd = 0.1f;
const float max_roll_output = 500.0f;
const float max_roll_integral = 200.0f;

const float pitch_kp = 1.0f;
const float pitch_ki = 0.0f;
const float pitch_kd = 0.1f;
const float max_pitch_output = 500.0f;
const float max_pitch_integral = 200.0f;

const float yaw_kp = 1.0f;
const float yaw_ki = 0.0f;
const float yaw_kd = 0.1f;
const float max_yaw_output = 500.0f;
const float max_yaw_integral = 200.0f;

//Limits

const float max_roll_angle = 45.0f; 
const float max_pitch_angle = 15.0f; 

/// for althold pid
const float alt_kp = 1.0f;
const float alt_ki = 0.0f;
const float alt_kd = 0.1f;
const float max_alt_output = max_pitch_angle;
const float max_alt_integral = 10.0f;


//waypoints

const int num_waypoints = 2;               
const waypoint missionwaypoints[]= {
    
    {2.0536, 1.2189333333333333333, 100},
    {2, 2, 100},
    
};

