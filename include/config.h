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

// I2C pins and speed (ESP32)
constexpr int   imu_sda_pin      = 21;
constexpr int   imu_scl_pin      = 22;
constexpr uint32_t imu_i2c_freq_hz = 400000; // 400 kHz

// Full-scale ranges selected in imu_init()
constexpr int   imu_accel_fs_g   = 4;    // {2,4,8,16}
constexpr int   imu_gyro_fs_dps  = 500;  // {250,500,1000,2000}

// Accelerometer biases (g)
constexpr float ax_bias_g = 0.0f;
constexpr float ay_bias_g = 0.0f;
constexpr float az_bias_g = 0.0f;
// Accelerometer scale
constexpr float ax_scale  = 1.0f;
constexpr float ay_scale  = 1.0f;
constexpr float az_scale  = 1.0f;

// Gyro biases (deg/s)
constexpr float gx_bias_dps = 0.0f;
constexpr float gy_bias_dps = 0.0f;
constexpr float gz_bias_dps = 0.0f;
// Gyro scale
constexpr float gx_scale    = 1.0f;
constexpr float gy_scale    = 1.0f;
constexpr float gz_scale    = 1.0f;

// Mag biases (µT)
constexpr float mx_bias_uT = 0.0f;
constexpr float my_bias_uT = 0.0f;
constexpr float mz_bias_uT = 0.0f;
// Mag scale
constexpr float mx_scale   = 1.0f;
constexpr float my_scale   = 1.0f;
constexpr float mz_scale   = 1.0f;



// Nominal AHRS update period (seconds), used when dt_s <= 0.
constexpr float ahrs_default_dt_s = 0.01f;   // 100 Hz

// Complementary filter gains:
// roll_pitch_alpha close to 1 → trust gyro more (short-term), accel for long-term.
// yaw_alpha same idea for yaw.
constexpr float roll_pitch_alpha = 0.98f;   // standard complementary filter choice[web:447][web:449]
constexpr float yaw_alpha        = 0.98f;


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

