#pragma once
#include "datatypes.h"

// IMU tuning and calibration constants live in include/config.h.
// IMU_Read() returns the production estimator used by test/main.cpp:
// complementary roll/pitch plus Kalman-assisted yaw fusion.
void IMU_Init();
bool IMU_Calibrate_Gyro();
bool IMU_Run_Level_Calibration(float &roll_offset_deg, float &pitch_offset_deg);
bool IMU_Run_Mag_Calibration(float &offset_x, float &offset_y, float &offset_z,
                             float &scale_x, float &scale_y, float &scale_z);
void IMU_Read(IMUData_raw &data);
