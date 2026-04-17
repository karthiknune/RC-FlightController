#pragma once
#include "datatypes.h"

void IMU_Init();
bool IMU_Calibrate_Gyro();
bool IMU_Run_Level_Calibration(float &roll_offset_deg, float &pitch_offset_deg);
void IMU_Read(IMUData_raw &data);
