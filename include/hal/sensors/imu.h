#pragma once
#include "datatypes.h"

void IMU_Init();
void IMU_Calibrate_Gyro();
void IMU_Read(IMUData_raw &data);
