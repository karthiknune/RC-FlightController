//Previous code - 
// #pragma once
// #include "datatypes.h"

// void IMU_Init();

// void IMU_Read(IMUData_raw &data);

//New code 
#pragma once

struct imu_sample_t {
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float mx_uT;
    float my_uT;
    float mz_uT;
};

bool imu_init();                 // wraps imu.begin + configure + calibration
bool imu_read_sample(imu_sample_t &sample);  // wraps imu.readSample()
