#pragma once
#include "datatypes.h"

void IMU_Init();

void IMU_Read(IMUData_raw &data);


#pragma once

#include <Arduino.h>
#include <Wire.h>

// ============================================================
// One full raw IMU sample in AIRCRAFT BODY FRAME
// ============================================================

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

    uint32_t timestamp_ms;
};


// IMU public API
bool imu_init();
bool imu_read_sample(imu_sample_t &sample);

bool imu_is_online();
bool imu_mag_is_online();
