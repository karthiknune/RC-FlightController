#pragma once
#include "datatypes.h"

class AHRS
{
private:
    float raw_accel_x, raw_accel_y, raw_accel_z;
    float raw_gyro_x, raw_gyro_y, raw_gyro_z;
    float raw_mag_x, raw_mag_y, raw_mag_z;

    void read_imu();
    void compute_attitude();

public:
    AHRS(); // constructor

    void init();
    void ahrs_update();
};
