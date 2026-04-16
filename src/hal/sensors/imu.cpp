// --- src/hal/sensors/imu.cpp ---
#include "hal/sensors/imu.h"
#include "hal/sensors/i2c_bus.h"
#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include "config.h"

Adafruit_ICM20948 icm;

void IMU_Init()
{
    i2c_bus_init();

    // Wake up the I2C bus
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_BUS_FREQUENCY_HZ);

    if (!i2c_bus_lock())
    {
        Serial.println("Failed to lock I2C bus for IMU init");
        while (1)
        {
            delay(10);
        }
    }

    // Initialize ICM-20948
    const bool imu_ready = icm.begin_I2C(ICM20948_I2CADDR_DEFAULT, &Wire);
    i2c_bus_unlock();

    if (!imu_ready)
    {
        Serial.println("Failed to find ICM-20948 chip");
        while (1)
        {
            delay(10);
        }
    }

    // Flight Controller specific hardware limits
    icm.setAccelRange(ICM20948_ACCEL_RANGE_8_G);    // 8G handles flight vibrations well
    icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);  // 500 degrees/sec is standard for RC
    icm.setMagDataRate(AK09916_MAG_DATARATE_50_HZ); // Keep mag updating fast enough for flight

    Serial.println("ICM-20948 Initialized Successfully.");
}

// --- NEW: Bias Storage ---
float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;

void IMU_Calibrate_Gyro()
{
    Serial.println("Calibrating Gyroscope... DO NOT MOVE THE AIRCRAFT!");

    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    const int num_samples = 200;

    for (int i = 0; i < num_samples; i++)
    {
        sensors_event_t a, g, temp, m;

        i2c_bus_lock();
        icm.getEvent(&a, &g, &temp, &m);
        i2c_bus_unlock();

        // Convert and accumulate (using the same body-frame transforms as IMU_Read)
        sum_gx += -(g.gyro.x * 57.2958f);
        sum_gy += -(g.gyro.y * 57.2958f);
        sum_gz += -(g.gyro.z * 57.2958f);

        delay(5); // Wait 5ms between samples
    }

    // Calculate the average bias
    gyro_bias_x = sum_gx / num_samples;
    gyro_bias_y = sum_gy / num_samples;
    gyro_bias_z = sum_gz / num_samples;

    Serial.printf("Gyro Biases [deg/s] -> X: %.2f | Y: %.2f | Z: %.2f\n",
                  gyro_bias_x, gyro_bias_y, gyro_bias_z);
}




// ====================================================================
// HARDWARE TO BODY FRAME TRANSFORMATION
// The physical IMU is mounted "backwards" (X arrow points to Tail).
// To convert this to Aircraft Body Frame (X=Front, Y=Right, Z=Down):
// X_body = -X_imu | Y_body = -Y_imu | Z_body = Z_imu
// ====================================================================

void IMU_Read(IMUData_raw &data)
{
    sensors_event_t a, g, temp, m;

    if (!i2c_bus_lock())
    {
        data.healthy = false;
        return;
    }

    icm.getEvent(&a, &g, &temp, &m);
    i2c_bus_unlock();

    // 1. Accelerometer (Body Frame transform)
    data.accel_x = -a.acceleration.x;
    data.accel_y = -a.acceleration.y;
    data.accel_z = a.acceleration.z;

    // 2. Gyroscope (Body Frame + Bias Subtraction)
    data.gyro_x = -(g.gyro.x * 57.2958f) - gyro_bias_x;
    data.gyro_y = -(g.gyro.y * 57.2958f) - gyro_bias_y;
    data.gyro_z = -(g.gyro.z * 57.2958f) - gyro_bias_z;

    // 3. Magnetometer (Hard Iron Offsets from config.h)
    float cal_mag_x = m.magnetic.x - MAG_OFFSET_X;
    float cal_mag_y = m.magnetic.y - MAG_OFFSET_Y;
    float cal_mag_z = m.magnetic.z - MAG_OFFSET_Z;

    data.mag_x = -cal_mag_x;
    data.mag_y = -cal_mag_y;
    data.mag_z = cal_mag_z;

    data.healthy = true;
}