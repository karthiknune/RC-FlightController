///////temp using MPU6050

#include "hal/sensors/imu.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


Adafruit_MPU6050 mpu;
unsigned long lastTime = 0;

void IMU_Init() {
    Wire.begin(); // Uses default SDA/SCL pins for ESP32 Feather V2
    
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10); // Halt execution if IMU isn't found
        }
    }
    
    // Flight controller specific settings
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // 8G is good to handle flight vibrations
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // Low-pass filter to smooth motor noise
    
    lastTime = micros();
    Serial.println("IMU Initialized Successfully.");
}

void IMU_Read(IMUData_raw &data) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 1. Read Raw Data
    data.accel_x = a.acceleration.x;
    data.accel_y = a.acceleration.y;
    data.accel_z = a.acceleration.z;
    data.gyro_x = g.gyro.x;
    data.gyro_y = g.gyro.y;
    data.gyro_z = g.gyro.z;

    // 2. Calculate time elapsed since last read (dt)
    float dt = (micros() - lastTime) / 1000000.0;
    lastTime = micros();

    // 3. Calculate absolute angles from Accelerometer (prone to vibration noise)
    float accel_roll = atan2(data.accel_y, data.accel_z) * 180 / PI;
    float accel_pitch = atan2(-data.accel_x, sqrt(data.accel_y * data.accel_y + data.accel_z * data.accel_z)) * 180 / PI;

    // 4. Complementary Filter: Trust the Gyro for short-term movement, Trust Accel for long-term gravity reference
    data.roll = 0.98 * (data.roll + data.gyro_x * dt) + 0.02 * accel_roll;
    data.pitch = 0.98 * (data.pitch + data.gyro_y * dt) + 0.02 * accel_pitch;
    
    data.healthy = true;
}
