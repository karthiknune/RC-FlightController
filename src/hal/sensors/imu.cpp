#include "hal/sensors/imu.h"
#include "config.h"

#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

// ============================================================================
// IMU HARDWARE LAYER USING ADAFRUIT_ICM20948
// ============================================================================
// This file is the ONLY place that talks directly to the ICM-20948 + AK09916
// hardware. Everywhere else in the project should use this small API:
//
//   bool imu_init();
//   bool imu_read_sample(imu_sample_t &sample);
//   bool imu_is_online();
//   bool imu_mag_is_online();
//
// Internally this file:
//   - starts the ESP32 I2C peripheral on the configured pins
//   - initializes the Adafruit_ICM20948 driver
//   - configures accel / gyro / mag ranges and rates
//   - reads accel, gyro, mag in the sensor frame
//   - converts those readings into the AIRCRAFT BODY FRAME
//       * using the SAME orientation logic you had in ICM20948BodyFrame.cpp:
//           AK frame  -> ICM frame
//           ICM frame -> BODY frame
//   - applies calibration constants in BODY FRAME
//
// It does NOT compute roll/pitch/yaw; that is done by src/math/ahrs.cpp.
// ============================================================================


// ---------------------------------------------------------------------------
// GLOBAL ADAFRUIT IMU OBJECT + STATUS FLAGS
// ---------------------------------------------------------------------------

static Adafruit_ICM20948 icm;     // Represents the physical ICM-20948 IMU.

static bool icm_online_flag = false; // True if ICM core init succeeded.
static bool mag_online_flag = false; // True if mag path is considered OK.


// ============================================================================
// INTERNAL HELPER: SENSOR FRAME -> BODY FRAME + CALIBRATION
// ============================================================================
// This function takes the four Adafruit sensor events:
//
//   accel_event.acceleration.{x,y,z}  [m/s^2]
//   gyro_event.gyro.{x,y,z}          [rad/s]
//   mag_event.magnetic.{x,y,z}       [microtesla]
//
// and converts them into one imu_sample_t in the AIRCRAFT BODY FRAME:
//
//   - accel in g
//   - gyro in degrees per second
//   - mag in microtesla
//
// Orientation mapping is exactly what you already derived in your driver:
//
//   1) Magnetometer AK frame -> ICM frame:
//        x_icm =  x_ak
//        y_icm = -y_ak
//        z_icm = -z_ak
//
//   2) ICM frame -> BODY frame (your mounting note):
//        ICM +X points opposite aircraft forward
//        ICM +Y matches aircraft +Y
//
//      So for ALL three sensors (accel, gyro, mag):
//        x_body = -x_icm
//        y_body =  y_icm
//        z_body =  z_icm
//
// Finally, we apply calibration biases and scales in the BODY frame.
// ============================================================================
static void fill_body_frame_sample(const sensors_event_t &accel_event,
                                   const sensors_event_t &gyro_event,
                                   const sensors_event_t &mag_event,
                                   imu_sample_t &out_sample)
{
    // ------------------------------------------------------------------------
    // Step 1: convert Adafruit units -> ICM frame in convenient units.
    // ------------------------------------------------------------------------
    // Adafruit accelerometer is in m/s^2; convert to g.
    const float g_per_ms2   = 1.0f / 9.80665f;  // 1 g = 9.80665 m/s^2
    // Adafruit gyro is in rad/s; convert to degrees per second.
    const float deg_per_rad = 180.0f / PI;

    // Accelerometer: sensor/ICM frame in g.
    float ax_icm_g = accel_event.acceleration.x * g_per_ms2;
    float ay_icm_g = accel_event.acceleration.y * g_per_ms2;
    float az_icm_g = accel_event.acceleration.z * g_per_ms2;

    // Gyroscope: sensor/ICM frame in deg/s.
    float gx_icm_dps = gyro_event.gyro.x * deg_per_rad;
    float gy_icm_dps = gyro_event.gyro.y * deg_per_rad;
    float gz_icm_dps = gyro_event.gyro.z * deg_per_rad;

    // Magnetometer: Adafruit gives AK frame in microtesla.
    // Apply AK -> ICM mapping you used before:
    //   x_icm =  x_ak
    //   y_icm = -y_ak
    //   z_icm = -z_ak
    float mx_icm_uT = mag_event.magnetic.x;
    float my_icm_uT = -mag_event.magnetic.y;
    float mz_icm_uT = -mag_event.magnetic.z;

    // ------------------------------------------------------------------------
    // Step 2: ICM frame -> AIRCRAFT BODY FRAME (same as your old code).
    // ------------------------------------------------------------------------
    // Mounting note (from ICM20948BodyFrame.cpp):
    //   ICM +X points opposite aircraft forward.
    //   ICM +Y matches aircraft +Y.
    //
    // So:
    //   x_body = -x_icm
    //   y_body =  y_icm
    //   z_body =  z_icm
    //
    // Apply this to accel, gyro, and mag.
    float ax_body = -ax_icm_g;
    float ay_body =  ay_icm_g;
    float az_body =  az_icm_g;

    float gx_body = -gx_icm_dps;
    float gy_body =  gy_icm_dps;
    float gz_body =  gz_icm_dps;

    float mx_body = -mx_icm_uT;
    float my_body =  my_icm_uT;
    float mz_body =  mz_icm_uT;

    // ------------------------------------------------------------------------
    // Step 3: apply calibration constants from config.h in BODY FRAME.
    // ------------------------------------------------------------------------
    // accel_bias_g, accel_scale, gyro_bias_dps, gyro_scale, mag_bias_uT, mag_scale
    // are all defined in config.h and are assumed to be already in BODY FRAME.
    out_sample.ax_g = (ax_body - ax_bias_g) * ax_scale;
    out_sample.ay_g = (ay_body - ay_bias_g) * ay_scale;
    out_sample.az_g = (az_body - az_bias_g) * az_scale;

    out_sample.gx_dps = (gx_body - gx_bias_dps) * gx_scale;
    out_sample.gy_dps = (gy_body - gy_bias_dps) * gy_scale;
    out_sample.gz_dps = (gz_body - gz_bias_dps) * gz_scale;

    out_sample.mx_uT = (mx_body - mx_bias_uT) * mx_scale;
    out_sample.my_uT = (my_body - my_bias_uT) * my_scale;
    out_sample.mz_uT = (mz_body - mz_bias_uT) * mz_scale;

    // ------------------------------------------------------------------------
    // Step 4: timestamp the sample.
    // ------------------------------------------------------------------------
    out_sample.timestamp_ms = millis();
}


// ============================================================================
// PUBLIC IMU API IMPLEMENTATION
// ============================================================================

bool imu_init()
{
    // ------------------------------------------------------------------------
    // Step 1: start the I2C peripheral on pins defined in config.h.
    // ------------------------------------------------------------------------
    Wire.begin(imu_sda_pin, imu_scl_pin, imu_i2c_freq_hz);

    // ------------------------------------------------------------------------
    // Step 2: initialize the Adafruit ICM20948 driver over I2C.
    // ------------------------------------------------------------------------
    if (!icm.begin_I2C(ICM20948_I2CADDR_DEFAULT, &Wire)) {
        icm_online_flag = false;
        mag_online_flag = false;
        return false;
    }

    // If we get here, the ICM core responded correctly.
    icm_online_flag = true;

    // ------------------------------------------------------------------------
    // Step 3: configure accelerometer full-scale range.
    // ------------------------------------------------------------------------
    icm20948_accel_range_t accel_range_enum = ICM20948_ACCEL_RANGE_4_G;
    if (imu_accel_fs_g == 2) {
        accel_range_enum = ICM20948_ACCEL_RANGE_2_G;
    } else if (imu_accel_fs_g == 4) {
        accel_range_enum = ICM20948_ACCEL_RANGE_4_G;
    } else if (imu_accel_fs_g == 8) {
        accel_range_enum = ICM20948_ACCEL_RANGE_8_G;
    } else { // default/fallback
        accel_range_enum = ICM20948_ACCEL_RANGE_16_G;
    }
    icm.setAccelRange(accel_range_enum);

    // ------------------------------------------------------------------------
    // Step 4: configure gyroscope full-scale range.
    // ------------------------------------------------------------------------
    icm20948_gyro_range_t gyro_range_enum = ICM20948_GYRO_RANGE_250_DPS;
    if (imu_gyro_fs_dps == 250) {
        gyro_range_enum = ICM20948_GYRO_RANGE_250_DPS;
    } else if (imu_gyro_fs_dps == 500) {
        gyro_range_enum = ICM20948_GYRO_RANGE_500_DPS;
    } else if (imu_gyro_fs_dps == 1000) {
        gyro_range_enum = ICM20948_GYRO_RANGE_1000_DPS;
    } else { // default/fallback
        gyro_range_enum = ICM20948_GYRO_RANGE_2000_DPS;
    }
    icm.setGyroRange(gyro_range_enum);

    // ------------------------------------------------------------------------
    // Step 5: configure magnetometer data rate.
    // ------------------------------------------------------------------------
    // We pick a rate at or above the main AHRS update rate.
    // Your AHRS loop is around 100 Hz, so 100 Hz mag is reasonable.
    icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
    mag_online_flag = true; // we assume mag is OK if config succeeds

    return true;
}


bool imu_read_sample(imu_sample_t &sample)
{
    if (!icm_online_flag) {
        return false;
    }

    // Adafruit API reads accel, gyro, mag, and temp in one call.
    sensors_event_t accel_event;
    sensors_event_t gyro_event;
    sensors_event_t mag_event;
    sensors_event_t temp_event;  // currently unused

    icm.getEvent(&accel_event, &gyro_event, &temp_event, &mag_event);

    // Convert from Adafruit sensor frame into BODY FRAME imu_sample_t.
    fill_body_frame_sample(accel_event, gyro_event, mag_event, sample);

    return true;
}


bool imu_is_online()
{
    return icm_online_flag;
}


bool imu_mag_is_online()
{
    return mag_online_flag;
}


// ///////temp using MPU6050

// #include "hal/sensors/imu.h"
// #include <Wire.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>


// Adafruit_MPU6050 mpu;
// unsigned long lastTime = 0;

// void IMU_Init() {
//     Wire.begin(); // Uses default SDA/SCL pins for ESP32 Feather V2
    
//     if (!mpu.begin()) {
//         Serial.println("Failed to find MPU6050 chip");
//         while (1) {
//             delay(10); // Halt execution if IMU isn't found
//         }
//     }
    
//     // Flight controller specific settings
//     mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // 8G is good to handle flight vibrations
//     mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//     mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // Low-pass filter to smooth motor noise
    
//     lastTime = micros();
//     Serial.println("IMU Initialized Successfully.");
// }

// void IMU_Read(IMUData_raw &data) {
//     sensors_event_t a, g, temp;
//     mpu.getEvent(&a, &g, &temp);

//     // 1. Read Raw Data
//     data.accel_x = a.acceleration.x;
//     data.accel_y = a.acceleration.y;
//     data.accel_z = a.acceleration.z;
//     data.gyro_x = g.gyro.x;
//     data.gyro_y = g.gyro.y;
//     data.gyro_z = g.gyro.z;

//     // 2. Calculate time elapsed since last read (dt)
//     float dt = (micros() - lastTime) / 1000000.0;
//     lastTime = micros();

//     // 3. Calculate absolute angles from Accelerometer (prone to vibration noise)
//     float accel_roll = atan2(data.accel_y, data.accel_z) * 180 / PI;
//     float accel_pitch = atan2(-data.accel_x, sqrt(data.accel_y * data.accel_y + data.accel_z * data.accel_z)) * 180 / PI;

//     // 4. Complementary Filter: Trust the Gyro for short-term movement, Trust Accel for long-term gravity reference
//     data.roll = 0.98 * (data.roll + data.gyro_x * dt) + 0.02 * accel_roll;
//     data.pitch = 0.98 * (data.pitch + data.gyro_y * dt) + 0.02 * accel_pitch;
    
//     data.healthy = true;
// }
