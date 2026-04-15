#include "hal/sensors/imu.h"
#include "config.h"

#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

// ============================================================================
// IMU HARDWARE LAYER USING ADAFRUIT_ICM20948
// ============================================================================
// Responsibilities:
//
//   - start ESP32 I2C on pins from config.h
//   - initialize Adafruit ICM20948 driver
//   - configure accel/gyro/mag ranges & rates
//   - read accel, gyro, mag from the chip
//   - apply orientation transforms:
//
//       AK (mag sensor frame)  -> ICM frame
//       ICM (chip frame)       -> AIRCRAFT BODY FRAME
//
//     using the same mapping you previously derived:
//
//       AK -> ICM (mag only):
//         x_icm =  x_ak
//         y_icm = -y_ak
//         z_icm = -z_ak
//
//       ICM -> BODY (all sensors):
//         ICM +X points opposite aircraft forward
//         ICM +Y matches aircraft +Y
//         => x_body = -x_icm
//            y_body =  y_icm
//            z_body =  z_icm
//
//   - apply calibration constants from config.h in BODY FRAME
//
// It does NOT compute roll/pitch/yaw; that happens in src/math/ahrs.cpp.
// ============================================================================


// ---------------------------------------------------------------------------
// GLOBAL ADAFRUIT IMU OBJECT + STATUS FLAGS
// ---------------------------------------------------------------------------

static Adafruit_ICM20948 icm;

static bool icm_online_flag = false;
static bool mag_online_flag = false;


// ============================================================================
// INTERNAL HELPER: SENSOR FRAME -> BODY FRAME + CALIBRATION
// ============================================================================
//
// Take Adafruit sensor events and fill one body-frame, calibrated imu_sample_t.
// ============================================================================
static void fill_body_frame_sample(const sensors_event_t &accel_event,
                                   const sensors_event_t &gyro_event,
                                   const sensors_event_t &mag_event,
                                   imu_sample_t &out_sample)
{
    // 1) Convert Adafruit units into ICM frame in convenient units.
    //
    //   accel: m/s^2 -> g
    //   gyro : rad/s -> deg/s
    //   mag  : already in microtesla
    const float g_per_ms2   = 1.0f / 9.80665f;
    const float deg_per_rad = 180.0f / PI;

    float ax_icm_g = accel_event.acceleration.x * g_per_ms2;
    float ay_icm_g = accel_event.acceleration.y * g_per_ms2;
    float az_icm_g = accel_event.acceleration.z * g_per_ms2;

    float gx_icm_dps = gyro_event.gyro.x * deg_per_rad;
    float gy_icm_dps = gyro_event.gyro.y * deg_per_rad;
    float gz_icm_dps = gyro_event.gyro.z * deg_per_rad;

    // AK (mag) -> ICM mapping from your original driver:
    //   x_icm =  x_ak
    //   y_icm = -y_ak
    //   z_icm = -z_ak
    float mx_icm_uT = mag_event.magnetic.x;
    float my_icm_uT = -mag_event.magnetic.y;
    float mz_icm_uT = -mag_event.magnetic.z;

    // 2) ICM -> BODY frame mapping from your mounting note:
    //
    //   ICM +X points opposite aircraft forward.
    //   ICM +Y matches aircraft +Y.
    //
    // => x_body = -x_icm
    //    y_body =  y_icm
    //    z_body =  z_icm
    float ax_body = -ax_icm_g;
    float ay_body =  ay_icm_g;
    float az_body =  az_icm_g;

    float gx_body = -gx_icm_dps;
    float gy_body =  gy_icm_dps;
    float gz_body =  gz_icm_dps;

    float mx_body = -mx_icm_uT;
    float my_body =  my_icm_uT;
    float mz_body =  mz_icm_uT;

    // 3) Apply BODY-FRAME calibration constants from config.h.
    out_sample.ax_g = (ax_body - ax_bias_g) * ax_scale;
    out_sample.ay_g = (ay_body - ay_bias_g) * ay_scale;
    out_sample.az_g = (az_body - az_bias_g) * az_scale;

    out_sample.gx_dps = (gx_body - gx_bias_dps) * gx_scale;
    out_sample.gy_dps = (gy_body - gy_bias_dps) * gy_scale;
    out_sample.gz_dps = (gz_body - gz_bias_dps) * gz_scale;

    out_sample.mx_uT = (mx_body - mx_bias_uT) * mx_scale;
    out_sample.my_uT = (my_body - my_bias_uT) * my_scale;
    out_sample.mz_uT = (mz_body - mz_bias_uT) * mz_scale;

    // Timestamp in milliseconds.
    out_sample.timestamp_ms = millis();
}


// ============================================================================
// PUBLIC API IMPLEMENTATION
// ============================================================================

bool imu_init()
{
    // Start I2C on pins/frequency from config.h.
    Wire.begin(imu_sda_pin, imu_scl_pin, imu_i2c_freq_hz);

    // Initialize the Adafruit ICM-20948 driver.
    if (!icm.begin_I2C(ICM20948_I2CADDR_DEFAULT, &Wire)) {
        icm_online_flag = false;
        mag_online_flag = false;
        return false;
    }

    icm_online_flag = true;

    // Configure accelerometer range from config.h.
    icm20948_accel_range_t accel_range_enum = ICM20948_ACCEL_RANGE_4_G;
    if (imu_accel_fs_g == 2) {
        accel_range_enum = ICM20948_ACCEL_RANGE_2_G;
    } else if (imu_accel_fs_g == 4) {
        accel_range_enum = ICM20948_ACCEL_RANGE_4_G;
    } else if (imu_accel_fs_g == 8) {
        accel_range_enum = ICM20948_ACCEL_RANGE_8_G;
    } else {
        accel_range_enum = ICM20948_ACCEL_RANGE_16_G;
    }
    icm.setAccelRange(accel_range_enum);

    // Configure gyroscope range from config.h.
    icm20948_gyro_range_t gyro_range_enum = ICM20948_GYRO_RANGE_250_DPS;
    if (imu_gyro_fs_dps == 250) {
        gyro_range_enum = ICM20948_GYRO_RANGE_250_DPS;
    } else if (imu_gyro_fs_dps == 500) {
        gyro_range_enum = ICM20948_GYRO_RANGE_500_DPS;
    } else if (imu_gyro_fs_dps == 1000) {
        gyro_range_enum = ICM20948_GYRO_RANGE_1000_DPS;
    } else {
        gyro_range_enum = ICM20948_GYRO_RANGE_2000_DPS;
    }
    icm.setGyroRange(gyro_range_enum);

    // Configure magnetometer data rate (near AHRS loop rate).
    icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
    mag_online_flag = true;

    return true;
}


bool imu_read_sample(imu_sample_t &sample)
{
    if (!icm_online_flag) {
        return false;
    }

    // Read accel, gyro, mag, temp from Adafruit driver in one call.
    sensors_event_t accel_event;
    sensors_event_t gyro_event;
    sensors_event_t mag_event;
    sensors_event_t temp_event; // unused for now

    icm.getEvent(&accel_event, &gyro_event, &temp_event, &mag_event);

    // Convert to body-frame calibrated sample.
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
