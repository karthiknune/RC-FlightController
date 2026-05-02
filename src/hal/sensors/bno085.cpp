#include "hal/sensors/bno085.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <sh2.h>
#include <cmath>
#include "config.h"
#include "hal/sensors/sensor_bus.h"

namespace
{
    Adafruit_BNO08x bno08x;
    bool g_bno_ready = false;
    uint32_t g_last_init_attempt_ms = 0;
    uint8_t g_last_calibration_status = 0;
    float g_yaw_tare_offset = 0.0f;

    bool g_background_calibration_enabled = false;
    float g_mag_bias_x = 0.0f;
    float g_mag_bias_y = 0.0f;
    float g_mag_bias_z = 0.0f;
    float g_gyro_bias_x = 0.0f;
    float g_gyro_bias_y = 0.0f;
    float g_gyro_bias_z = 0.0f;
    bool g_initial_yaw_tared = false;

    // Cache for asynchronous reports
    IMUData_raw g_cached_data = {};

    bool TryInitializeBNO()
    {
        const uint32_t now_ms = millis();
        if (g_last_init_attempt_ms != 0 && (now_ms - g_last_init_attempt_ms) < SENSOR_RECONNECT_INTERVAL_MS)
        {
            return g_bno_ready;
        }
        g_last_init_attempt_ms = now_ms;

        if (!SensorBus_Init() || !SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS)))
        {
            if (g_bno_ready && SENSOR_STATUS_LOGGING_ENABLED)
                Serial.println("BNO085 unavailable. Continuing without BNO085 data.");
            g_bno_ready = false;
            return false;
        }

        // Default address is 0x4A. Use BNO08x_I2CADDR_ALT for 0x4B
        bool begin_ok = bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire);

        if (begin_ok)
        {
            // Enable desired reports at 100Hz (10000us)
            bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
            bno08x.enableReport(SH2_ACCELEROMETER, 10000);
            bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
            bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 20000);
        }

        SensorBus_Unlock();

        if (begin_ok && !g_bno_ready && SENSOR_STATUS_LOGGING_ENABLED)
        {
            Serial.println("BNO085 available.");
        }
        else if (!begin_ok && g_bno_ready && SENSOR_STATUS_LOGGING_ENABLED)
        {
            Serial.println("BNO085 unavailable. Continuing without BNO085 data.");
        }

        g_bno_ready = begin_ok;
        return begin_ok;
    }
} // namespace

void BNO085_Init()
{
    (void)TryInitializeBNO();
}

void BNO085_Read(IMUData_raw &data)
{
    if (!g_bno_ready && !TryInitializeBNO())
    {
        data = g_cached_data;
        data.healthy = false;
        return;
    }

    if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS)))
    {
        data = g_cached_data;
        data.healthy = false;
        return;
    }

    if (bno08x.wasReset())
    {
        bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
        bno08x.enableReport(SH2_ACCELEROMETER, 10000);
        bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
        bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 20000);
        if (g_background_calibration_enabled)
        {
            bno08x.enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, 20000);
            bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 20000);
        }
    }

    sh2_SensorValue_t sensorValue;
    // Drain the event queue for fresh data
    while (bno08x.getSensorEvent(&sensorValue))
    {
        switch (sensorValue.sensorId)
        {
        case SH2_ACCELEROMETER:
            g_cached_data.accel_x = sensorValue.un.accelerometer.x * IMU_BODY_FRAME_X_SIGN;
            g_cached_data.accel_y = sensorValue.un.accelerometer.y * IMU_BODY_FRAME_Y_SIGN;
            g_cached_data.accel_z = sensorValue.un.accelerometer.z * IMU_BODY_FRAME_Z_SIGN;
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            // BNO outputs rad/s, datatypes struct standardizes to deg/s
            g_cached_data.gyro_x = (sensorValue.un.gyroscope.x * 57.2957795f) * IMU_BODY_FRAME_X_SIGN;
            g_cached_data.gyro_y = (sensorValue.un.gyroscope.y * 57.2957795f) * IMU_BODY_FRAME_Y_SIGN;
            g_cached_data.gyro_z = (sensorValue.un.gyroscope.z * 57.2957795f) * IMU_BODY_FRAME_Z_SIGN;
            break;
        case SH2_MAGNETIC_FIELD_CALIBRATED:
            g_cached_data.mag_x = sensorValue.un.magneticField.x * IMU_MAG_SENSOR_ALIGN_X_SIGN;
            g_cached_data.mag_y = sensorValue.un.magneticField.y * IMU_MAG_SENSOR_ALIGN_Y_SIGN;
            g_cached_data.mag_z = sensorValue.un.magneticField.z * IMU_MAG_SENSOR_ALIGN_Z_SIGN;
            break;
        case SH2_MAGNETIC_FIELD_UNCALIBRATED:
            g_mag_bias_x = sensorValue.un.magneticFieldUncal.biasX * IMU_MAG_SENSOR_ALIGN_X_SIGN;
            g_mag_bias_y = sensorValue.un.magneticFieldUncal.biasY * IMU_MAG_SENSOR_ALIGN_Y_SIGN;
            g_mag_bias_z = sensorValue.un.magneticFieldUncal.biasZ * IMU_MAG_SENSOR_ALIGN_Z_SIGN;
            break;
        case SH2_GYROSCOPE_UNCALIBRATED:
            g_gyro_bias_x = sensorValue.un.gyroscopeUncal.biasX * IMU_BODY_FRAME_X_SIGN;
            g_gyro_bias_y = sensorValue.un.gyroscopeUncal.biasY * IMU_BODY_FRAME_Y_SIGN;
            g_gyro_bias_z = sensorValue.un.gyroscopeUncal.biasZ * IMU_BODY_FRAME_Z_SIGN;
            break;
        case SH2_ROTATION_VECTOR:
        {
            g_last_calibration_status = sensorValue.status;

            float qr = sensorValue.un.rotationVector.real;
            float qi = sensorValue.un.rotationVector.i;
            float qj = sensorValue.un.rotationVector.j;
            float qk = sensorValue.un.rotationVector.k;
            float ysqr = qj * qj;

            float t0 = +2.0f * (qr * qi + qj * qk);
            float t1 = +1.0f - 2.0f * (qi * qi + ysqr);
            float raw_roll = atan2f(t0, t1) * 57.2957795f;

            float t2 = +2.0f * (qr * qj - qk * qi);
            t2 = t2 > 1.0f ? 1.0f : (t2 < -1.0f ? -1.0f : t2);
            float raw_pitch = asinf(t2) * 57.2957795f;

            float t3 = +2.0f * (qr * qk + qi * qj);
            float t4 = +1.0f - 2.0f * (ysqr + qk * qk);
            float raw_yaw = atan2f(t3, t4) * 57.2957795f;

            g_cached_data.roll = (raw_roll * IMU_BODY_FRAME_X_SIGN) - IMU_LEVEL_ROLL_OFFSET_DEG;
            g_cached_data.pitch = (raw_pitch * IMU_BODY_FRAME_Y_SIGN) - IMU_LEVEL_PITCH_OFFSET_DEG;

            // Automatically tare the yaw to 0.0 on the very first valid reading after boot
            if (!g_initial_yaw_tared)
            {
                g_yaw_tare_offset = raw_yaw * IMU_BODY_FRAME_Z_SIGN;
                g_initial_yaw_tared = true;
            }

            float yaw_val = (raw_yaw * IMU_BODY_FRAME_Z_SIGN) - g_yaw_tare_offset;
            // Keep yaw wrapped between -180 and +180
            while (yaw_val > 180.0f)
                yaw_val -= 360.0f;
            while (yaw_val < -180.0f)
                yaw_val += 360.0f;

            g_cached_data.yaw = yaw_val;
            break;
        }
        }
    }

    SensorBus_Unlock();

    g_cached_data.healthy = true;
    data = g_cached_data;
}

void BNO085_EnableBackgroundCalibration()
{
    if (!g_bno_ready)
        return;
    if (SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS)))
    {
        // The Adafruit library exposes the native Hillcrest SH-2 HAL.
        // We call the SH-2 API directly to enable Accel, Gyro, and Mag background calibration.
        sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
        g_background_calibration_enabled = true;
        bno08x.enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, 20000);
        bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 20000);
        SensorBus_Unlock();
    }
}

void BNO085_SaveCalibrationToFlash()
{
    if (!g_bno_ready)
        return;
    if (SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS)))
    {
        // Issue the Dynamic Calibration Data (DCD) save command directly to the SH-2 HAL
        sh2_saveDcdNow();

        // We MUST pump the event loop immediately to flush the SH-2 TX queue.
        // Otherwise, the command is never actually sent to the sensor until the
        // next Read(), which causes a fatal crash when the sensor goes deaf.
        sh2_SensorValue_t dummy;
        for (int i = 0; i < 3; i++)
        {
            bno08x.getSensorEvent(&dummy);
            delay(5);
        }

        SensorBus_Unlock();
    }

    // The BNO085 halts its I2C engine and goes completely offline to write to its
    // internal EEPROM flash memory. Delaying gives it plenty of time.
    delay(2000);
}

uint8_t BNO085_GetCalibrationStatus()
{
    // Status scale: 0 = Unreliable, 1 = Low, 2 = Medium, 3 = High
    return g_last_calibration_status;
}

void BNO085_TareYaw()
{
    // Add the current measured yaw to the offset so it instantly becomes 0
    g_yaw_tare_offset += g_cached_data.yaw;
}

void BNO085_GetCalibrationBiases(float &mag_x, float &mag_y, float &mag_z, float &gyro_x, float &gyro_y, float &gyro_z)
{
    mag_x = g_mag_bias_x;
    mag_y = g_mag_bias_y;
    mag_z = g_mag_bias_z;
    gyro_x = g_gyro_bias_x;
    gyro_y = g_gyro_bias_y;
    gyro_z = g_gyro_bias_z;
}
