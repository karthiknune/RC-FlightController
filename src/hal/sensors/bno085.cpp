#include "hal/sensors/bno085.h"

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <cmath>

#include "config.h"
#include "hal/sensors/sensor_bus.h"

namespace
{
    Adafruit_BNO08x bno08x;
    bool g_bno_ready = false;
    uint32_t g_last_init_attempt_ms = 0;
    sh2_SensorValue_t sensorValue;

    // Retain last known values since reports arrive asynchronously
    float g_accel_x = 0.0f, g_accel_y = 0.0f, g_accel_z = 0.0f;
    float g_gyro_x = 0.0f, g_gyro_y = 0.0f, g_gyro_z = 0.0f;
    float g_mag_x = 0.0f, g_mag_y = 0.0f, g_mag_z = 0.0f;
    float g_roll = 0.0f, g_pitch = 0.0f, g_yaw = 0.0f;

    void UpdateBNOAvailability(bool available)
    {
        if (available && !g_bno_ready && SENSOR_STATUS_LOGGING_ENABLED)
        {
            Serial.println("BNO085 available.");
        }
        else if (!available && g_bno_ready && SENSOR_STATUS_LOGGING_ENABLED)
        {
            Serial.println("BNO085 unavailable. Continuing without BNO085 data.");
        }
        g_bno_ready = available;
    }

    bool TryInitializeBNO()
    {
        const uint32_t now_ms = millis();
        if (g_last_init_attempt_ms != 0 && (now_ms - g_last_init_attempt_ms) < SENSOR_RECONNECT_INTERVAL_MS)
        {
            return g_bno_ready;
        }
        g_last_init_attempt_ms = now_ms;

        if (!SensorBus_Init())
        {
            UpdateBNOAvailability(false);
            return false;
        }

        if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS)))
        {
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
        UpdateBNOAvailability(begin_ok);
        return begin_ok;
    }

    float ApplyBodyFrameAxis(float value, float axis_sign)
    {
        return value * axis_sign;
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
        data.healthy = false;
        return;
    }

    if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS)))
    {
        data.healthy = false;
        return;
    }

    if (bno08x.wasReset())
    {
        bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
        bno08x.enableReport(SH2_ACCELEROMETER, 10000);
        bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
        bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 20000);
    }

    // Drain the event queue for fresh data
    while (bno08x.getSensorEvent(&sensorValue))
    {
        switch (sensorValue.sensorId)
        {
        case SH2_ACCELEROMETER:
            g_accel_x = sensorValue.un.accelerometer.x;
            g_accel_y = sensorValue.un.accelerometer.y;
            g_accel_z = sensorValue.un.accelerometer.z;
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            // BNO outputs rad/s, datatypes struct standardizes to deg/s
            g_gyro_x = sensorValue.un.gyroscope.x * 57.2957795f;
            g_gyro_y = sensorValue.un.gyroscope.y * 57.2957795f;
            g_gyro_z = sensorValue.un.gyroscope.z * 57.2957795f;
            break;
        case SH2_MAGNETIC_FIELD_CALIBRATED:
            g_mag_x = sensorValue.un.magneticField.x;
            g_mag_y = sensorValue.un.magneticField.y;
            g_mag_z = sensorValue.un.magneticField.z;
            break;
        case SH2_ROTATION_VECTOR:
            float qr = sensorValue.un.rotationVector.real;
            float qi = sensorValue.un.rotationVector.i;
            float qj = sensorValue.un.rotationVector.j;
            float qk = sensorValue.un.rotationVector.k;
            float ysqr = qj * qj;

            float t0 = +2.0f * (qr * qi + qj * qk);
            float t1 = +1.0f - 2.0f * (qi * qi + ysqr);
            g_roll = atan2f(t0, t1) * 57.2957795f;

            float t2 = +2.0f * (qr * qj - qk * qi);
            t2 = t2 > 1.0f ? 1.0f : (t2 < -1.0f ? -1.0f : t2);
            g_pitch = asinf(t2) * 57.2957795f;

            float t3 = +2.0f * (qr * qk + qi * qj);
            float t4 = +1.0f - 2.0f * (ysqr + qk * qk);
            g_yaw = atan2f(t3, t4) * 57.2957795f;
            break;
        }
    }

    SensorBus_Unlock();

    // Apply body frame orientations from config.h to align with the aircraft FRD frame
    data.accel_x = ApplyBodyFrameAxis(g_accel_x, IMU_BODY_FRAME_X_SIGN);
    data.accel_y = ApplyBodyFrameAxis(g_accel_y, IMU_BODY_FRAME_Y_SIGN);
    data.accel_z = ApplyBodyFrameAxis(g_accel_z, IMU_BODY_FRAME_Z_SIGN);

    data.gyro_x = ApplyBodyFrameAxis(g_gyro_x, IMU_BODY_FRAME_X_SIGN);
    data.gyro_y = ApplyBodyFrameAxis(g_gyro_y, IMU_BODY_FRAME_Y_SIGN);
    data.gyro_z = ApplyBodyFrameAxis(g_gyro_z, IMU_BODY_FRAME_Z_SIGN);

    data.mag_x = ApplyBodyFrameAxis(g_mag_x, IMU_MAG_SENSOR_ALIGN_X_SIGN);
    data.mag_y = ApplyBodyFrameAxis(g_mag_y, IMU_MAG_SENSOR_ALIGN_Y_SIGN);
    data.mag_z = ApplyBodyFrameAxis(g_mag_z, IMU_MAG_SENSOR_ALIGN_Z_SIGN);

    // Apply Euler alignments & startup offsets from config.h
    data.roll = ApplyBodyFrameAxis(g_roll, IMU_BODY_FRAME_X_SIGN) - IMU_LEVEL_ROLL_OFFSET_DEG;
    data.pitch = ApplyBodyFrameAxis(g_pitch, IMU_BODY_FRAME_Y_SIGN) - IMU_LEVEL_PITCH_OFFSET_DEG;
    data.yaw = ApplyBodyFrameAxis(g_yaw, IMU_BODY_FRAME_Z_SIGN);

    data.healthy = true;
}
