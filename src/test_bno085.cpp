#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#include "config.h"
#include "hal/sensors/sensor_bus.h"

namespace
{
    Adafruit_BNO08x bno08x;
    sh2_SensorValue_t sensorValue;

}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    Serial.println("\n============================================");
    Serial.println("         BNO085 IMU TEST SCRIPT             ");
    Serial.println("============================================");

    if (!SensorBus_Init())
    {
        Serial.println("Sensor I2C bus init failed.");
        while (1)
        {
            delay(10);
        }
    }

    // Try locking the bus to initialize the sensor safely
    if (SensorBus_Lock(pdMS_TO_TICKS(100)))
    {
        // Default address is 0x4A. Change to BNO08x_I2CADDR_ALT (0x4B) if needed.
        if (!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire))
        {
            Serial.println("Failed to find BNO085 chip. Check wiring or I2C address.");
            SensorBus_Unlock();
            while (1)
            {
                delay(10);
            }
        }

        Serial.println("BNO085 Found!");

        // Enable reports for the specific data we want at 100Hz
        bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
        bno08x.enableReport(SH2_ACCELEROMETER, 10000);
        bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
        bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 20000);

        SensorBus_Unlock();
    }
}

void loop()
{
    if (!SensorBus_Lock(pdMS_TO_TICKS(10)))
    {
        return;
    }

    if (bno08x.wasReset())
    {
        Serial.println("Sensor was reset, re-enabling reports...");
        bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
        bno08x.enableReport(SH2_ACCELEROMETER, 10000);
        bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
        bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 20000);
    }

    if (bno08x.getSensorEvent(&sensorValue))
    {
        switch (sensorValue.sensorId)
        {
        case SH2_ROTATION_VECTOR:
        {
            float qr = sensorValue.un.rotationVector.real;
            float qi = sensorValue.un.rotationVector.i;
            float qj = sensorValue.un.rotationVector.j;
            float qk = sensorValue.un.rotationVector.k;

            float ysqr = qj * qj;

            float t0 = +2.0f * (qr * qi + qj * qk);
            float t1 = +1.0f - 2.0f * (qi * qi + ysqr);
            float roll = atan2f(t0, t1) * 57.2957795f;

            float t2 = +2.0f * (qr * qj - qk * qi);
            t2 = t2 > 1.0f ? 1.0f : (t2 < -1.0f ? -1.0f : t2);
            float pitch = asinf(t2) * 57.2957795f;

            float t3 = +2.0f * (qr * qk + qi * qj);
            float t4 = +1.0f - 2.0f * (ysqr + qk * qk);
            float yaw = atan2f(t3, t4) * 57.2957795f;

            Serial.printf("RPY [deg]: Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f\n", roll, pitch, yaw);
            break;
        }
        case SH2_ACCELEROMETER:
            // Serial.printf("Accel [m/s2]: X: %6.2f, Y: %6.2f, Z: %6.2f\n", sensorValue.un.accelerometer.x, sensorValue.un.accelerometer.y, sensorValue.un.accelerometer.z);
            break;
        }
    }
    SensorBus_Unlock();

    delay(10);
}