#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "hal/sensors/baro.h"
#include "hal/sensors/i2c_bus.h"

#include <cmath>

#include "config.h"

#define SEALEVELPRESSURE_HPA (1031.2) /// adjust based on local sea level press

Adafruit_BMP3XX bmp;

namespace
{

    float pressure_to_altitude_meters(float pressure_hpa, float sea_level_hpa)
    {
        return 44330.0f * (1.0f - std::pow(pressure_hpa / sea_level_hpa, 0.1903f));
    }

} // namespace

void Barometer_Init()
{
    i2c_bus_init();

    if (!i2c_bus_lock())
    {
        Serial.println("Failed to lock I2C bus for BMP3 init");
        while (1)
        {
            delay(10);
        }
    }

    const bool barometer_ready = bmp.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire);
    i2c_bus_unlock();

    if (!barometer_ready)
    {
        Serial.println("Could not find a valid BMP3 sensor, check wiring!");
        // while (1) {
        //     delay(10);
        // }
    }

    // Set up oversampling and filter configurations
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("Barometer initialized successfully.");
}

void Barometer_Read(BarometerData &data)
{
    if (!i2c_bus_lock())
    {
        data.healthy = false;
        return;
    }

    const bool read_ok = bmp.performReading();
    i2c_bus_unlock();

    if (!read_ok)
    {
        // COMMENT THIS OUT to stop the serial spam!
        // Serial.println("Failed to perform reading :(");
        data.healthy = false;
        return;
    }

    data.pressure = bmp.pressure / 100.0;                                             //  hPa
    data.altitude = pressure_to_altitude_meters(data.pressure, SEALEVELPRESSURE_HPA); // m
    data.healthy = true;
}
