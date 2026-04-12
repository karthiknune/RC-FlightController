#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "hal/sensors/baro.h"

#define SEALEVELPRESSURE_HPA (1031.2) /// adjust based on local sea level press

Adafruit_BMP3XX bmp;

void Barometer_Init() {
    if (!bmp.begin_I2C()) {   
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter configurations
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void Barometer_Read(BarometerData &data) {
    if (!bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        data.healthy = false;
        return;
    }
    
    data.pressure = bmp.pressure / 100.0; //  hPa
    data.altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA); //m
    data.healthy = true;
}
