#include "hal/sensors/baro.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#include "config.h"
#include "hal/sensors/sensor_bus.h"

Adafruit_BMP3XX bmp;

namespace {

uint32_t g_last_init_attempt_ms = 0;
bool g_baro_ready = false;
bool g_baro_missing_logged = false;

void UpdateBarometerAvailability(bool available) {
  if (available) {
    if (!g_baro_ready && SENSOR_STATUS_LOGGING_ENABLED) {
      Serial.println("BMP3XX available.");
    }

    g_baro_ready = true;
    g_baro_missing_logged = false;
    return;
  }

  if ((g_baro_ready || !g_baro_missing_logged) && SENSOR_STATUS_LOGGING_ENABLED) {
    Serial.println("BMP3XX unavailable. Continuing without barometer data.");
  }

  g_baro_ready = false;
  g_baro_missing_logged = true;
}

float PressureToAltitudeMeters(float pressure_hpa) {
  return 44330.0f * (1.0f - powf(pressure_hpa / SEALEVELPRESSURE_HPA, 0.1903f));
}

bool TryInitializeBarometer() {
  const uint32_t now_ms = millis();
  if (g_last_init_attempt_ms != 0 &&
      (now_ms - g_last_init_attempt_ms) < SENSOR_RECONNECT_INTERVAL_MS) {
    return g_baro_ready;
  }

  g_last_init_attempt_ms = now_ms;

  if (!SensorBus_Init()) {
    UpdateBarometerAvailability(false);
    return false;
  }

  if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS))) {
    return false;
  }

  const bool begin_ok = bmp.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire);
  if (begin_ok) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  SensorBus_Unlock();

  if (!begin_ok) {
    UpdateBarometerAvailability(false);
    return false;
  }

  UpdateBarometerAvailability(true);
  return true;
}

} // namespace

void Barometer_Init() {
  (void)TryInitializeBarometer();
}

void Barometer_Read(BarometerData &data) {
  if (!g_baro_ready && !TryInitializeBarometer()) {
    data.healthy = false;
    return;
  }

  if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS))) {
    data.healthy = false;
    return;
  }

  const bool reading_ok = bmp.performReading();
  const float pressure_hpa = bmp.pressure / 100.0f;
  SensorBus_Unlock();

  if (!reading_ok) {
    UpdateBarometerAvailability(false);
    data.healthy = false;
    return;
  }

  data.pressure = pressure_hpa;
  data.altitude = PressureToAltitudeMeters(pressure_hpa);
  data.healthy = true;
  UpdateBarometerAvailability(true);
}
