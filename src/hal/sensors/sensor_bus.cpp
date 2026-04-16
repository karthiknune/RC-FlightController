#include "hal/sensors/sensor_bus.h"

#include <Wire.h>
#include <freertos/semphr.h>

#include "config.h"

namespace {

SemaphoreHandle_t g_sensor_bus_mutex = nullptr;
bool g_sensor_bus_initialized = false;

} // namespace

bool SensorBus_Init() {
    if (g_sensor_bus_mutex == nullptr) {
        g_sensor_bus_mutex = xSemaphoreCreateMutex();
        if (g_sensor_bus_mutex == nullptr) {
            return false;
        }
    }

    if (!g_sensor_bus_initialized) {
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
        Wire.setClock(I2C_BUS_FREQUENCY_HZ);
        Wire.setTimeOut(SENSOR_I2C_TIMEOUT_MS);
        g_sensor_bus_initialized = true;
    }

    return true;
}

bool SensorBus_Lock(TickType_t timeout_ticks) {
    return (g_sensor_bus_mutex != nullptr) &&
           (xSemaphoreTake(g_sensor_bus_mutex, timeout_ticks) == pdTRUE);
}

void SensorBus_Unlock() {
    if (g_sensor_bus_mutex != nullptr) {
        xSemaphoreGive(g_sensor_bus_mutex);
    }
}
