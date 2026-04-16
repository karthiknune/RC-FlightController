#include "hal/sensors/i2c_bus.h"

#include "freertos/semphr.h"

namespace {

SemaphoreHandle_t g_i2c_mutex = nullptr;

} // namespace

void i2c_bus_init() {
    if (g_i2c_mutex == nullptr) {
        g_i2c_mutex = xSemaphoreCreateMutex();
    }
}

bool i2c_bus_lock(TickType_t timeout_ticks) {
    i2c_bus_init();
    return (g_i2c_mutex != nullptr) &&
           (xSemaphoreTake(g_i2c_mutex, timeout_ticks) == pdTRUE);
}

void i2c_bus_unlock() {
    if (g_i2c_mutex != nullptr) {
        xSemaphoreGive(g_i2c_mutex);
    }
}
