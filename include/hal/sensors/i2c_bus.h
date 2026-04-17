#pragma once

#include "freertos/FreeRTOS.h"

void i2c_bus_init();
bool i2c_bus_lock(TickType_t timeout_ticks = portMAX_DELAY);
void i2c_bus_unlock();
