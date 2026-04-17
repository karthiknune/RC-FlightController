#pragma once

#include <freertos/FreeRTOS.h>

bool SensorBus_Init();
bool SensorBus_Lock(TickType_t timeout_ticks = portMAX_DELAY);
void SensorBus_Unlock();
