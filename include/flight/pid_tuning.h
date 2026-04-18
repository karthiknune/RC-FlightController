#pragma once

#include <stddef.h>
#include <stdint.h>

void PIDTuning_Init();
bool PIDTuning_ApplyLoRaUpdate(const uint8_t *data, size_t length);
bool PIDTuning_ShouldHoldTelemetryTx();
