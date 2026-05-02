#pragma once
#include <stdint.h>
#include <stddef.h>
#include "datatypes.h"

// Initializes PID values from config defaults, then attempts to load overrides from the SD card.
void GCS_LoadPIDTuningsFromConfig();
// Processes raw incoming LoRa packets, validates them, and queues commands.
void GCS_ProcessIncomingPacket(const uint8_t* buffer, size_t length);
// Applies queued GCS commands safely via FreeRTOS locks.
void GCS_ApplyPendingCommands();
// Safely reads the active PID gains via spinlocks to populate the telemetry snapshot.
void GCS_PopulateTelemetryTuning(telemetrydata& snapshot);
// Returns the active AltHold target altitude, initialized from target_alt_agl in config.h.
float GCS_GetTargetAltitudeAGL();
