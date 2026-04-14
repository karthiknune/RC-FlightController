#include "flight/telemetry.h"
#include "hal/comms/lora.h"

static_assert(sizeof(telemetrydata) <= 255,
              "telemetrydata exceeds LoRa packet size");

bool telemetry_send(const telemetrydata &data) {
    // Current transport sends a raw telemetry snapshot in a single LoRa packet.
    return lora_send(reinterpret_cast<const uint8_t *>(&data), sizeof(data));
}
