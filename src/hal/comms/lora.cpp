#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <string.h>
#include "hal/comms/lora.h"

namespace {

constexpr int SCK_PIN = 5;
constexpr int MOSI_PIN = 19;
constexpr int MISO_PIN = 21;

constexpr int CS_PIN = 27;
constexpr int RST_PIN = 32;
constexpr int IRQ_PIN = 14;

constexpr long LORA_FREQ = 915000000L;
constexpr uint8_t SYNC_WORD = 0xF3;

SemaphoreHandle_t g_lora_mutex = nullptr;
bool g_lora_ready = false;

bool take_lora_lock() {
  return (g_lora_mutex != nullptr) &&
         (xSemaphoreTake(g_lora_mutex, portMAX_DELAY) == pdTRUE);
}

void give_lora_lock() {
  if (g_lora_mutex != nullptr) {
    xSemaphoreGive(g_lora_mutex);
  }
}

}  // namespace

bool lora_init() {
  if (g_lora_ready) {
    return true;
  }

  if (g_lora_mutex == nullptr) {
    g_lora_mutex = xSemaphoreCreateMutex();
    if (g_lora_mutex == nullptr) {
      return false;
    }
  }

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, -1);
  LoRa.setSPI(SPI);
  LoRa.setPins(CS_PIN, RST_PIN, IRQ_PIN);

  if (!LoRa.begin(LORA_FREQ)) {
    return false;
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSyncWord(SYNC_WORD);
  g_lora_ready = true;
  return true;
}

bool lora_send(const uint8_t *data, size_t length) {
  if (!g_lora_ready || data == nullptr || length == 0 || !take_lora_lock()) {
    return false;
  }

  const int begin_result = LoRa.beginPacket();
  const size_t bytes_written =
      (begin_result == 1) ? LoRa.write(data, length) : 0;
  const int end_result = (begin_result == 1) ? LoRa.endPacket() : 0;

  give_lora_lock();
  return begin_result == 1 && bytes_written == length && end_result == 1;
}

bool lora_send(const char *message) {
  if (message == nullptr) {
    return false;
  }

  return lora_send(reinterpret_cast<const uint8_t *>(message), strlen(message));
}

size_t lora_receive(uint8_t *buffer, size_t max_length) {
  if (!g_lora_ready || buffer == nullptr || max_length == 0 || !take_lora_lock()) {
    return 0;
  }

  const int packet_size = LoRa.parsePacket();
  if (packet_size <= 0) {
    give_lora_lock();
    return 0;
  }

  size_t bytes_read = 0;
  while (LoRa.available() && bytes_read < max_length) {
    buffer[bytes_read++] = static_cast<uint8_t>(LoRa.read());
  }

  while (LoRa.available()) {
    (void)LoRa.read();
  }

  give_lora_lock();
  return bytes_read;
}
