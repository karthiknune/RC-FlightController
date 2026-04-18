#include <SPI.h>
#include <LoRa.h>

// ESP32-WROOM DevKit + SX127x wiring (ground station).
// Update all six pins if your wiring is different.
constexpr int SCK_PIN = 18;
constexpr int MISO_PIN = 19;
constexpr int MOSI_PIN = 23;
constexpr int CS_PIN = 5;
constexpr int RST_PIN = 4;
constexpr int IRQ_PIN = 2;

// Must match flight controller config (include/config.h).
constexpr long LORA_FREQ = 915000000L;
constexpr uint8_t SYNC_WORD = 0xF3;
constexpr int SPREADING_FACTOR = 7;
constexpr int LORA_TX_POWER_DBM = 10;
constexpr long LORA_SIGNAL_BANDWIDTH_HZ = 125000L;
constexpr int LORA_CODING_RATE_DENOM = 5;
constexpr long LORA_PREAMBLE_LENGTH = 8;
constexpr bool LORA_ENABLE_CRC = true;
constexpr long LORA_SPI_FREQUENCY = 2000000L;

// 50ms is very aggressive for half-duplex LoRa when flight side also transmits telemetry.
// Use 200ms during bring-up, then reduce if needed.
constexpr uint32_t TX_PERIOD_MS = 200UL;
constexpr uint32_t RX_WINDOW_MS = 120UL;

uint32_t g_cmd_id = 1;

bool packetLooksText(const uint8_t *data, size_t length) {
  if (data == nullptr || length == 0U) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    const uint8_t c = data[i];
    const bool printableAscii = (c >= 32U && c <= 126U);
    const bool whitespace = (c == '\n' || c == '\r' || c == '\t');
    if (!printableAscii && !whitespace) {
      return false;
    }
  }

  return true;
}

void pollIncomingPackets(uint32_t window_ms) {
  const uint32_t start_ms = millis();

  while ((millis() - start_ms) < window_ms) {
    const int packet_size = LoRa.parsePacket();
    if (packet_size <= 0) {
      delay(5);
      continue;
    }

    uint8_t payload[256] = {};
    const size_t bytes_to_read =
        (packet_size > 255) ? 255U : static_cast<size_t>(packet_size);
    const size_t bytes_read = LoRa.readBytes(payload, bytes_to_read);

    while (LoRa.available()) {
      (void)LoRa.read();
    }

    if (packetLooksText(payload, bytes_read)) {
      payload[bytes_read] = '\0';
      Serial.printf("RX text (%dB): %s\n", packet_size, payload);
    } else {
      Serial.printf("RX binary packet (%dB), RSSI=%d, SNR=%.1f\n",
                    packet_size,
                    LoRa.packetRssi(),
                    LoRa.packetSnr());
    }
  }
}

void setup() {
  Serial.begin(115200);
  const uint32_t serial_wait_start_ms = millis();
  while (!Serial && (millis() - serial_wait_start_ms) < 2000UL) {
    delay(10);
  }

  Serial.println("Initializing LoRa Sender...");
  Serial.printf("Pins SCK=%d MISO=%d MOSI=%d CS=%d RST=%d IRQ=%d\n",
                SCK_PIN,
                MISO_PIN,
                MOSI_PIN,
                CS_PIN,
                RST_PIN,
                IRQ_PIN);

  LoRa.end();
  SPI.end();
  delay(20);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  LoRa.setSPI(SPI);
  LoRa.setSPIFrequency(LORA_SPI_FREQUENCY);
  LoRa.setPins(CS_PIN, RST_PIN, IRQ_PIN);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("Starting LoRa failed!");
    while (true) {
      delay(100);
    }
  }

  LoRa.setTxPower(LORA_TX_POWER_DBM);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH_HZ);
  LoRa.setCodingRate4(LORA_CODING_RATE_DENOM);
  LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setSyncWord(SYNC_WORD);
  if (LORA_ENABLE_CRC) {
    LoRa.enableCrc();
  } else {
    LoRa.disableCrc();
  }
  LoRa.receive();

  Serial.println("LoRa Sender Ready. TX + short RX window enabled.");
}

void loop() {
  String tuning_cmd = "{\"axis\":\"roll\",\"kp\":22.5,\"cmd_id\":";
  tuning_cmd += String(static_cast<unsigned long>(g_cmd_id++));
  tuning_cmd += "}";

  Serial.print("Sending: ");
  Serial.println(tuning_cmd);

  const int begin_status = LoRa.beginPacket();
  const size_t bytes_written =
      (begin_status == 1) ? LoRa.print(tuning_cmd) : 0U;
  const int end_status = (begin_status == 1) ? LoRa.endPacket() : 0;

  if (begin_status != 1 || bytes_written != tuning_cmd.length() ||
      end_status != 1) {
    Serial.printf("TX failed (begin=%d, written=%u, end=%d)\n",
                  begin_status,
                  static_cast<unsigned>(bytes_written),
                  end_status);
  } else {
    Serial.println("TX ok.");
  }

  LoRa.receive();
  pollIncomingPackets(RX_WINDOW_MS);

  delay(TX_PERIOD_MS);
}
