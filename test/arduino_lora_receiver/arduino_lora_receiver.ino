#include <SPI.h>
#include <LoRa.h>
#include <stdint.h>
#include <string.h>

// Paste one JSON command per line in Arduino IDE Serial Monitor, then press Enter.
// Ground station auto-adds cmd_id and waits for flight ACK/NACK.
// Examples:
// {"axis":"roll","kp":18.5}
// {"axis":"roll","kp":18.5,"ki":0.02,"kd":0.12}
// {"pid":{"roll":{"kp":19.0,"ki":0.01,"kd":0.10},"pitch":{"kp":17.0,"ki":0.00,"kd":0.08}}}
// {"axis":"headingerror","kp":0.8,"ki":0.0,"kd":0.04}

// ESP32-WROOM DevKit + SX1276/SX1278 wiring.
// Change these only if your receiver module is wired differently.
constexpr int SCK_PIN = 18;
constexpr int MISO_PIN = 19;
constexpr int MOSI_PIN = 23;
constexpr int CS_PIN = 5;

// Optional for bring-up; set to actual pins if wired.
constexpr int RST_PIN = 4;
constexpr int IRQ_PIN = 2;

constexpr long LORA_FREQ = 915E6;
constexpr uint8_t SYNC_WORD = 0xF3;
constexpr int SPREADING_FACTOR = 7;
constexpr int LORA_TX_POWER_DBM = 10;
constexpr long LORA_SIGNAL_BANDWIDTH_HZ = 125000L;
constexpr int LORA_CODING_RATE_DENOM = 5;
constexpr long LORA_PREAMBLE_LENGTH = 8;
constexpr bool LORA_ENABLE_CRC = true;
constexpr long LORA_SPI_FREQUENCY = 2000000L;

constexpr unsigned long SERIAL_BAUD = 115200UL;
constexpr size_t MAX_COMMAND_BYTES = 240U;
constexpr size_t MAX_RX_PACKET_BYTES = 255U;
constexpr uint8_t COMMAND_MAX_RETRIES = 8U;
constexpr uint32_t COMMAND_ACK_TIMEOUT_MS = 1600UL;
constexpr uint32_t COMMAND_RETRY_GAP_MIN_MS = 90UL;
constexpr uint32_t COMMAND_RETRY_GAP_MAX_MS = 260UL;
constexpr uint32_t COMMAND_PRE_TX_IDLE_MS = 120UL;

struct TelemetryPacket {
  float roll;
  float pitch;
  float throttle;
  float yaw;
  float des_roll;
  float des_pitch;
  float des_throttle;
  float des_yaw;
  float altitude;
  float des_altitude;
  float airspeed;
  float gps_lat;
  float gps_long;
  float gps_alt;
  float gps_speed;
  float gps_heading;
  int32_t gps_sats;
  int32_t gps_fix_quality;
  int32_t gps_lock_acquired;
  float baro_altitude;
  float flightmode;
  float failsafe_status;
  float waypoint_distance;
  float waypoint_heading;
  float waypoint_target_lat;
  float waypoint_target_lon;
  float waypoint_target_alt;
  float waypoint_leg_progress;
  float waypoint_mission_progress;
  int32_t waypoint_index;
  int32_t waypoint_total;
  int32_t waypoint_mission_complete;

  float pid_roll_kp;
  float pid_roll_ki;
  float pid_roll_kd;
  float pid_roll_max_output;
  float pid_roll_max_integral;

  float pid_pitch_kp;
  float pid_pitch_ki;
  float pid_pitch_kd;
  float pid_pitch_max_output;
  float pid_pitch_max_integral;

  float pid_yaw_kp;
  float pid_yaw_ki;
  float pid_yaw_kd;
  float pid_yaw_max_output;
  float pid_yaw_max_integral;

  float pid_altitude_kp;
  float pid_altitude_ki;
  float pid_altitude_kd;
  float pid_altitude_max_output;
  float pid_altitude_max_integral;

  float pid_headingerror_kp;
  float pid_headingerror_ki;
  float pid_headingerror_kd;
  float pid_headingerror_max_output;
  float pid_headingerror_max_integral;
};

static_assert(sizeof(TelemetryPacket) == 228, "Telemetry packet size mismatch");

uint32_t g_packet_counter = 0;
uint32_t g_next_command_id = 1;
String g_pending_command;
uint32_t g_last_rx_packet_ms = 0;

void DumpPacketHex(const uint8_t *data, size_t length) {
  Serial.println("Raw payload:");

  for (size_t index = 0; index < length; ++index) {
    const uint8_t value = data[index];
    if ((index % 16U) == 0U) {
      Serial.printf("\n%03u: ", static_cast<unsigned>(index));
    }
    Serial.printf("%02X ", value);
  }

  Serial.println();
}

bool PacketLooksLikeText(const uint8_t *data, size_t length) {
  if (data == nullptr || length == 0U) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    const uint8_t c = data[i];
    const bool printable_ascii = (c >= 32U && c <= 126U);
    const bool whitespace = (c == '\n' || c == '\r' || c == '\t');
    if (!printable_ascii && !whitespace) {
      return false;
    }
  }

  return true;
}

void PrintPacket(const TelemetryPacket &packet, int rssi, float snr) {
  Serial.printf(
      "[%lu] RSSI=%d dBm | SNR=%.1f | mode=%.0f | sats=%ld | lock=%ld\n",
      static_cast<unsigned long>(g_packet_counter),
      rssi,
      snr,
      packet.flightmode,
      static_cast<long>(packet.gps_sats),
      static_cast<long>(packet.gps_lock_acquired));

  Serial.printf(
      "Attitude  roll=%.2f  pitch=%.2f  yaw=%.2f\n",
      packet.roll,
      packet.pitch,
      packet.yaw);

  Serial.printf(
      "Altitude  alt=%.2f  baro=%.2f  target=%.2f\n",
      packet.altitude,
      packet.baro_altitude,
      packet.des_altitude);

  Serial.printf(
      "GPS       lat=%.6f  lon=%.6f  alt=%.2f  spd=%.2f  hdg=%.2f\n",
      packet.gps_lat,
      packet.gps_long,
      packet.gps_alt,
      packet.gps_speed,
      packet.gps_heading);

  Serial.printf(
      "Waypoint  dist=%.2f  hdg=%.2f  idx=%ld/%ld  done=%ld\n",
      packet.waypoint_distance,
      packet.waypoint_heading,
      static_cast<long>(packet.waypoint_index),
      static_cast<long>(packet.waypoint_total),
      static_cast<long>(packet.waypoint_mission_complete));

  Serial.printf(
      "PID      R(%.3f,%.3f,%.3f) P(%.3f,%.3f,%.3f) Y(%.3f,%.3f,%.3f) A(%.3f,%.3f,%.3f) H(%.3f,%.3f,%.3f)\n",
      packet.pid_roll_kp, packet.pid_roll_ki, packet.pid_roll_kd,
      packet.pid_pitch_kp, packet.pid_pitch_ki, packet.pid_pitch_kd,
      packet.pid_yaw_kp, packet.pid_yaw_ki, packet.pid_yaw_kd,
      packet.pid_altitude_kp, packet.pid_altitude_ki, packet.pid_altitude_kd,
      packet.pid_headingerror_kp, packet.pid_headingerror_ki, packet.pid_headingerror_kd);

  Serial.printf(
      "PID Lim  R(%.1f,%.1f) P(%.1f,%.1f) Y(%.1f,%.1f) A(%.1f,%.1f) H(%.1f,%.1f)\n",
      packet.pid_roll_max_output, packet.pid_roll_max_integral,
      packet.pid_pitch_max_output, packet.pid_pitch_max_integral,
      packet.pid_yaw_max_output, packet.pid_yaw_max_integral,
      packet.pid_altitude_max_output, packet.pid_altitude_max_integral,
      packet.pid_headingerror_max_output, packet.pid_headingerror_max_integral);

  Serial.println();
}

bool BuildCommandWithId(const String &raw_command,
                        uint32_t cmd_id,
                        String &command_with_id) {
  command_with_id = raw_command;
  command_with_id.trim();

  if (command_with_id.length() == 0U) {
    return false;
  }

  if (command_with_id.charAt(0) != '{' ||
      command_with_id.charAt(command_with_id.length() - 1U) != '}') {
    return false;
  }

  if (command_with_id.indexOf("\"cmd_id\"") >= 0) {
    return true;
  }

  const bool has_existing_fields = command_with_id.length() > 2U;
  command_with_id.remove(command_with_id.length() - 1U);
  if (has_existing_fields) {
    command_with_id += ",";
  }
  command_with_id += "\"cmd_id\":";
  command_with_id += String(static_cast<unsigned long>(cmd_id));
  command_with_id += "}";
  return true;
}

bool ParseAckPacket(const String &text,
                    uint32_t &cmd_id,
                    bool &ack_success,
                    String &reason) {
  if (text.indexOf("\"type\":\"pid_tuning_ack\"") < 0) {
    return false;
  }

  cmd_id = 0;
  ack_success = text.indexOf("\"status\":\"ack\"") >= 0;

  const int cmd_id_key_index = text.indexOf("\"cmd_id\":");
  if (cmd_id_key_index >= 0) {
    int value_start = cmd_id_key_index + 9;
    while (value_start < text.length() && text.charAt(value_start) == ' ') {
      ++value_start;
    }

    int value_end = value_start;
    while (value_end < text.length()) {
      const char ch = text.charAt(value_end);
      if (ch < '0' || ch > '9') {
        break;
      }
      ++value_end;
    }

    if (value_end > value_start) {
      cmd_id =
          static_cast<uint32_t>(text.substring(value_start, value_end).toInt());
    }
  }

  reason = "unknown";
  const int reason_key_index = text.indexOf("\"reason\":\"");
  if (reason_key_index >= 0) {
    const int reason_start = reason_key_index + 10;
    const int reason_end = text.indexOf('"', reason_start);
    if (reason_end > reason_start) {
      reason = text.substring(reason_start, reason_end);
    }
  }

  return true;
}

bool ProcessOneIncomingPacketForAck(uint32_t expected_cmd_id,
                                    bool &ack_matched,
                                    bool &ack_success) {
  const int packet_size = LoRa.parsePacket();
  if (packet_size <= 0) {
    return false;
  }

  g_last_rx_packet_ms = millis();
  ++g_packet_counter;

  uint8_t payload_buffer[MAX_RX_PACKET_BYTES + 1U] = {};
  const size_t bytes_requested =
      (packet_size > static_cast<int>(MAX_RX_PACKET_BYTES))
          ? MAX_RX_PACKET_BYTES
          : static_cast<size_t>(packet_size);
  const size_t bytes_read = LoRa.readBytes(payload_buffer, bytes_requested);

  while (LoRa.available()) {
    (void)LoRa.read();
  }

  if ((packet_size == static_cast<int>(sizeof(TelemetryPacket))) &&
      (bytes_read == sizeof(TelemetryPacket))) {
    TelemetryPacket packet = {};
    memcpy(&packet, payload_buffer, sizeof(packet));
    PrintPacket(packet, LoRa.packetRssi(), LoRa.packetSnr());
    return true;
  }

  if (PacketLooksLikeText(payload_buffer, bytes_read)) {
    payload_buffer[bytes_read] = '\0';
    const String text_payload =
        String(reinterpret_cast<const char *>(payload_buffer));

    uint32_t ack_cmd_id = 0;
    bool parsed_ack_success = false;
    String reason;
    if (ParseAckPacket(text_payload, ack_cmd_id, parsed_ack_success, reason)) {
      Serial.printf(
          "[%lu] Flight ACK cmd_id=%lu status=%s reason=%s\n\n",
          static_cast<unsigned long>(g_packet_counter),
          static_cast<unsigned long>(ack_cmd_id),
          parsed_ack_success ? "ack" : "nack",
          reason.c_str());

      if (expected_cmd_id != 0U &&
          (ack_cmd_id == expected_cmd_id || ack_cmd_id == 0U)) {
        ack_matched = true;
        ack_success = parsed_ack_success;
      }
      return true;
    }

    Serial.printf("[%lu] Text packet (%dB): %s\n\n",
                  static_cast<unsigned long>(g_packet_counter),
                  packet_size,
                  text_payload.c_str());
    return true;
  }

  Serial.printf(
      "[%lu] Unexpected packet size: %d bytes, expected %u\n",
      static_cast<unsigned long>(g_packet_counter),
      packet_size,
      static_cast<unsigned>(sizeof(TelemetryPacket)));
  DumpPacketHex(payload_buffer, bytes_read);
  return true;
}

void SendCommandOverLoRa(const String &raw_command) {
  uint32_t cmd_id = g_next_command_id++;
  if (cmd_id == 0U) {
    cmd_id = g_next_command_id++;
  }

  String command;
  if (!BuildCommandWithId(raw_command, cmd_id, command)) {
    Serial.println("TX reject: command must be a JSON object line.");
    return;
  }

  if (command.length() > MAX_COMMAND_BYTES) {
    Serial.printf(
        "TX reject: command too long after cmd_id append (%u > %u bytes)\n",
        static_cast<unsigned>(command.length()),
        static_cast<unsigned>(MAX_COMMAND_BYTES));
    return;
  }

  for (uint8_t attempt = 1U; attempt <= COMMAND_MAX_RETRIES; ++attempt) {
    // Wait for a brief quiet window after the latest downlink packet.
    while ((millis() - g_last_rx_packet_ms) < COMMAND_PRE_TX_IDLE_MS) {
      bool ignore_ack = false;
      bool ignore_success = false;
      if (!ProcessOneIncomingPacketForAck(0U, ignore_ack, ignore_success)) {
        delay(5);
      }
    }

    const int begin_status = LoRa.beginPacket();
    if (begin_status != 1) {
      Serial.println("TX failed: LoRa.beginPacket() returned error");
      const uint32_t retry_gap_ms =
          COMMAND_RETRY_GAP_MIN_MS +
          static_cast<uint32_t>(
              random(static_cast<long>(COMMAND_RETRY_GAP_MAX_MS -
                                       COMMAND_RETRY_GAP_MIN_MS + 1UL)));
      delay(retry_gap_ms);
      continue;
    }

    const size_t written = LoRa.print(command);
    const int end_status = LoRa.endPacket();
    LoRa.receive();

    if ((written != command.length()) || (end_status != 1)) {
      Serial.printf("TX failed for cmd_id=%lu attempt %u/%u\n",
                    static_cast<unsigned long>(cmd_id),
                    static_cast<unsigned>(attempt),
                    static_cast<unsigned>(COMMAND_MAX_RETRIES));
      const uint32_t retry_gap_ms =
          COMMAND_RETRY_GAP_MIN_MS +
          static_cast<uint32_t>(
              random(static_cast<long>(COMMAND_RETRY_GAP_MAX_MS -
                                       COMMAND_RETRY_GAP_MIN_MS + 1UL)));
      delay(retry_gap_ms);
      continue;
    }

    Serial.printf("TX cmd_id=%lu attempt %u/%u, waiting for flight ACK...\n",
                  static_cast<unsigned long>(cmd_id),
                  static_cast<unsigned>(attempt),
                  static_cast<unsigned>(COMMAND_MAX_RETRIES));

    bool ack_matched = false;
    bool ack_success = false;
    const unsigned long wait_start = millis();
    while ((millis() - wait_start) < COMMAND_ACK_TIMEOUT_MS) {
      const bool processed_packet =
          ProcessOneIncomingPacketForAck(cmd_id, ack_matched, ack_success);
      if (ack_matched) {
        if (ack_success) {
          Serial.printf("Command cmd_id=%lu confirmed by flight.\n\n",
                        static_cast<unsigned long>(cmd_id));
        } else {
          Serial.printf("Command cmd_id=%lu rejected by flight.\n\n",
                        static_cast<unsigned long>(cmd_id));
        }
        return;
      }

      if (!processed_packet) {
        delay(10);
      }
    }

    Serial.printf("No ACK yet for cmd_id=%lu after %lums.\n",
                  static_cast<unsigned long>(cmd_id),
                  static_cast<unsigned long>(COMMAND_ACK_TIMEOUT_MS));
    const uint32_t retry_gap_ms =
        COMMAND_RETRY_GAP_MIN_MS +
        static_cast<uint32_t>(
            random(static_cast<long>(COMMAND_RETRY_GAP_MAX_MS -
                                     COMMAND_RETRY_GAP_MIN_MS + 1UL)));
    delay(retry_gap_ms);
  }

  Serial.printf("Command cmd_id=%lu failed: no flight ACK after %u attempts.\n\n",
                static_cast<unsigned long>(cmd_id),
                static_cast<unsigned>(COMMAND_MAX_RETRIES));
}

void HandleSerialCommandInput() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());

    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      g_pending_command.trim();

      if (g_pending_command.length() > 0U) {
        SendCommandOverLoRa(g_pending_command);
      }

      g_pending_command = "";
      continue;
    }

    if (g_pending_command.length() < (MAX_COMMAND_BYTES + 16U)) {
      g_pending_command += ch;
    }
  }
}

bool InitLoRa() {
  Serial.printf(
      "LoRa pins: SCK=%d MISO=%d MOSI=%d CS=%d RST=%d IRQ=%d\n",
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
    return false;
  }

  LoRa.setTxPower(LORA_TX_POWER_DBM);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH_HZ);
  LoRa.setCodingRate4(LORA_CODING_RATE_DENOM);
  LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
  LoRa.setSyncWord(SYNC_WORD);
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  if (LORA_ENABLE_CRC) {
    LoRa.enableCrc();
  } else {
    LoRa.disableCrc();
  }
  LoRa.receive();
  return true;
}

void PrintCommandHelp() {
  Serial.println("LoRa command TX enabled.");
  Serial.println("Type one JSON object per line, then press Enter.");
  Serial.println("Ground station auto-adds cmd_id and waits for flight ACK/NACK.");
  Serial.println("Examples:");
  Serial.println("  {\"axis\":\"roll\",\"kp\":18.5}");
  Serial.println("  {\"axis\":\"roll\",\"kp\":18.5,\"ki\":0.02,\"kd\":0.12}");
  Serial.println("  {\"pid\":{\"roll\":{\"kp\":19.0,\"ki\":0.01,\"kd\":0.10}}}");
  Serial.println();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  randomSeed(static_cast<unsigned long>(micros()));

  const unsigned long serial_wait_start = millis();
  while (!Serial && (millis() - serial_wait_start) < 2000UL) {
    delay(10);
  }

  Serial.println();
  Serial.println("ESP32 LoRa telemetry receiver booting...");
  Serial.printf("Expected payload size: %u bytes\n",
                static_cast<unsigned>(sizeof(TelemetryPacket)));

  if (!InitLoRa()) {
    Serial.println("LoRa init failed.");
    Serial.println("Check SX127x wiring and try CS/SCK/MISO/MOSI pin remap.");
    Serial.println("If wired, you can set RST_PIN and IRQ_PIN to real GPIO values.");
    while (true) {
      delay(100);
    }
  }

  Serial.println("LoRa initialized. Waiting for telemetry packets...");
  PrintCommandHelp();
  Serial.println();
}

void loop() {
  HandleSerialCommandInput();

  bool ack_matched = false;
  bool ack_success = false;
  const bool processed_packet =
      ProcessOneIncomingPacketForAck(0, ack_matched, ack_success);
  if (!processed_packet) {
    delay(10);
  }
}
