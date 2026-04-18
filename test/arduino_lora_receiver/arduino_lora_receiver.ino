#include <SPI.h>
#include <LoRa.h>
#include <stdint.h>

// Paste one JSON command per line in Arduino IDE Serial Monitor, then press Enter.
// Examples:
// {"axis":"roll","kp":18.5}
// {"axis":"roll","kp":18.5,"ki":0.02,"kd":0.12}
// {"axis":"pitch","max_output":420,"max_integral":120}
// {"pid":{"roll":{"kp":19.0,"ki":0.01,"kd":0.10},"pitch":{"kp":17.0,"ki":0.00,"kd":0.08}}}
// {"axis":"headingerror","kp":0.8,"ki":0.0,"kd":0.04}

// ESP32-WROOM DevKit + SX1276/SX1278 wiring.
// Change these only if your receiver module is wired differently.
struct LoRaPinProfile {
  const char *name;
  int sck;
  int miso;
  int mosi;
  int cs;
  int rst;
  int irq;
};

// Auto-probe common ESP32 + SX127x pin maps.
constexpr LoRaPinProfile kLoRaProfiles[] = {
    {"devkit_ra02_default", 18, 19, 23, 5, 14, 2},
    {"devkit_ra02_common", 5, 19, 27, 18, 14, 26},
    {"feather_style_shared_spi", 5, 21, 19, 26, 4, 39},
    {"devkit_alt_cs", 18, 19, 23, 18, 14, 26},
};
constexpr size_t LORA_PROFILE_COUNT =
    sizeof(kLoRaProfiles) / sizeof(kLoRaProfiles[0]);

constexpr long LORA_FREQ = 915E6;
constexpr uint8_t SYNC_WORD = 0xF3;
constexpr int SPREADING_FACTOR = 7;
constexpr unsigned long SERIAL_BAUD = 115200UL;
constexpr size_t MAX_COMMAND_BYTES = 240U;
constexpr size_t MAX_RX_PACKET_BYTES = 255U;

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
};

static_assert(sizeof(TelemetryPacket) == 128, "Telemetry packet size mismatch");

uint32_t g_packet_counter = 0;
String g_pending_command;
const LoRaPinProfile *g_active_profile = nullptr;

void PrintCommandHelp() {
  Serial.println("LoRa command TX enabled.");
  Serial.println("Type one JSON object per line, then press Enter.");
  Serial.println("Examples:");
  Serial.println("  {\"axis\":\"roll\",\"kp\":18.5}");
  Serial.println("  {\"axis\":\"roll\",\"kp\":18.5,\"ki\":0.02,\"kd\":0.12}");
  Serial.println("  {\"pid\":{\"roll\":{\"kp\":19.0,\"ki\":0.01,\"kd\":0.10}}}");
  Serial.println();
}

void SendCommandOverLoRa(const String &command) {
  if (command.length() == 0U) {
    return;
  }

  if (command.length() > MAX_COMMAND_BYTES) {
    Serial.printf("TX reject: command too long (%u > %u bytes)\n",
                  static_cast<unsigned>(command.length()),
                  static_cast<unsigned>(MAX_COMMAND_BYTES));
    return;
  }

  const int begin_status = LoRa.beginPacket();
  if (begin_status != 1) {
    Serial.println("TX failed: LoRa.beginPacket() returned error");
    return;
  }

  const size_t written = LoRa.print(command);
  const int end_status = LoRa.endPacket();
  LoRa.receive();
  if ((written == command.length()) && (end_status == 1)) {
    Serial.print("TX JSON: ");
    Serial.println(command);
  } else {
    Serial.println("TX failed: incomplete packet write");
  }
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

  Serial.println();
}

bool InitLoRa() {
  for (size_t i = 0; i < LORA_PROFILE_COUNT; ++i) {
    const LoRaPinProfile &profile = kLoRaProfiles[i];

    Serial.printf(
        "Trying LoRa profile %u/%u: %s (SCK=%d MISO=%d MOSI=%d CS=%d RST=%d IRQ=%d)\n",
        static_cast<unsigned>(i + 1U),
        static_cast<unsigned>(LORA_PROFILE_COUNT),
        profile.name,
        profile.sck,
        profile.miso,
        profile.mosi,
        profile.cs,
        profile.rst,
        profile.irq);

    LoRa.end();
    SPI.end();
    delay(20);

    SPI.begin(profile.sck, profile.miso, profile.mosi, profile.cs);
    LoRa.setSPI(SPI);
    LoRa.setPins(profile.cs, profile.rst, profile.irq);

    if (!LoRa.begin(LORA_FREQ)) {
      Serial.println("  -> begin failed\n");
      continue;
    }

    LoRa.setSyncWord(SYNC_WORD);
    LoRa.setSpreadingFactor(SPREADING_FACTOR);
    LoRa.receive();
    g_active_profile = &profile;

    Serial.printf("  -> init success with profile: %s\n\n", profile.name);
    return true;
  }

  return false;
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  const unsigned long serial_wait_start = millis();
  while (!Serial && (millis() - serial_wait_start) < 2000UL) {
    delay(10);
  }

  Serial.println();
  Serial.println("ESP32 LoRa telemetry receiver booting...");
  Serial.printf("Expected payload size: %u bytes\n", static_cast<unsigned>(sizeof(TelemetryPacket)));

  if (!InitLoRa()) {
    Serial.println("LoRa init failed for all pin profiles.");
    Serial.println("Check SX127x wiring, power, and reset/irq pins.");
    while (true) {
      delay(100);
    }
  }

  Serial.println("LoRa initialized. Waiting for telemetry packets...");
  if (g_active_profile != nullptr) {
    Serial.printf("Active profile: %s\n", g_active_profile->name);
  }
  PrintCommandHelp();
  Serial.println();
}

void loop() {
  HandleSerialCommandInput();

  const int packet_size = LoRa.parsePacket();
  if (packet_size <= 0) {
    delay(10);
    return;
  }

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
    return;
  }

  if (PacketLooksLikeText(payload_buffer, bytes_read)) {
    payload_buffer[bytes_read] = '\0';
    Serial.printf("[%lu] Text packet (%dB): %s\n\n",
                  static_cast<unsigned long>(g_packet_counter),
                  packet_size,
                  reinterpret_cast<const char *>(payload_buffer));
    return;
  }

  if (packet_size != static_cast<int>(sizeof(TelemetryPacket))) {
    Serial.printf(
        "[%lu] Unexpected packet size: %d bytes, expected %u bytes\n",
        static_cast<unsigned long>(g_packet_counter),
        packet_size,
        static_cast<unsigned>(sizeof(TelemetryPacket)));
    DumpPacketHex(payload_buffer, bytes_read);
    return;
  }

  if (bytes_read != sizeof(TelemetryPacket)) {
    Serial.printf(
        "[%lu] Short read: got %u of %u bytes\n",
        static_cast<unsigned long>(g_packet_counter),
        static_cast<unsigned>(bytes_read),
        static_cast<unsigned>(sizeof(TelemetryPacket)));
    return;
  }
}
