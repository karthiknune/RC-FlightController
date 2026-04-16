//////// config file for all pin assignments, tuning parameters and other constants

#pragma once
#include <cstdint>

#include "datatypes.h"
#include "flight/flightmodes.h"

/// pin definintions

// LoRa-------------
// LoRa SPI pins
constexpr int SCK_PIN = 5;
constexpr int MOSI_PIN = 19;
constexpr int MISO_PIN = 21;
// LoRa control pins
constexpr int CS_PIN = 27;
constexpr int RST_PIN = 32;
constexpr int IRQ_PIN = 14;
// LoRa parameters
constexpr long LORA_FREQ = 915000000L;
constexpr uint8_t SYNC_WORD = 0xF3;
constexpr int TELEMETRY_TASK_PERIOD_MS = 500;
constexpr int TELEMETRY_TASK_STACK_SIZE = 4096;
constexpr int TELEMETRY_TASK_PRIORITY = 1;
constexpr int TELEMETRY_TASK_CORE = 1;
// LoRa-------------

// GPS-------------
constexpr int GPS_UART_NUM = 2;
constexpr unsigned long GPS_BAUD_RATE = 115200UL;
constexpr int GPS_TX_PIN = 8;
constexpr int GPS_RX_PIN = 7;
constexpr int GPS_TIME_ZONE_OFFSET = -4; //  UTC-4 for Eastern Daylight Time (EDT)
constexpr int GPS_TASK_PERIOD_MS = 50;
constexpr int GPS_TASK_STACK_SIZE = 4096;
constexpr int GPS_TASK_PRIORITY = 1;
constexpr int GPS_TASK_CORE = 1;
constexpr int GPS_SENTENCE_BUFFER_SIZE = 128;
constexpr int GPS_MAX_FIELDS = 20;
constexpr bool GPS_DEBUG_OUTPUT_ENABLED = true;
// GPS-------------

// IMU-------------
constexpr bool IMU_DEBUG_OUTPUT_ENABLED = true;
constexpr float MAG_OFFSET_X = -18.5f;
constexpr float MAG_OFFSET_Y = -10.0f;
constexpr float MAG_OFFSET_Z = -38.5f;
constexpr float LEVEL_ROLL_OFFSET = 1.50f;
constexpr float LEVEL_PITCH_OFFSET = -2.20f;
// IMU-------------

// I2C-------------
constexpr int I2C_SDA_PIN = 22;
constexpr int I2C_SCL_PIN = 20;
constexpr unsigned long I2C_BUS_FREQUENCY_HZ = 400000UL;
// I2C-------------

// Control-------------
constexpr FlightMode DEFAULT_FLIGHT_MODE = FlightMode::Waypoint;
constexpr float FLIGHT_MODE_PWM_MANUAL_MAX = 1200.0f;
constexpr float FLIGHT_MODE_PWM_STABILIZE_MAX = 1400.0f;
constexpr float FLIGHT_MODE_PWM_ALT_HOLD_MAX = 1600.0f;
constexpr float FLIGHT_MODE_PWM_GLIDE_MAX = 1800.0f;
constexpr int BARO_TASK_PERIOD_MS = 50;
constexpr int BARO_TASK_STACK_SIZE = 4096;
constexpr int BARO_TASK_PRIORITY = 1;
constexpr int BARO_TASK_CORE = 1;
constexpr int FLIGHT_CONTROL_TASK_PERIOD_MS = 100;
constexpr int FLIGHT_CONTROL_TASK_STACK_SIZE = 4096;
constexpr int FLIGHT_CONTROL_TASK_PRIORITY = 1;
constexpr int FLIGHT_CONTROL_TASK_CORE = 1;
// Control-------------

/// Motor and servo pins

// Feather ESP32 V2 safe PWM outputs for bring-up with USB serial, GPS, LoRa, and I2C sensors enabled.
constexpr int esc_pin = 26;
constexpr int aileron_pin = 25;
constexpr int elevator_pin = 4;
constexpr int rudder_pin = 33;

// channels

const int esc_channel = 0;
const int aileron_channel = 1;
const int elevator_channel = 2;
const int rudder_channel = 3;

/// tuning parameters
const float roll_kp = 1.0f;
const float roll_ki = 0.0f;
const float roll_kd = 0.1f;
const float max_roll_output = 500.0f;
const float max_roll_integral = 200.0f;

const float pitch_kp = 1.0f;
const float pitch_ki = 0.0f;
const float pitch_kd = 0.1f;
const float max_pitch_output = 500.0f;
const float max_pitch_integral = 200.0f;

const float yaw_kp = 1.0f;
const float yaw_ki = 0.0f;
const float yaw_kd = 0.1f;
const float max_yaw_output = 500.0f;
const float max_yaw_integral = 200.0f;


// Limits

const float max_roll_angle = 45.0f;
const float max_pitch_angle = 15.0f;
const float max_yaw_angle = 30.0f;
/// for althold pid
const float alt_kp = 1.0f;
const float alt_ki = 0.0f;
const float alt_kd = 0.1f;
const float max_alt_output = max_pitch_angle;
const float max_alt_integral = 10.0f;

// waypoints
constexpr double NAV_EARTH_RADIUS_METERS = 6371000.0;
constexpr float WAYPOINT_ACCEPTANCE_RADIUS_METERS = 5.0f;
constexpr float WAYPOINT_CONTROL_DT_SECONDS = 0.1f;
constexpr float WAYPOINT_HEADING_TO_ROLL_KP = 0.5f;
constexpr float WAYPOINT_MIN_GROUND_SPEED_MPS = 1.5f;

const int num_waypoints = 2;
const waypoint missionwaypoints[] = {

    {2.0536, 1.2189333333333333333, 100},
    {2, 2, 100},

};
