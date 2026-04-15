//////// config file for all pin assignments, tuning parameters and other constants

#pragma once
#include "datatypes.h"

///pin definintions


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

// LoRa-------------

// GPS-------------
// GPS-------------


///Motor and servo pins

#define esc_pin 0
#define aileron_pin 1
#define elevator_pin 2
#define rudder_pin 3

//channels

const int esc_channel = 0;
const int aileron_channel = 1;
const int elevator_channel = 2;    
const int rudder_channel = 3;

///tuning parameters
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

//Limits

const float max_roll_angle = 45.0f; 
const float max_pitch_angle = 15.0f; 

/// for althold pid
const float alt_kp = 1.0f;
const float alt_ki = 0.0f;
const float alt_kd = 0.1f;
const float max_alt_output = max_pitch_angle;
const float max_alt_integral = 10.0f;


//waypoints

const int num_waypoints = 2;               
const waypoint missionwaypoints[]= {
    
    {2.0536, 1.2189333333333333333, 100},
    {2, 2, 100},
    
};

