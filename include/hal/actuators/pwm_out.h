#pragma once
#include <Arduino.h>


//////////TO DO: MOVE THESE TO CONFIG.H 
#define Motor_pin 0
#define aileron_pin 1
#define elevator_pin 2
#define rudder_pin 3


void pwm_init();
void pwm_write(uint8_t channel, uint16_t microseconds);
void pwm_reset();