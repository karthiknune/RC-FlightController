#pragma once
#include <Arduino.h>


void pwm_init();
void pwm_write(uint8_t channel, uint16_t microseconds);
void pwm_reset();
