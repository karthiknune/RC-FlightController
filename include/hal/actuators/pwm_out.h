#pragma once
#include <Arduino.h>

void pwm_init();
void pwm_write(uint8_t channel, uint16_t microseconds);
uint16_t pwm_surface_angle_to_us(float angle_deg);
void pwm_write_surface_angle(uint8_t channel, float angle_deg);
void pwm_reset();