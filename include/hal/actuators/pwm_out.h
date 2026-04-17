#pragma once
#include <Arduino.h>

void setThrottle(int pulse);
void setAileron(int pulse);
void setRudder(int pulse);
void setElevator(int pulse);
float pwmToThrottle(unsigned long pw);
float pwmToRudder(unsigned long pw);
float pwmToElevator(unsigned long pw);
int ThrottleToPWM(float throttle_percent);
int RudderToPWM(float rudder_angle);
int ElevatorToPWM(float elevator_angle);

void pwm_init();
void pwm_write(uint8_t channel, uint16_t microseconds);
void pwm_reset();
char* debugPWM();