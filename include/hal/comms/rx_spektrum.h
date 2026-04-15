#pragma once

void rx_init();
void rx_read();  ///read rc channel raw pwms



///rc channel pwm to target roll pitch setpoints
float get_des_roll();
float get_des_pitch();
float get_des_yaw();
float get_des_throttle();
float get_flight_mode_pwm();


