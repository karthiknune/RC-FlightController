#pragma once

void rx_init();
volatile int* rx_read();
void pwm_reader_task(void *pvParameters);



///rc channel pwm to target roll pitch setpoints
float get_des_roll();
float get_des_pitch();
float get_des_yaw();
float get_des_throttle();
bool get_auto_mode();


