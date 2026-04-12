#pragma once

void motormixer_init(); //sets all the servos to center

void motormixer_compute(float throttle_output, float roll_output, float pitch_output, float yaw_output); //compute motor pwms from pid outs