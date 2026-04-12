#include "hal/comms/rx_spektrum.h"
#include "config.h"

/// TODO: delete after writing rx_read()
float raw_aileron_pwm = 1500;
float raw_elevator_pwm = 1500;
float raw_throttle_pwm = 1000;


void rx_init() {
    ////kens code
}


void rx_read() {
    
    ///kens code(placeholder for reading raw pwm values from the rx)


    ////return raw_aileron_pwm, raw_elevator_pwm, raw_throttle_pwm
}

float rx_to_angle(float raw_pwm, float max_angle){
    if (raw_pwm<1000)
    {
        raw_pwm = 1000;
    }
    else if (raw_pwm > 2000)
    {
        raw_pwm = 2000;
    }
    return ((raw_pwm-1500.0f)/500.0f) * max_angle;
    
}

float rx_to_throttle(float raw_pwm){
    if (raw_pwm<1050)
    {
        raw_pwm = 1000;
    }
    else if (raw_pwm > 2000)
    {
        raw_pwm = 2000;
    }
    return raw_pwm;
}

float get_des_roll() {
    float des_roll = rx_to_angle(raw_aileron_pwm, max_roll_angle); //////////////////outputs angles
    return des_roll;
}

float get_des_pitch() {
    float des_pitch = rx_to_angle(raw_elevator_pwm, max_pitch_angle);
    return des_pitch;
}

float get_des_throttle(){
    float des_throttle = rx_to_throttle(raw_throttle_pwm);
    return des_throttle;
}

