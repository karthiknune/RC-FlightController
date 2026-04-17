#include "hal/actuators/pwm_out.h"
#include "config.h"


void pwm_init() {
    // Set up PWM channels with a frequency of 50Hz and 16-bit resolution
    ledcSetup(esc_channel, 50, 16);
    ledcSetup(aileron_channel, 50, 16);
    ledcSetup(elevator_channel, 50, 16);
    ledcSetup(rudder_channel, 50, 16);

    //////attach the channels to the corresponding GPIO pins
    ledcAttachPin(esc_pin, esc_channel);
    ledcAttachPin(aileron_pin, aileron_channel);
    ledcAttachPin(elevator_pin, elevator_channel);
    ledcAttachPin(rudder_pin, rudder_channel);

    pwm_reset();
}

void pwm_write(uint8_t channel, uint16_t microseconds) {

    microseconds = constrain(microseconds, 1000, 2000);

    // convert microseconds to duty cycle based on 16-bit resolution
    uint32_t duty = (microseconds * 65535) / 20000; // 20ms 
    ledcWrite(channel, duty);
}

void pwm_reset() {
    //  neutral positions
    pwm_write(esc_channel, 1000); //    default PWM uS value should be a variable in config.h
    pwm_write(aileron_channel, 1500); 
    pwm_write(elevator_channel, 1500); 
    pwm_write(rudder_channel, 1500); 
}
