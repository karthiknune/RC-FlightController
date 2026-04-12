#include "config.h"
#include "flight/flightmodes.h"
#include "hal/sensors/imu.h"
#include "hal/sensors/baro.h"
#include "hal/comms/rx_spektrum.h"
#include "hal/actuators/pwm_out.h"
#include "flight/motormixer.h"

void mode_manual_init() {
    
}

void manual_mode_run(){
    //float des_roll = rx_read().raw_aileron_pwm;
    //float des_pitch = 
    //float des_throttle =

//    motormixer_compute(des_throttle, des_roll, des_pitch, 0.0f); 
}