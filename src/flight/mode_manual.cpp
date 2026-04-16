#include "config.h"
#include "flight/flightmodes.h"
#include "hal/sensors/imu.h"
#include "hal/sensors/baro.h"
#include "hal/comms/rx_spektrum.h"
#include "hal/actuators/pwm_out.h"
#include "flight/motormixer.h"
#include "hal/comms/rx_spektrum.h"

void mode_manual_init() {
    
}

void mode_manual_run(){

    if(!rc_data.healthy){
        motormixer_compute(0.0f, 0.0f, 0.0f, 0.0f); //kill motors if rc data is not healthy
        return;
    }

    float desired_throttle = rc_data.throttle_pwm-1000.0f; //convert to 0-1000 range
    float desired_roll = rc_data.aileron_pwm - 1500.0f; //convert to -500 to 500 range
    float desired_pitch = rc_data.elevator_pwm - 1500.0f; 
    float desired_yaw = rc_data.rudder_pwm - 1500.0f; 

    motormixer_compute(desired_throttle, desired_roll, desired_pitch, desired_yaw);
}
