#include "flight/motormixer.h"
#include "hal/actuators/pwm_out.h"
#include "config.h"


void motormixer_init(){
    setThrottle(1000);
    setAileron(1500);
    setElevator(1500);
    setRudder(1500);
}

void motormixer_compute(float throttle_pid, float roll_pid, float pitch_pid, float yaw_pid){
    
    //calc motor pwm outputs
    int throttle_out = 1000 + throttle_pid;
    int aileron_out = 1500 + roll_pid;
    int elevator_out = 1500 + pitch_pid;
    int rudder_out = 1500 + yaw_pid;

    //clamp pwm outputs
    throttle_out = std::max(1000, std::min(2000, throttle_out));
    aileron_out = std::max(1000, std::min(2000, aileron_out));
    elevator_out = std::max(1000, std::min(2000, elevator_out));
    rudder_out = std::max(1000, std::min(2000, rudder_out));

    // write pwm to motors
    setThrottle(throttle_out);
    setAileron(aileron_out);
    setElevator(elevator_out);
    setRudder(rudder_out);
}