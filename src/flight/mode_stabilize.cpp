  /////////////////////confirm the logic///////////////////////////////////////////////////////////////////

#include "flight/flightmodes.h"
#include "flight/motormixer.h"
#include "math/pid.h"
#include "hal/sensors/imu.h"
#include "hal/comms/rx_spektrum.h"

extern IMUData_filtered imu_data;
extern PIDController roll_pid;
extern PIDController pitch_pid;


void mode_stabilize_init() {
    roll_pid.PIDreset();
    pitch_pid.PIDreset();     
}

void mode_stabilize_run() {
    //from imu
    float actual_roll = imu_data.roll;
    float actual_pitch = imu_data.pitch;
    //stick inputs
    float des_roll = get_des_roll();
    float des_pitch = get_des_pitch();
    float des_throttle = get_des_throttle();

    float dt =0.1f;

    float roll = roll_pid.compute(des_roll , actual_roll, dt);
    float pitch = pitch_pid.compute(des_pitch, actual_pitch, dt);
    float throttle = des_throttle;

    motormixer_compute(throttle, roll, pitch, 0.0f); ///////////////change throttle tp rc_throttle pwm

  
}
