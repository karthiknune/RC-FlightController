#include "flight/flightmodes.h"
#include "flight/motormixer.h"
#include "math/pid.h"
#include "hal/sensors/imu.h"
#include "hal/comms/rx_spektrum.h"

extern IMUData_filtered imu_data;
extern PIDController roll_pid;
extern PIDController pitch_pid;

void mode_glide_init() {
    roll_pid.PIDreset();
    pitch_pid.PIDreset();
}

void mode_glide_run() {
    const float desired_roll = get_des_roll();
    const float desired_pitch = 0.0f;

    const float roll_output = roll_pid.compute(desired_roll, imu_data.roll, 0.1f);
    const float pitch_output = pitch_pid.compute(desired_pitch, imu_data.pitch, 0.1f);

    motormixer_compute(0.0f, roll_output, pitch_output, 0.0f);
}
