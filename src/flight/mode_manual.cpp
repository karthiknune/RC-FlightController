#include "config.h"
#include "flight/flightmodes.h"
#include "hal/sensors/imu.h"
#include "hal/sensors/baro.h"
#include "hal/comms/rx_spektrum.h"
#include "hal/actuators/pwm_out.h"
#include "flight/motormixer.h"

namespace {

float clamp_value(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

} // namespace

void mode_manual_init() {
    
}

void mode_manual_run(){
    const float desired_roll = get_des_roll();
    const float desired_pitch = get_des_pitch();
    const float desired_throttle = get_des_throttle();

    const float roll_output =
        clamp_value((desired_roll / max_roll_angle) * max_roll_output, -max_roll_output, max_roll_output);
    const float pitch_output =
        clamp_value((desired_pitch / max_pitch_angle) * max_pitch_output, -max_pitch_output, max_pitch_output);

    motormixer_compute(desired_throttle, roll_output, pitch_output, 0.0f);
}
