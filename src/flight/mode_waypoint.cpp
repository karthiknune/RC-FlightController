#include "config.h"
#include "datatypes.h"
#include "flight/flightmodes.h"
#include "flight/motormixer.h"
#include "nav/waypoint.h"
#include "math/pid.h"
#include "hal/comms/rx_spektrum.h"

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

float wrap_heading_error(float heading_error_degrees) {
    while (heading_error_degrees > 180.0f) {
        heading_error_degrees -= 360.0f;
    }

    while (heading_error_degrees < -180.0f) {
        heading_error_degrees += 360.0f;
    }

    return heading_error_degrees;
}

} // namespace

extern PIDController roll_pid;
extern PIDController pitch_pid;
extern PIDController altitude_pid;
//extern PIDController yaw_pid;

extern IMUData_filtered imu_data;
extern BarometerData baro_data;
extern GPSData gps_data;

void mode_waypoint_init() {
    roll_pid.PIDreset();
    pitch_pid.PIDreset(); 
    altitude_pid.PIDreset();    
    //yaw_pid.PIDreset();

}

void mode_waypoint_run(){
    const float throttle_output = get_des_throttle();

    if (navigation.mission_completed() || !gps_data.lock_acquired) {
        const float level_roll_output =
            roll_pid.compute(0.0f, imu_data.roll, WAYPOINT_CONTROL_DT_SECONDS);
        const float level_pitch_output =
            pitch_pid.compute(2.0f, imu_data.pitch, WAYPOINT_CONTROL_DT_SECONDS);
        motormixer_compute(throttle_output, level_roll_output, level_pitch_output, 0.0f);
        return;
    }

    const float target_heading = navigation.get_target_heading();
    const float target_altitude = navigation.get_target_altitude();
    const float actual_heading = gps_data.heading;
    const float actual_altitude = baro_data.healthy ? baro_data.altitude : gps_data.altitude;

    float desired_roll = 0.0f;
    if (gps_data.speed >= WAYPOINT_MIN_GROUND_SPEED_MPS) {
        const float heading_error = wrap_heading_error(target_heading - actual_heading);
        desired_roll = heading_error * WAYPOINT_HEADING_TO_ROLL_KP;
    }
    desired_roll = clamp_value(desired_roll, -max_roll_angle, max_roll_angle);

    float desired_pitch =
        altitude_pid.compute(target_altitude, actual_altitude, WAYPOINT_CONTROL_DT_SECONDS);
    desired_pitch = clamp_value(desired_pitch, -max_pitch_angle, max_pitch_angle);

    const float roll_output =
        roll_pid.compute(desired_roll, imu_data.roll, WAYPOINT_CONTROL_DT_SECONDS);
    const float pitch_output =
        pitch_pid.compute(desired_pitch, imu_data.pitch, WAYPOINT_CONTROL_DT_SECONDS);

    motormixer_compute(throttle_output, roll_output, pitch_output, 0.0f);
}
