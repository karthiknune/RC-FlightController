#include "flight/flightmodes.h"
# include "hal/comms/rx_spektrum.h"
#include "hal/sensors/imu.h"
#include "hal/sensors/baro.h"
#include "hal/sensors/gps.h"
#include "flight/motormixer.h"
#include "math/pid.h"
#include "config.h"


float target_altitude = 50.0f; ///////////althold taget alt

extern IMUData_filtered imu_data;
extern BarometerData baro_data;
extern GPSData gps_data;

extern PIDController roll_pid;
extern PIDController pitch_pid;
extern PIDController altitude_pid;

void mode_alt_hold_init() {
    roll_pid.PIDreset();
    pitch_pid.PIDreset(); 
    altitude_pid.PIDreset();    

    target_altitude = baro_data.healthy ? baro_data.altitude : gps_data.altitude; //set target altitude to current altitude when mode is activated
}

void mode_alt_hold_run(){
    float actual_roll = imu_data.roll;
    float actual_pitch = imu_data.pitch;
    float des_throttle = get_des_throttle();
    float actual_altitude = baro_data.healthy ? baro_data.altitude : gps_data.altitude;

    float des_roll = get_des_roll();
    float des_pitch = get_des_pitch();
    if (baro_data.healthy || gps_data.lock_acquired) {
        des_pitch = altitude_pid.compute(target_altitude, actual_altitude, 0.1f);   //  where is target_altitude updated? 
        if (des_pitch > max_pitch_angle) {
            des_pitch = max_pitch_angle;
        }
        else if (des_pitch < -max_pitch_angle)
        {
            des_pitch = -max_pitch_angle;
        }
    }

    float roll = roll_pid.compute(des_roll,  actual_roll, 0.1f);
    float pitch = pitch_pid.compute(des_pitch, actual_pitch, 0.1f);
    float throttle = des_throttle;

    motormixer_compute(throttle, roll, pitch, 0.0f);

}


