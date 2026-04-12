#include "config.h"
#include "datatypes.h"
#include "flight/flightmodes.h"
#include "nav/waypoint.h"
#include "math/pid.h"
#include "hal/comms/rx_spektrum.h"

extern Navigation navigation;

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

    navigation.update(gps_data.latitude, gps_data.longitude, gps_data.altitude);

    if (!navigation.mission_completed()){
        float target_heading = navigation.get_target_heading();
        
    }

}