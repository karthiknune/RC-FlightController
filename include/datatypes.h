#pragma once

struct IMUData_raw {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    float roll;
    float pitch;
    bool healthy; // True if sensor is connected and reading
};

struct IMUData_filtered{
    float roll;
    float pitch;
    float yaw;
};

struct GPSLocalTime {
    int hour;
    int minute;
    int second;
    bool valid;
};

struct GPSRawCoordinates {
    char latitude[16];
    char latitude_dir;
    char longitude[16];
    char longitude_dir;
};

struct GPSData {
    double latitude;
    double longitude;
    float altitude;
    float speed;
    float heading;
    int satellites;
    int fix_quality;
    GPSLocalTime local_time;
    GPSRawCoordinates raw_coordinates;
    bool lock_acquired;
    bool healthy; 
};

struct BarometerData {
    float pressure;
    float altitude;
    bool healthy; 
};


struct RCData {
    float aileron_pwm;
    float elevator_pwm;
    float throttle_pwm;
    float rudder_pwm;
    float flightmode_pwm;
    bool healthy; 
};


struct telemetrydata{
    float roll;
    float pitch;
    float throttle;
    float yaw;
    float des_roll;
    float des_pitch;    
    float des_throttle;
    float des_yaw;
    float altitude;
    float des_altitude;
    float airspeed;
    float gps_lat;
    float gps_long;
    float gps_alt;
    float gps_speed;
    float gps_heading;
    int gps_sats;
    int gps_fix_quality;
    int gps_lock_acquired;
    float baro_altitude;
    float flightmode;
    float failsafe_status;
    float waypoint_distance;
    float waypoint_heading;
    float waypoint_target_lat;
    float waypoint_target_lon;
    float waypoint_target_alt;
    float waypoint_leg_progress;
    float waypoint_mission_progress;
    int waypoint_index;
    int waypoint_total;
    int waypoint_mission_complete;

    // Live PID tuning snapshot (RAM state, sent over LoRa telemetry).
    float pid_roll_kp;
    float pid_roll_ki;
    float pid_roll_kd;
    float pid_roll_max_output;
    float pid_roll_max_integral;

    float pid_pitch_kp;
    float pid_pitch_ki;
    float pid_pitch_kd;
    float pid_pitch_max_output;
    float pid_pitch_max_integral;

    float pid_yaw_kp;
    float pid_yaw_ki;
    float pid_yaw_kd;
    float pid_yaw_max_output;
    float pid_yaw_max_integral;

    float pid_altitude_kp;
    float pid_altitude_ki;
    float pid_altitude_kd;
    float pid_altitude_max_output;
    float pid_altitude_max_integral;

    float pid_headingerror_kp;
    float pid_headingerror_ki;
    float pid_headingerror_kd;
    float pid_headingerror_max_output;
    float pid_headingerror_max_integral;
};


struct waypoint{
    double lat;
    double lon;
    float alt_AGL;          ///////////////agl in m
};

struct HomeState{
    double lat;
    double lon;
    float alt_MSL;          //msl in m
    bool is_set;
};
