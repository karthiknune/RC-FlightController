#include "nav/waypoint.h"
#include "math.h"
#include "config.h"


Navigation::Navigation() {
    current_waypoint_index = 0;
    mission_complete = false;
    target_heading = 0.0f;
    target_distance = 0.0f;
}

void Navigation::restart_mission() {
    current_waypoint_index = 0;
    mission_complete = false;
    target_heading = 0.0f;
    target_distance = 0.0f;
}

void Navigation::update(float current_lat, float current_lon, float current_alt){
    (void)current_lat;
    (void)current_lon;
    (void)current_alt;

    if (current_waypoint_index >= num_waypoints){
        mission_complete = true;
        return;
    }
    
    waypoint target_wp = missionwaypoints[current_waypoint_index];

    //?????/////////////calculate distance and hwadinf to targwt_wp from curren loc data

    




    if (target_distance < acceptance_radius) {
        current_waypoint_index++;
    }
    
    

}

float Navigation::get_target_heading() {
    return target_heading;
}

float Navigation::get_target_distance() {
    return target_distance;
}

float Navigation::get_target_altitude() {
    if (current_waypoint_index >= num_waypoints) {
        return 0.0f;
    }

    return missionwaypoints[current_waypoint_index].alt;
}

bool Navigation::mission_completed() {
    return mission_complete;
}
