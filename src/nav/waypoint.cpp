#include "nav/waypoint.h"
#include "math.h"
#include "config.h"


Navigation::Navigation() {
    current_waypoint_index = 0;
    mission_completed = false;
    target_heading = 0.0f;
    target_distance = 0.0f;
}

void Navigation::update(float current_lat, float current_lon, float current_alt){

    if (current_waypoint_index >= num_waypoints){
        mission_completed = true;
        return;
    }
    
    waypoint target_wp = missionwaypoints[current_waypoint_index];

    //?????/////////////calculate distance and hwadinf to targwt_wp from curren loc data

    




    if (target_distance < acceptance_radius) {
        current_waypoint_index++;
    }
    
    

}