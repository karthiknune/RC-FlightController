#pragma once
#include "datatypes.h"
#include "config.h"

class Navigation{
    public:
        Navigation();

        void restart_mission(); 
        
        void update(float current_lat, float current_lon, float current_alt);

        
        float get_target_heading();
        float get_target_distance();
        float get_target_altitude();

        bool mission_completed();

    private:
        int current_waypoint_index;
        
        //intitiates
        bool mission_complete = false;
        float target_heading = 0.0f;
        float target_distance = 0.0f;
        

        const float acceptance_radius = 5.0f; ////wp achieved if within 5m from the defined coordnate
        

};  



