#pragma once

enum class FlightMode {
    Manual,
    Stabilize,
    AltHold,
    Glide,
    Waypoint
};


void mode_manual_init();
void mode_manual_run();

void mode_stabilize_init();
void mode_stabilize_run();

void mode_alt_hold_init();
void mode_alt_hold_run();
float mode_alt_hold_get_des_pitch();

void mode_glide_init();
void mode_glide_run();

void mode_waypoint_init();
void mode_waypoint_run();
float mode_waypoint_get_des_roll();
float mode_waypoint_get_des_pitch();
