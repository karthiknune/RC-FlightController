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

void mode_glide_init();
void mode_glide_run();

void mode_waypoint_init();
void mode_waypoint_run();
