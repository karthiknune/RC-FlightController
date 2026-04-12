#pragma once

const int failsafe_timeout = 1000;

void failsafe_timer(); //checks for time until last received signal

bool failsafe_check(); //true if no signal is received for 1 sec


