#pragma once

#include <Arduino.h>

#include "datatypes.h"
#include "config.h"

extern GPSData gps_data;

int GPS_Init();
bool GPS_Read(GPSData &data);
void GPS_PrintStatus(Stream &stream, const GPSData &data);
