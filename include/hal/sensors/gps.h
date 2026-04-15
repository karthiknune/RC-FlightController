#pragma once

#include <Arduino.h>

#include "datatypes.h"

extern GPSData gps_data;

void GPS_Init();
bool GPS_Read(GPSData &data);
void GPS_PrintStatus(Stream &stream, const GPSData &data);
