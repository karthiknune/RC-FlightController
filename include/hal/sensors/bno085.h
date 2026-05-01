#pragma once

#include "datatypes.h"

// Initializes the BNO085 sensor on the shared I2C bus
void BNO085_Init();

// Reads raw data and natively fused roll/pitch/yaw from the BNO085
// Data output aligns perfectly with the standard IMUData_raw interface
// so it acts as a drop-in replacement for the ICM20948 stream.
void BNO085_Read(IMUData_raw &data);
