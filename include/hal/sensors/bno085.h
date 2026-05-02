#pragma once

#include "datatypes.h"

// Initializes the BNO085 sensor on the shared I2C bus
void BNO085_Init();

// Reads raw data and natively fused roll/pitch/yaw from the BNO085
// Data output aligns perfectly with the standard IMUData_raw interface
// so it acts as a drop-in replacement for the ICM20948 stream.
void BNO085_Read(IMUData_raw &data);

// Commands the BNO085 to actively calibrate Accel, Gyro, and Mag in the background
void BNO085_EnableBackgroundCalibration();

// Saves the current dynamic calibration data (DCD) to the BNO085's internal flash memory
void BNO085_SaveCalibrationToFlash();

// Returns the current calibration accuracy status (0 = Unreliable, 3 = High)
uint8_t BNO085_GetCalibrationStatus();

// Zeros out the current yaw (heading) in software so the drone faces 0 degrees
void BNO085_TareYaw();

// Fetches the dynamic calibration biases currently evaluated/stored by the sensor
void BNO085_GetCalibrationBiases(float &mag_x, float &mag_y, float &mag_z, float &gyro_x, float &gyro_y, float &gyro_z);
