/**
 * BNO085 Sensor Driver Interface
 *
 * This interface provides a clean, drop-in replacement for raw IMUs (like the ICM-20948).
 * Because the BNO085 handles all complex sensor fusion (Kalman filtering, tilt-compensation,
 * and magnetic interference rejection) on its own internal ARM coprocessor, this interface
 * primarily acts as a bridge to extract the clean ENU (East-North-Up) data into our system.
 */
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
// Once saved, the sensor will automatically load these learned biases on every future boot.
void BNO085_SaveCalibrationToFlash();

// Returns the current calibration accuracy status (0 = Unreliable, 3 = High)
// Used primarily by the calibration utility to know when it is safe to save to flash.
uint8_t BNO085_GetCalibrationStatus();

// Zeros out the current yaw (heading) in software so the drone faces 0 degrees
// This only affects the relative yaw reported to the flight controller, leaving the
// sensor's internal True North tracking completely intact and safe.
void BNO085_TareYaw();

// Fetches the dynamic calibration biases currently evaluated/stored by the sensor
void BNO085_GetCalibrationBiases(float &mag_x, float &mag_y, float &mag_z, float &gyro_x, float &gyro_y, float &gyro_z);
