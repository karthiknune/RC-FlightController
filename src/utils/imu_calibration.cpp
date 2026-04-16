/**
 * LEVEL CALIBRATION UTILITY
 * Instructions:
 * 1. Place the aircraft in its "Natural Level Flight" attitude.
 * 2. Support it so it is perfectly still.
 * 3. Run this function and copy the resulting offsets to config.h.
 */

#include <Arduino.h>
#include "datatypes.h"
#include "math/pid.h"
#include "config.h"
#include "flight/flightmodes.h"
#include "flight/motormixer.h"
#include "hal/sensors/imu.h"
#include "hal/sensors/baro.h"
#include "hal/sensors/gps.h"
#include "hal/comms/lora.h"
#include "hal/comms/rx_spektrum.h"
#include "hal/actuators/pwm_out.h"
#include "flight/telemetry.h"
#include "nav/waypoint.h"
#include "math/ahrs.h"

extern IMUData_raw currentIMU;
extern IMUData_filtered imu_data;
extern AHRS ahrs;
void Run_Level_Calibration()
{
    Serial.println("\n========================================");
    Serial.println("   AIRCRAFT LEVEL CALIBRATION MODE      ");
    Serial.println("========================================");
    Serial.println("Hold aircraft steady in level flight attitude...");

    // Countdown to allow vibrations to settle
    for (int i = 5; i > 0; i--)
    {
        Serial.printf("Starting in %d...\n", i);
        delay(1000);
    }

    float sum_roll = 0;
    float sum_pitch = 0;
    const int num_samples = 500;

    Serial.println("Recording samples...");
    for (int i = 0; i < num_samples; i++)
    {
        // Read raw hardware
        IMU_Read(currentIMU);
        // Update math (using 0 offsets for this calculation)
        ahrs.update(currentIMU, imu_data);

        sum_roll += imu_data.roll;
        sum_pitch += imu_data.pitch;

        if (i % 50 == 0)
            Serial.print("#");
        delay(10); // 100Hz sampling
    }

    float measured_roll_error = sum_roll / num_samples;
    float measured_pitch_error = sum_pitch / num_samples;

    Serial.println("\n\n--- CALIBRATION RESULTS ---");
    Serial.printf("Average Roll Error:  %6.2f degrees\n", measured_roll_error);
    Serial.printf("Average Pitch Error: %6.2f degrees\n", measured_pitch_error);
    Serial.println("---------------------------");
    Serial.println("COPY THESE VALUES TO include/config.h:");
    Serial.printf("constexpr float LEVEL_ROLL_OFFSET = %6.2f;\n", measured_roll_error);
    Serial.printf("constexpr float LEVEL_PITCH_OFFSET = %6.2f;\n", measured_pitch_error);
    Serial.println("========================================\n");
    delay(5000); // Keep results on screen for a while
}