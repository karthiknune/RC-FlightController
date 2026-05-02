/**
 * BNO085 Interactive Calibration Utility
 *
 * A guided state-machine utility that safely enables background calibration,
 * guides the user through magnetometer figure-8 tumbling, commands the sensor
 * to burn the offsets to its internal EEPROM, and automatically calculates static
 * Roll/Pitch desk-level offsets for config.h.
 */
#include <Arduino.h>
#include "config.h"
#include "hal/sensors/bno085.h"
#include "hal/sensors/sensor_bus.h"

namespace
{
    constexpr uint32_t kImuReadPeriodMs = 10; // 100 Hz
    constexpr uint32_t kPrintPeriodMs = 250;  // 4 Hz print rate

    IMUData_raw g_imu_data = {};
    uint32_t g_last_imu_read_ms = 0;
    uint32_t g_last_print_ms = 0;

    // Defines the steps of the interactive calibration wizard
    enum CalibState
    {
        STATE_WAIT_START,
        STATE_MAG_PREP,
        STATE_MAG_RECORD,
        STATE_LEVEL_PREP,
        STATE_LEVEL_RECORD,
        STATE_YAW_PREVIEW,
        STATE_FINISHED
    };
    CalibState g_state = STATE_WAIT_START;
    uint32_t g_state_start_ms = 0;
    int g_last_countdown_sec = -1;
    double g_sum_roll = 0.0;
    double g_sum_pitch = 0.0;
    int g_sample_count = 0;
} // namespace

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
    }

    Serial.println("\n============================================");
    Serial.println("   BNO085 CALIBRATION & EEPROM UTILITY      ");
    Serial.println("============================================");

    if (!SensorBus_Init())
    {
        Serial.println("Sensor I2C bus init failed.");
    }

    BNO085_Init();

    // Command the BNO085 to actively figure out its biases right now
    BNO085_EnableBackgroundCalibration();

    Serial.println("BNO085 initialized and Background Calibration ENABLED.");

    // Wait briefly to receive the first uncalibrated reports which contain the biases loaded from Flash
    Serial.println("\nReading existing calibration biases from flash...");
    uint32_t wait_start = millis();
    while (millis() - wait_start < 1500)
    {
        BNO085_Read(g_imu_data);
        delay(10);
    }
    float mx, my, mz, gx, gy, gz;
    BNO085_GetCalibrationBiases(mx, my, mz, gx, gy, gz);
    Serial.println("\n--- EXISTING SENSOR STATE ---");
    Serial.printf(" Flash Mag Bias  [uT]    : X: %6.2f | Y: %6.2f | Z: %6.2f\n", mx, my, mz);
    Serial.printf(" Flash Gyro Bias [rad/s] : X: %6.4f | Y: %6.4f | Z: %6.4f\n", gx, gy, gz);
    Serial.printf(" Config.h Roll Offset    : %6.2f deg\n", IMU_LEVEL_ROLL_OFFSET_DEG);
    Serial.printf(" Config.h Pitch Offset   : %6.2f deg\n", IMU_LEVEL_PITCH_OFFSET_DEG);
    Serial.println("-----------------------------\n");

    Serial.println("Press ENTER in the serial monitor to begin the calibration sequence...");
    g_state = STATE_WAIT_START;
}

void loop()
{
    const uint32_t now_ms = millis();

    if ((now_ms - g_last_imu_read_ms) >= kImuReadPeriodMs)
    {
        g_last_imu_read_ms = now_ms;
        BNO085_Read(g_imu_data);

        if (g_state == STATE_LEVEL_RECORD && g_imu_data.healthy)
        {
            // To calculate the raw level offsets, we add back whatever offset is currently applied
            g_sum_roll += (g_imu_data.roll + IMU_LEVEL_ROLL_OFFSET_DEG);
            g_sum_pitch += (g_imu_data.pitch + IMU_LEVEL_PITCH_OFFSET_DEG);
            g_sample_count++;
        }
    }

    switch (g_state)
    {
    case STATE_WAIT_START:
        if (Serial.available())
        {
            while (Serial.available())
                Serial.read(); // Clear input buffer
            g_state = STATE_MAG_PREP;
            g_state_start_ms = now_ms;
            g_last_countdown_sec = -1;
            Serial.println("\n>>> PHASE 1: MAGNETOMETER CALIBRATION <<<");
            Serial.println("Pick up the aircraft. Get ready to tumble it in figure-8 patterns.");
        }
        break;

    case STATE_MAG_PREP:
    {
        int remain = 5 - ((now_ms - g_state_start_ms) / 1000);
        if (remain != g_last_countdown_sec)
        {
            g_last_countdown_sec = remain;
            if (remain > 0)
                Serial.printf("Starting in %d...\n", remain);
        }
        if (remain <= 0)
        {
            g_state = STATE_MAG_RECORD;
            g_state_start_ms = now_ms;
            g_last_countdown_sec = -1;
            Serial.println("\n[RECORDING] Tumble the aircraft NOW! Keep moving for 15 seconds...");
        }
        break;
    }

    case STATE_MAG_RECORD:
    {
        int remain = 15 - ((now_ms - g_state_start_ms) / 1000);
        if (remain != g_last_countdown_sec)
        {
            g_last_countdown_sec = remain;
            uint8_t acc = BNO085_GetCalibrationStatus();
            if (remain > 0)
                Serial.printf("  Time left: %2ds | Current Accuracy: %d/3\n", remain, acc);
        }
        if (remain <= 0)
        {
            // Trigger the native EEPROM burn. This takes roughly 1.5 - 2 seconds internally.
            Serial.println("\nSaving dynamic calibration to BNO085 internal flash...");
            BNO085_SaveCalibrationToFlash();
            Serial.println("*** FLASH SAVED ***\n");

            g_state = STATE_LEVEL_PREP;
            g_state_start_ms = millis();
            g_last_countdown_sec = -1;
            Serial.println("\n>>> PHASE 2: LEVEL CALIBRATION <<<");
            Serial.println("Place the aircraft PERFECTLY FLAT AND STILL on the desk.");
        }
        break;
    }

    case STATE_LEVEL_PREP:
    {
        int remain = 5 - ((now_ms - g_state_start_ms) / 1000);
        if (remain != g_last_countdown_sec)
        {
            g_last_countdown_sec = remain;
            if (remain > 0)
                Serial.printf("Starting in %d...\n", remain);
        }
        if (remain <= 0)
        {
            g_sum_roll = 0.0;
            g_sum_pitch = 0.0;
            g_sample_count = 0;
            g_state = STATE_LEVEL_RECORD;
            g_state_start_ms = now_ms;
            g_last_countdown_sec = -1;
            Serial.println("\n[RECORDING] Keep aircraft STILL for 5 seconds...");
        }
        break;
    }

    case STATE_LEVEL_RECORD:
    {
        int remain = 5 - ((now_ms - g_state_start_ms) / 1000);
        if (remain != g_last_countdown_sec)
        {
            g_last_countdown_sec = remain;
            if (remain > 0)
                Serial.printf("  Recording... %ds left\n", remain);
        }
        if (remain <= 0)
        {
            if (g_sample_count > 0)
            {
                float new_roll_offset = g_sum_roll / g_sample_count;
                float new_pitch_offset = g_sum_pitch / g_sample_count;

                Serial.println("\n============================================");
                Serial.println("         CALIBRATION COMPLETE               ");
                Serial.println("============================================");
                Serial.println("BNO085 Mag/Gyro offsets are already burned into its flash memory.");
                Serial.println("\nFor the Roll/Pitch leveling, copy these lines into include/config.h:");
                Serial.printf("constexpr float IMU_LEVEL_ROLL_OFFSET_DEG = %6.2ff;\n", new_roll_offset);
                Serial.printf("constexpr float IMU_LEVEL_PITCH_OFFSET_DEG = %6.2ff;\n", new_pitch_offset);
                Serial.println("============================================");
                Serial.println("\nStreaming raw data for 5 seconds before snapping Yaw to 0...");
            }
            else
            {
                Serial.println("\nError: No samples collected during level calibration.");
            }

            g_state = STATE_YAW_PREVIEW;
            g_state_start_ms = now_ms;
            g_last_countdown_sec = -1;
        }
        break;
    }

    case STATE_YAW_PREVIEW:
    {
        int remain = 5 - ((now_ms - g_state_start_ms) / 1000);

        if ((now_ms - g_last_print_ms) >= kPrintPeriodMs)
        {
            g_last_print_ms = now_ms;
            if (g_imu_data.healthy)
            {
                uint8_t accuracy = BNO085_GetCalibrationStatus();
                Serial.printf("Acc: %d/3 | Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f\n",
                              accuracy, g_imu_data.roll, g_imu_data.pitch, g_imu_data.yaw);
            }
        }

        if (remain <= 0)
        {
            BNO085_TareYaw();
            Serial.println("\n*** YAW SNAPPED TO 0 DEGREES ***");
            Serial.println("\nNow streaming live data (Press 't' to tare Yaw again).");
            g_state = STATE_FINISHED;
        }
        break;
    }

    case STATE_FINISHED:
    {
        if (Serial.available())
        {
            char c = Serial.read();
            if (c == 't' || c == 'T')
            {
                BNO085_TareYaw();
                Serial.println("\n*** YAW TARED TO 0 DEGREES ***\n");
            }
        }

        if ((now_ms - g_last_print_ms) >= kPrintPeriodMs)
        {
            g_last_print_ms = now_ms;
            if (g_imu_data.healthy)
            {
                uint8_t accuracy = BNO085_GetCalibrationStatus();
                Serial.printf("Acc: %d/3 | Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f\n",
                              accuracy, g_imu_data.roll, g_imu_data.pitch, g_imu_data.yaw);
            }
        }
        break;
    }
    }
}
