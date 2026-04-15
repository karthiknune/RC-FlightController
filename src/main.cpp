#include <Arduino.h>
#include "datatypes.h"
#include "math/pid.h"
#include "config.h"
#include "hal/sensors/imu.h"
#include "hal/sensors/gps.h"
#include "hal/comms/lora.h"

IMUData_raw currentIMU; // Global variable to hold our sensor state
GPSData gps_data = {};  // Global variable to hold GPS state

///pid initialisations
PIDController roll_pid(roll_kp, roll_ki, roll_kd, max_roll_output, max_roll_integral);
PIDController pitch_pid(pitch_kp, pitch_ki, pitch_kd, max_pitch_output, max_pitch_integral);
PIDController yaw_pid(yaw_kp, yaw_ki, yaw_kd, max_yaw_output, max_yaw_integral);
PIDController althold_pid(alt_kp, alt_ki, alt_kd, max_alt_output, max_alt_integral);



// FreeRTOS Task for IMU Reading
void TaskIMURead(void *pvParameters) {
    // 100Hz loop rate
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // Read the sensor and update the global struct
        IMU_Read(currentIMU);
        
        // Block this task precisely until the next 10ms interval
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskGPSRead(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(GPS_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        if (GPS_Read(gps_data) && GPS_DEBUG_OUTPUT_ENABLED) {   // currently, GPS functionality is limited to printing coordinates
            GPS_PrintStatus(Serial, gps_data);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial); 
    
    IMU_Init();
    GPS_Init();
    if (!lora_init()) {
        Serial.println("LoRa init failed.");
    }

    // Create the FreeRTOS Task
    xTaskCreatePinnedToCore(
        TaskIMURead,            // Function to implement the task
        "IMU_Task",             // Name of the task
        2048,                   // Stack size in words
        NULL,                   // Task input parameter
        1,                      // Priority (1 is high)
        NULL,                   // Task handle
        1                       // Pin to Core 1
    );

    xTaskCreatePinnedToCore(
        TaskGPSRead,            // Function to implement the task
        "GPS_Task",             // Name of the task
        GPS_TASK_STACK_SIZE,    // Stack size in words
        NULL,                   // Task input parameter
        GPS_TASK_PRIORITY,      // Priority (1 is high)
        NULL,                   // Task handle
        GPS_TASK_CORE           // Pin to Core 1
    );
}

void loop() {
    // Core 0 handles the default loop(). We will just use it to print data.

    if (IMU_DEBUG_OUTPUT_ENABLED) {
        Serial.printf("Roll: %6.2f | Pitch: %6.2f\n", currentIMU.roll, currentIMU.pitch);
    }

    delay(50); 
}
