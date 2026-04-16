// #include <Arduino.h>
// #include "datatypes.h"
// #include "math/pid.h"
// #include "config.h"
// #include "flight/flightmodes.h"
// #include "flight/motormixer.h"
// #include "hal/sensors/imu.h"
// #include "hal/sensors/baro.h"
// #include "hal/sensors/gps.h"
// #include "hal/comms/lora.h"
// #include "hal/comms/rx_spektrum.h"
// #include "hal/actuators/pwm_out.h"
// #include "flight/telemetry.h"
// #include "nav/waypoint.h"

// IMUData_raw currentIMU; // Global variable to hold our sensor state
// IMUData_filtered imu_data = {};
// BarometerData baro_data = {};
// GPSData gps_data = {};  // Global variable to hold GPS state
// //FlightMode active_flight_mode = DEFAULT_FLIGHT_MODE;

// ///pid initialisations
// PIDController roll_pid(roll_kp, roll_ki, roll_kd, max_roll_output, max_roll_integral);
// PIDController pitch_pid(pitch_kp, pitch_ki, pitch_kd, max_pitch_output, max_pitch_integral);
// PIDController yaw_pid(yaw_kp, yaw_ki, yaw_kd, max_yaw_output, max_yaw_integral);
// PIDController altitude_pid(alt_kp, alt_ki, alt_kd, max_alt_output, max_alt_integral);

// namespace {

// bool g_flight_mode_initialized = false;

// void UpdateFilteredIMUData() {
//     imu_data.roll = currentIMU.roll;
//     imu_data.pitch = currentIMU.pitch;

//     if (gps_data.lock_acquired && gps_data.speed >= WAYPOINT_MIN_GROUND_SPEED_MPS) {
//         imu_data.yaw = gps_data.heading;
//     }
// }
// /*
// FlightMode DetermineFlightMode() {
//     const float flight_mode_pwm = get_flight_mode_pwm();

//     if (flight_mode_pwm < 900.0f || flight_mode_pwm > 2100.0f) {
//         return DEFAULT_FLIGHT_MODE;
//     }

//     if (flight_mode_pwm <= FLIGHT_MODE_PWM_MANUAL_MAX) {
//         return FlightMode::Manual;
//     }

//     if (flight_mode_pwm <= FLIGHT_MODE_PWM_STABILIZE_MAX) {
//         return FlightMode::Stabilize;
//     }

//     if (flight_mode_pwm <= FLIGHT_MODE_PWM_ALT_HOLD_MAX) {
//         return FlightMode::AltHold;
//     }

//     if (flight_mode_pwm <= FLIGHT_MODE_PWM_GLIDE_MAX) {
//         return FlightMode::Glide;
//     }

//     return FlightMode::Waypoint;
// }

// void InitializeFlightMode(FlightMode mode) {
//     switch (mode) {
//         case FlightMode::Manual:
//             mode_manual_init();
//             break;
//         case FlightMode::Stabilize:
//             mode_stabilize_init();
//             break;
//         case FlightMode::AltHold:
//             mode_alt_hold_init();
//             break;
//         case FlightMode::Glide:
//             mode_glide_init();
//             break;
//         case FlightMode::Waypoint:
//             navigation.restart_mission();
//             mode_waypoint_init();
//             break;
//     }
// }

// void RunFlightMode(FlightMode mode) {
//     switch (mode) {
//         case FlightMode::Manual:
//             mode_manual_run();
//             break;
//         case FlightMode::Stabilize:
//             mode_stabilize_run();
//             break;
//         case FlightMode::AltHold:
//             mode_alt_hold_run();
//             break;
//         case FlightMode::Glide:
//             mode_glide_run();
//             break;
//         case FlightMode::Waypoint:
//             mode_waypoint_run();
//             break;
//     }
// }
// */
// } // namespace

// telemetrydata BuildTelemetrySnapshot() {
//     telemetrydata snapshot = {};

//     snapshot.roll = imu_data.roll;
//     snapshot.pitch = imu_data.pitch;
//     snapshot.yaw = imu_data.yaw;
//     snapshot.altitude = baro_data.healthy ? baro_data.altitude : gps_data.altitude;
//     snapshot.des_altitude = navigation.get_target_altitude();
//     snapshot.gps_lat = static_cast<float>(gps_data.latitude);
//     snapshot.gps_long = static_cast<float>(gps_data.longitude);
//     snapshot.gps_alt = gps_data.altitude;
//     snapshot.gps_speed = gps_data.speed;
//     snapshot.gps_heading = gps_data.heading;
//     snapshot.gps_sats = gps_data.satellites;
//     snapshot.gps_fix_quality = gps_data.fix_quality;
//     snapshot.gps_lock_acquired = gps_data.lock_acquired ? 1 : 0;
//     snapshot.baro_altitude = baro_data.altitude;
//     snapshot.flightmode = static_cast<float>(static_cast<int>(active_flight_mode));
//     snapshot.waypoint_distance = navigation.get_target_distance();
//     snapshot.waypoint_heading = navigation.get_target_heading();
//     snapshot.waypoint_target_alt = navigation.get_target_altitude();
//     snapshot.waypoint_leg_progress = navigation.get_leg_progress_percent();
//     snapshot.waypoint_mission_progress = navigation.get_mission_progress_percent();
//     snapshot.waypoint_index = navigation.get_current_waypoint_index();
//     snapshot.waypoint_total = navigation.get_total_waypoint_count();
//     snapshot.waypoint_mission_complete = navigation.mission_completed() ? 1 : 0;

//     if (snapshot.waypoint_index >= 0 && snapshot.waypoint_index < num_waypoints) {
//         snapshot.waypoint_target_lat =
//             static_cast<float>(missionwaypoints[snapshot.waypoint_index].lat);
//         snapshot.waypoint_target_lon =
//             static_cast<float>(missionwaypoints[snapshot.waypoint_index].lon);
//     }

//     return snapshot;
// }

// void TaskIMURead(void *pvParameters) {
//     TickType_t xLastWakeTime;
//     const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
//     xLastWakeTime = xTaskGetTickCount();

//     for (;;) {
//         IMU_Read(currentIMU);
//         UpdateFilteredIMUData();
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }
// }

// void TaskBarometerRead(void *pvParameters) {
//     TickType_t xLastWakeTime;
//     const TickType_t xFrequency = pdMS_TO_TICKS(BARO_TASK_PERIOD_MS);
//     xLastWakeTime = xTaskGetTickCount();

//     for (;;) {
//         Barometer_Read(baro_data);
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }
// }

// void TaskGPSRead(void *pvParameters) {
//     TickType_t xLastWakeTime;
//     const TickType_t xFrequency = pdMS_TO_TICKS(GPS_TASK_PERIOD_MS);
//     xLastWakeTime = xTaskGetTickCount();

//     for (;;) {
//         if (GPS_Read(gps_data)) {
//             if (gps_data.lock_acquired) {
//                 navigation.update(gps_data.latitude, gps_data.longitude, gps_data.altitude);
//             }
//         }

//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }
// }
// /*
// void TaskFlightControl(void *pvParameters) {
//     TickType_t xLastWakeTime;
//     const TickType_t xFrequency = pdMS_TO_TICKS(FLIGHT_CONTROL_TASK_PERIOD_MS);
//     xLastWakeTime = xTaskGetTickCount();

//     for (;;) {
//         rx_read();

//         const FlightMode desired_mode = DetermineFlightMode();
//         if (!g_flight_mode_initialized || desired_mode != active_flight_mode) {
//             active_flight_mode = desired_mode;
//             InitializeFlightMode(active_flight_mode);
//             g_flight_mode_initialized = true;
//         }

//         RunFlightMode(active_flight_mode);
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }
// }
// */
// void TaskTelemetryTx(void *pvParameters) {
//     TickType_t xLastWakeTime;
//     const TickType_t xFrequency = pdMS_TO_TICKS(TELEMETRY_TASK_PERIOD_MS);
//     xLastWakeTime = xTaskGetTickCount();

//     for (;;) {
//         const telemetrydata snapshot = BuildTelemetrySnapshot();
//         (void)telemetry_send(snapshot);
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }
// }

// void setup() {
//     Serial.begin(115200);
//     while (!Serial);

//     pwm_init();
//     motormixer_init();
//     rx_init();
//     IMU_Init();
//     Barometer_Init();
//     GPS_Init();
//     //navigation.restart_mission();

//     if (!lora_init()) {
//         Serial.println("LoRa init failed.");
//     }

//     xTaskCreatePinnedToCore(
//         TaskIMURead,
//         "IMU_Task",
//         2048,
//         NULL,
//         1,
//         NULL,
//         1
//     );

//     xTaskCreatePinnedToCore(
//         TaskBarometerRead,
//         "Baro_Task",
//         BARO_TASK_STACK_SIZE,
//         NULL,
//         BARO_TASK_PRIORITY,
//         NULL,
//         BARO_TASK_CORE
//     );

//     xTaskCreatePinnedToCore(
//         TaskGPSRead,            // Function to implement the task
//         "GPS_Task",             // Name of the task
//         GPS_TASK_STACK_SIZE,    // Stack size in words
//         NULL,                   // Task input parameter
//         GPS_TASK_PRIORITY,      // Priority
//         NULL,                   // Task handle
//         GPS_TASK_CORE           // Pin to Core 1
//     );
// /*
//     xTaskCreatePinnedToCore(
//         TaskFlightControl,
//         "FlightCtrl_Task",
//         FLIGHT_CONTROL_TASK_STACK_SIZE,
//         NULL,
//         FLIGHT_CONTROL_TASK_PRIORITY,
//         NULL,
//         FLIGHT_CONTROL_TASK_CORE
//     );*/

//     xTaskCreatePinnedToCore(
//         TaskTelemetryTx,
//         "Telemetry_Task",
//         TELEMETRY_TASK_STACK_SIZE,
//         NULL,
//         TELEMETRY_TASK_PRIORITY,
//         NULL,
//         TELEMETRY_TASK_CORE
//     );
// }

// void loop() {
//     // Core 0 handles the default loop(). We will just use it to print data.

//     if (IMU_DEBUG_OUTPUT_ENABLED) {
//         Serial.printf("Roll: %6.2f | Pitch: %6.2f\n", currentIMU.roll, currentIMU.pitch);
//     }

//     delay(50);
// }
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

IMUData_raw currentIMU;
IMUData_filtered imu_data = {};
BarometerData baro_data = {};
GPSData gps_data = {};
FlightMode active_flight_mode = DEFAULT_FLIGHT_MODE;
AHRS ahrs;
void Run_Level_Calibration();

// PID Initializations
PIDController roll_pid(roll_kp, roll_ki, roll_kd, max_roll_output, max_roll_integral);
PIDController pitch_pid(pitch_kp, pitch_ki, pitch_kd, max_pitch_output, max_pitch_integral);
PIDController yaw_pid(yaw_kp, yaw_ki, yaw_kd, max_yaw_output, max_yaw_integral);
PIDController altitude_pid(alt_kp, alt_ki, alt_kd, max_alt_output, max_alt_integral);

namespace
{

    bool g_flight_mode_initialized = false;

    // void UpdateFilteredIMUData()
    // {
    //     imu_data.roll = currentIMU.roll;
    //     imu_data.pitch = currentIMU.pitch;

    //     if (gps_data.lock_acquired && gps_data.speed >= WAYPOINT_MIN_GROUND_SPEED_MPS)
    //     {
    //         imu_data.yaw = gps_data.heading;
    //     }
    // }

    FlightMode DetermineFlightMode()
    {
        // UPDATED: Using the clean rc_data struct we built for the Spektrum receiver
        const float flight_mode_pwm = rc_data.flightmode_pwm;

        if (flight_mode_pwm < 900.0f || flight_mode_pwm > 2100.0f)
        {
            return DEFAULT_FLIGHT_MODE;
        }

        if (flight_mode_pwm <= FLIGHT_MODE_PWM_MANUAL_MAX)
        {
            return FlightMode::Manual;
        }

        if (flight_mode_pwm <= FLIGHT_MODE_PWM_STABILIZE_MAX)
        {
            return FlightMode::Stabilize;
        }

        if (flight_mode_pwm <= FLIGHT_MODE_PWM_ALT_HOLD_MAX)
        {
            return FlightMode::AltHold;
        }

        if (flight_mode_pwm <= FLIGHT_MODE_PWM_GLIDE_MAX)
        {
            return FlightMode::Glide;
        }

        return FlightMode::Waypoint;
    }

    void InitializeFlightMode(FlightMode mode)
    {
        switch (mode)
        {
        case FlightMode::Manual:
            mode_manual_init();
            break;
        case FlightMode::Stabilize:
            mode_stabilize_init();
            break;
        case FlightMode::AltHold:
            mode_alt_hold_init();
            break;
        case FlightMode::Glide:
            mode_glide_init();
            break;
        case FlightMode::Waypoint:
            navigation.restart_mission();
            mode_waypoint_init();
            break;
        }
    }

    void RunFlightMode(FlightMode mode)
    {
        switch (mode)
        {
        case FlightMode::Manual:
            mode_manual_run();
            break;
        case FlightMode::Stabilize:
            mode_stabilize_run();
            break;
        case FlightMode::AltHold:
            mode_alt_hold_run();
            break;
        case FlightMode::Glide:
            mode_glide_run();
            break;
        case FlightMode::Waypoint:
            mode_waypoint_run();
            break;
        }
    }

} // namespace

telemetrydata BuildTelemetrySnapshot()
{
    telemetrydata snapshot = {};

    snapshot.roll = imu_data.roll;
    snapshot.pitch = imu_data.pitch;
    snapshot.yaw = imu_data.yaw;
    snapshot.altitude = baro_data.healthy ? baro_data.altitude : gps_data.altitude;
    snapshot.des_altitude = navigation.get_target_altitude();
    snapshot.gps_lat = static_cast<float>(gps_data.latitude);
    snapshot.gps_long = static_cast<float>(gps_data.longitude);
    snapshot.gps_alt = gps_data.altitude;
    snapshot.gps_speed = gps_data.speed;
    snapshot.gps_heading = gps_data.heading;
    snapshot.gps_sats = gps_data.satellites;
    snapshot.gps_fix_quality = gps_data.fix_quality;
    snapshot.gps_lock_acquired = gps_data.lock_acquired ? 1 : 0;
    snapshot.baro_altitude = baro_data.altitude;
    snapshot.flightmode = static_cast<float>(static_cast<int>(active_flight_mode));
    snapshot.waypoint_distance = navigation.get_target_distance();
    snapshot.waypoint_heading = navigation.get_target_heading();
    snapshot.waypoint_target_alt = navigation.get_target_altitude();
    snapshot.waypoint_leg_progress = navigation.get_leg_progress_percent();
    snapshot.waypoint_mission_progress = navigation.get_mission_progress_percent();
    snapshot.waypoint_index = navigation.get_current_waypoint_index();
    snapshot.waypoint_total = navigation.get_total_waypoint_count();
    snapshot.waypoint_mission_complete = navigation.mission_completed() ? 1 : 0;

    if (snapshot.waypoint_index >= 0 && snapshot.waypoint_index < num_waypoints)
    {
        snapshot.waypoint_target_lat = static_cast<float>(missionwaypoints[snapshot.waypoint_index].lat);
        snapshot.waypoint_target_lon = static_cast<float>(missionwaypoints[snapshot.waypoint_index].lon);
    }

    return snapshot;
}

void TaskIMURead(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz
    xLastWakeTime = xTaskGetTickCount();

    // Start the math timer
    ahrs.init();

    for (;;)
    {
        // 1. Read the hardware
        IMU_Read(currentIMU);

        // 2. Do the math
        ahrs.update(currentIMU, imu_data);

        // 3. Optional: Overwrite Yaw with GPS if moving fast enough
        if (gps_data.lock_acquired && gps_data.speed >= WAYPOINT_MIN_GROUND_SPEED_MPS)
        {
            imu_data.yaw = gps_data.heading;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskBarometerRead(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(BARO_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        Barometer_Read(baro_data);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskGPSRead(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(GPS_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        if (GPS_Read(gps_data))
        {
            if (gps_data.lock_acquired)
            {
                navigation.update(gps_data.latitude, gps_data.longitude, gps_data.altitude);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskFlightControl(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(FLIGHT_CONTROL_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // 1. Read latest radio commands
        rx_read(); // UPDATED: Changed from rx_read() to match our new contract

        // 2. Decide what mode we are in
        const FlightMode desired_mode = DetermineFlightMode();
        if (!g_flight_mode_initialized || desired_mode != active_flight_mode)
        {
            active_flight_mode = desired_mode;
            InitializeFlightMode(active_flight_mode);
            g_flight_mode_initialized = true;
        }

        // 3. Run the math and write to the mixer
        RunFlightMode(active_flight_mode);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskTelemetryTx(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(TELEMETRY_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        const telemetrydata snapshot = BuildTelemetrySnapshot();
        (void)telemetry_send(snapshot);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup()
{
    Serial.begin(115200);
    const unsigned long serial_wait_start = millis();
    while (!Serial && (millis() - serial_wait_start) < 2000UL)
    {
        delay(10);
    }
    Serial.println("Booting flight controller...");

    pwm_init();
    motormixer_init();
    rx_init();
    IMU_Init();
    Run_Level_Calibration();
    IMU_Calibrate_Gyro();
    Barometer_Init();
    GPS_Init();

    if (!lora_init())
    {
        Serial.println("LoRa init failed.");
    }

    // --- NEW: ORIENTATION TESTER BLOCK ---
    Serial.println("\n--- ORIENTATION CHECK MODE (30 SECONDS) ---");
    Serial.println("Move aircraft to verify: Front=+X, Right=+Y, Down=+Z");

    unsigned long testStart = millis();
    while (millis() - testStart < 30000)
    { // 30 second window
        IMU_Read(currentIMU);
        ahrs.update(currentIMU, imu_data);

        // Print raw body-frame accelerations to verify NED gravity
        Serial.printf("AX:%6.2f AY:%6.2f AZ:%6.2f | R:%6.2f P:%6.2f\n",
                      currentIMU.accel_x, currentIMU.accel_y, currentIMU.accel_z,
                      imu_data.roll, imu_data.pitch);
        delay(100);
    }
    Serial.println("--- DIAGNOSTICS COMPLETE. STARTING FLIGHT TASKS ---");
    // -------------------------------------

    // RTOS tasks for concurrent execution of different subsystems
    xTaskCreatePinnedToCore(TaskIMURead, "IMU_Task", 2048, NULL, 1, NULL, 1);
    // xTaskCreatePinnedToCore(TaskBarometerRead, "Baro_Task", BARO_TASK_STACK_SIZE, NULL, BARO_TASK_PRIORITY, NULL, BARO_TASK_CORE);
    xTaskCreatePinnedToCore(TaskGPSRead, "GPS_Task", GPS_TASK_STACK_SIZE, NULL, GPS_TASK_PRIORITY, NULL, GPS_TASK_CORE);
    xTaskCreatePinnedToCore(TaskFlightControl, "FlightCtrl_Task", FLIGHT_CONTROL_TASK_STACK_SIZE, NULL, FLIGHT_CONTROL_TASK_PRIORITY, NULL, FLIGHT_CONTROL_TASK_CORE);
    xTaskCreatePinnedToCore(TaskTelemetryTx, "Telemetry_Task", TELEMETRY_TASK_STACK_SIZE, NULL, TELEMETRY_TASK_PRIORITY, NULL, TELEMETRY_TASK_CORE);
}

void loop()
{
    // Core 0 handles the default loop(). We will just use it to print data.
    if (IMU_DEBUG_OUTPUT_ENABLED)
    {
        // CHANGED: currentIMU -> imu_data
        Serial.printf("Roll: %6.2f | Pitch: %6.2f\n", imu_data.roll, imu_data.pitch);
    }

    delay(50);
}
