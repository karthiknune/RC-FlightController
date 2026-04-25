#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "datatypes.h"
#include "math/pid.h"
#include "config.h"
#include "flight/flightmodes.h"
#include "flight/motormixer.h"
#include "hal/sensors/imu.h"
#include "hal/sensors/baro.h"
#include "hal/sensors/sensor_bus.h"
#include "hal/sensors/gps.h"
#include "hal/sensors/airspeed.h"
#include "hal/comms/lora.h"
#include "hal/comms/lora_protocol.h"
#include "hal/comms/rx_spektrum.h"
#include "hal/actuators/pwm_out.h"
#include "flight/arming.h"
#include "flight/telemetry.h"
#include "logging/sd_logger.h"
#include "nav/waypoint.h"
#include "flight/home.h"
#include "status/status_led.h"

IMUData_raw currentIMU; // Global variable to hold our sensor state
IMUData_filtered imu_data = {};
BarometerData baro_data = {};
GPSData gps_data = {};  // Global variable to hold GPS state
AirspeedData airspeed_data = {};
FlightMode active_flight_mode = DEFAULT_FLIGHT_MODE;

///pid initialisations
PIDController roll_pid(roll_kp, roll_ki, roll_kd, max_roll_output, max_roll_integral);
PIDController pitch_pid(pitch_kp, pitch_ki, pitch_kd, max_pitch_output, max_pitch_integral);
PIDController yaw_pid(yaw_kp, yaw_ki, yaw_kd, max_yaw_output, max_yaw_integral);
PIDController altitude_pid(alt_kp, alt_ki, alt_kd, max_alt_output, max_alt_integral);
PIDController headingerror_pid(headingerror_kp, headingerror_ki, headingerror_kd, max_headingerror_output, max_headingerror_integral);

namespace {

bool g_flight_mode_initialized = false;
PIDTuningValues g_roll_pid_tuning = {roll_kp, roll_ki, roll_kd};
PIDTuningValues g_pitch_pid_tuning = {pitch_kp, pitch_ki, pitch_kd};
PIDTuningValues g_yaw_pid_tuning = {yaw_kp, yaw_ki, yaw_kd};
LoRaPIDCommandPacket g_pending_pid_command = {};
volatile bool g_pending_pid_command_valid = false;
LoRaPIDAckPacket g_last_pid_ack = {};
volatile bool g_last_pid_ack_valid = false;
portMUX_TYPE g_pid_command_lock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_tuning_lock = portMUX_INITIALIZER_UNLOCKED;

bool IsPIDTuningFinite(const PIDTuningValues &tuning) {
    return isfinite(tuning.kp) && isfinite(tuning.ki) && isfinite(tuning.kd);
}

bool IsValidPIDAxisMask(uint8_t axis_mask) {
    return (axis_mask & LORA_PID_AXIS_ALL) != 0U &&
           (axis_mask & ~LORA_PID_AXIS_ALL) == 0U;
}

bool IsValidPIDCommand(const LoRaPIDCommandPacket &packet) {
    if (packet.magic != LORA_PID_PROTOCOL_MAGIC ||
        packet.type != static_cast<uint8_t>(LoRaMessageType::PIDCommand) ||
        !IsValidPIDAxisMask(packet.axis_mask)) {
        return false;
    }

    if ((packet.axis_mask & LORA_PID_AXIS_ROLL) != 0U &&
        !IsPIDTuningFinite(packet.roll)) {
        return false;
    }

    if ((packet.axis_mask & LORA_PID_AXIS_PITCH) != 0U &&
        !IsPIDTuningFinite(packet.pitch)) {
        return false;
    }

    if ((packet.axis_mask & LORA_PID_AXIS_YAW) != 0U &&
        !IsPIDTuningFinite(packet.yaw)) {
        return false;
    }

    return true;
}

void PrintPIDTuning(const char *axis_name, const PIDTuningValues &tuning) {
    Serial.printf("%s PID: Kp=%.3f Ki=%.3f Kd=%.3f\n",
                  axis_name,
                  tuning.kp,
                  tuning.ki,
                  tuning.kd);
}

void SetControllerTuning(PIDController &controller,
                         PIDTuningValues &runtime_tuning,
                         const PIDTuningValues &new_tuning,
                         const char *axis_name) {
    portENTER_CRITICAL(&g_tuning_lock);
    runtime_tuning = new_tuning;
    controller.setTuning(new_tuning.kp, new_tuning.ki, new_tuning.kd);
    portEXIT_CRITICAL(&g_tuning_lock);
    Serial.printf("[PID] %s updated to Kp=%.3f Ki=%.3f Kd=%.3f\n",
                  axis_name,
                  new_tuning.kp,
                  new_tuning.ki,
                  new_tuning.kd);
}

void LoadPIDTuningsFromConfig() {
    Serial.println("Loading PID defaults from config...");
    SetControllerTuning(roll_pid, g_roll_pid_tuning, {roll_kp, roll_ki, roll_kd}, "Roll");
    SetControllerTuning(pitch_pid, g_pitch_pid_tuning, {pitch_kp, pitch_ki, pitch_kd}, "Pitch");
    SetControllerTuning(yaw_pid, g_yaw_pid_tuning, {yaw_kp, yaw_ki, yaw_kd}, "Yaw");

    if (SD_LOGGING_ENABLED && SD_Logger_IsReady()) {
        float r_kp, r_ki, r_kd, p_kp, p_ki, p_kd, y_kp, y_ki, y_kd;
        if (SD_Logger_LoadPIDConfig(r_kp, r_ki, r_kd, p_kp, p_ki, p_kd, y_kp, y_ki, y_kd)) {
            Serial.println("Valid pid_config.json found. Applying saved PID values...");
            SetControllerTuning(roll_pid, g_roll_pid_tuning, {r_kp, r_ki, r_kd}, "Roll");
            SetControllerTuning(pitch_pid, g_pitch_pid_tuning, {p_kp, p_ki, p_kd}, "Pitch");
            SetControllerTuning(yaw_pid, g_yaw_pid_tuning, {y_kp, y_ki, y_kd}, "Yaw");
        } else {
            Serial.println("No valid pid_config.json found on SD card. Proceeding with defaults.");
        }
    }

    PrintPIDTuning("Roll", g_roll_pid_tuning);
    PrintPIDTuning("Pitch", g_pitch_pid_tuning);
    PrintPIDTuning("Yaw", g_yaw_pid_tuning);
}

void QueuePendingPIDCommand(const LoRaPIDCommandPacket &packet) {
    portENTER_CRITICAL(&g_pid_command_lock);
    g_pending_pid_command = packet;
    g_pending_pid_command_valid = true;
    portEXIT_CRITICAL(&g_pid_command_lock);
}

bool TakePendingPIDCommand(LoRaPIDCommandPacket &packet) {
    bool has_pending = false;

    portENTER_CRITICAL(&g_pid_command_lock);
    if (g_pending_pid_command_valid) {
        packet = g_pending_pid_command;
        g_pending_pid_command_valid = false;
        has_pending = true;
    }
    portEXIT_CRITICAL(&g_pid_command_lock);

    return has_pending;
}

void CacheLastPIDAck(const LoRaPIDAckPacket &packet) {
    portENTER_CRITICAL(&g_pid_command_lock);
    g_last_pid_ack = packet;
    g_last_pid_ack_valid = true;
    portEXIT_CRITICAL(&g_pid_command_lock);
}

bool GetCachedPIDAck(uint8_t sequence, LoRaPIDAckPacket &packet) {
    bool found = false;

    portENTER_CRITICAL(&g_pid_command_lock);
    if (g_last_pid_ack_valid && g_last_pid_ack.sequence == sequence) {
        packet = g_last_pid_ack;
        found = true;
    }
    portEXIT_CRITICAL(&g_pid_command_lock);

    return found;
}

void SendPIDAck(const LoRaPIDAckPacket &packet, const char *context) {
    const bool ack_sent = lora_send(reinterpret_cast<const uint8_t *>(&packet),
                                    sizeof(packet));
    Serial.printf("[LoRa ACK] seq=%u %s (%s)\n",
                  static_cast<unsigned>(packet.sequence),
                  ack_sent ? "sent" : "send failed",
                  context);
}

void ApplyPendingPIDCommandIfNeeded() {
    LoRaPIDCommandPacket packet = {};
    if (!TakePendingPIDCommand(packet)) {
        return;
    }

    Serial.printf("[LoRa RX] Applying PID command seq=%u mask=0x%02X\n",
                  static_cast<unsigned>(packet.sequence),
                  static_cast<unsigned>(packet.axis_mask));

    if ((packet.axis_mask & LORA_PID_AXIS_ROLL) != 0U) {
        SetControllerTuning(roll_pid, g_roll_pid_tuning, packet.roll, "Roll");
    }

    if ((packet.axis_mask & LORA_PID_AXIS_PITCH) != 0U) {
        SetControllerTuning(pitch_pid, g_pitch_pid_tuning, packet.pitch, "Pitch");
    }

    if ((packet.axis_mask & LORA_PID_AXIS_YAW) != 0U) {
        SetControllerTuning(yaw_pid, g_yaw_pid_tuning, packet.yaw, "Yaw");
    }

    LoRaPIDAckPacket ack = {};
    ack.magic = LORA_PID_PROTOCOL_MAGIC;
    ack.type = static_cast<uint8_t>(LoRaMessageType::PIDAck);
    ack.sequence = packet.sequence;
    ack.axis_mask = packet.axis_mask;
    ack.status = static_cast<uint8_t>(LoRaPIDAckStatus::Applied);

    portENTER_CRITICAL(&g_tuning_lock);
    ack.roll = g_roll_pid_tuning;
    ack.pitch = g_pitch_pid_tuning;
    ack.yaw = g_yaw_pid_tuning;
    portEXIT_CRITICAL(&g_tuning_lock);

    CacheLastPIDAck(ack);
    SendPIDAck(ack, "applied");

    if (SD_LOGGING_ENABLED && SD_Logger_IsReady()) {
        PIDTuningValues roll_copy, pitch_copy, yaw_copy;
        portENTER_CRITICAL(&g_tuning_lock);
        roll_copy = g_roll_pid_tuning;
        pitch_copy = g_pitch_pid_tuning;
        yaw_copy = g_yaw_pid_tuning;
        portEXIT_CRITICAL(&g_tuning_lock);

        if (SD_Logger_SavePIDConfig(
                roll_copy.kp, roll_copy.ki, roll_copy.kd,
                pitch_copy.kp, pitch_copy.ki, pitch_copy.kd,
                yaw_copy.kp, yaw_copy.ki, yaw_copy.kd)) {
            Serial.println("[SD] Updated PID config saved to pid_config.json");
        } else {
            Serial.println("[SD] Failed to save PID config to SD card.");
        }
    }
}

void RunStartupIMUCalibrationIfEnabled() {
    if (IMU_RUN_STARTUP_GYRO_CALIBRATION) {
        (void)IMU_Calibrate_Gyro();
    }

    if (IMU_RUN_STARTUP_LEVEL_CALIBRATION) {
        float roll_offset_deg = 0.0f;
        float pitch_offset_deg = 0.0f;
        (void)IMU_Run_Level_Calibration(roll_offset_deg, pitch_offset_deg);
    }

    if (IMU_RUN_MAG_CALIBRATION) {
        float offset_x = 0.0f, offset_y = 0.0f, offset_z = 0.0f;
        float scale_x = 1.0f, scale_y = 1.0f, scale_z = 1.0f;
        (void)IMU_Run_Mag_Calibration(offset_x, offset_y, offset_z, scale_x, scale_y, scale_z);
    }
}

void UpdateFilteredIMUData() {
    if (currentIMU.healthy) {
        imu_data.roll = currentIMU.roll;
        imu_data.pitch = currentIMU.pitch;
        imu_data.yaw = currentIMU.yaw;
    }
}

FlightMode ResolveFlightModeFallback(FlightMode requested_mode) {
    if (requested_mode != FlightMode::Manual && !currentIMU.healthy) {
        return FlightMode::Manual;
    }

    return requested_mode;
}

FlightMode DetermineFlightMode() {
    const float flight_mode_pwm = get_flight_mode_pwm();

    if (flight_mode_pwm <= FLIGHT_MODE_PWM_MANUAL_MAX) {
        return FlightMode::Manual;
    }

    if (flight_mode_pwm <= FLIGHT_MODE_PWM_STABILIZE_MAX) {
        return FlightMode::Stabilize;
    }

    if (flight_mode_pwm <= FLIGHT_MODE_PWM_ALT_HOLD_MAX) {
        return FlightMode::AltHold;
    }
    return FlightMode::Glide;
}

void InitializeFlightMode(FlightMode mode) {
    switch (mode) {
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

void RunFlightMode(FlightMode mode) {
    switch (mode) {
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

telemetrydata BuildTelemetrySnapshot() {
    telemetrydata snapshot = {};

    snapshot.roll = imu_data.roll;
    snapshot.pitch = imu_data.pitch;
    snapshot.yaw = imu_data.yaw;
    snapshot.des_roll     = get_des_roll();
    snapshot.des_pitch    = get_des_pitch();
    snapshot.des_yaw      = get_des_yaw();
    snapshot.des_throttle = get_des_throttle();
    snapshot.altitude = baro_data.healthy ? baro_data.altitude : gps_data.altitude;
    snapshot.des_altitude = navigation.get_target_altitude();
    snapshot.airspeed = airspeed_data.healthy ? airspeed_data.airspeed_mps : 0.0f;
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

    snapshot.imu_healthy  = currentIMU.healthy  ? 1 : 0;
    snapshot.baro_healthy = baro_data.healthy   ? 1 : 0;
    snapshot.gps_healthy  = gps_data.healthy    ? 1 : 0;
    snapshot.rx_healthy   = rc_data.healthy     ? 1 : 0;
    snapshot.armed        = is_armed() ? 1 : 0;

    snapshot.roll_pid_out  = roll_pid.getLastOutput();
    snapshot.pitch_pid_out = pitch_pid.getLastOutput();
    snapshot.yaw_pid_out   = yaw_pid.getLastOutput();

    portENTER_CRITICAL(&g_tuning_lock);
    snapshot.roll_pid_kp = roll_pid.getkp();
    snapshot.roll_pid_ki = roll_pid.getki();
    snapshot.roll_pid_kd = roll_pid.getkd();
    snapshot.pitch_pid_kp = pitch_pid.getkp();
    snapshot.pitch_pid_ki = pitch_pid.getki();
    snapshot.pitch_pid_kd = pitch_pid.getkd();
    snapshot.yaw_pid_kp = yaw_pid.getkp();
    snapshot.yaw_pid_ki = yaw_pid.getki();
    snapshot.yaw_pid_kd = yaw_pid.getkd();
    snapshot.altitude_pid_kp = altitude_pid.getkp();
    snapshot.altitude_pid_ki = altitude_pid.getki();
    snapshot.altitude_pid_kd = altitude_pid.getkd();
    snapshot.headingerror_pid_kp = headingerror_pid.getkp();
    snapshot.headingerror_pid_ki = headingerror_pid.getki();
    snapshot.headingerror_pid_kd = headingerror_pid.getkd();
    portEXIT_CRITICAL(&g_tuning_lock);

    snapshot.rx_throttle_pwm = rc_data.throttle_pwm;
    snapshot.rx_aileron_pwm  = rc_data.aileron_pwm;
    snapshot.rx_elevator_pwm = rc_data.elevator_pwm;
    snapshot.rx_rudder_pwm   = rc_data.rudder_pwm;
    snapshot.rx_mode_pwm     = rc_data.flightmode_pwm;

    if (snapshot.waypoint_index >= 0 && snapshot.waypoint_index < num_waypoints) {
        snapshot.waypoint_target_lat =
            static_cast<float>(missionwaypoints[snapshot.waypoint_index].lat);
        snapshot.waypoint_target_lon =
            static_cast<float>(missionwaypoints[snapshot.waypoint_index].lon);
    }

    return snapshot;
}

void TaskIMURead(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(IMU_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        IMU_Read(currentIMU);
        UpdateFilteredIMUData();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskBarometerRead(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(BARO_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        Barometer_Read(baro_data);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskAirspeedRead(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(AIRSPEED_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        Airspeed_Read(airspeed_data);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskGPSRead(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(GPS_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        if (GPS_Read(gps_data)) {
            if (gps_data.lock_acquired) {

                if(!home_is_set()) {
                    float home_msl = baro_data.healthy ? baro_data.altitude : gps_data.altitude;
                    set_home_location(gps_data.latitude, gps_data.longitude, home_msl);
                    Serial.println("Home location set.");

                }

                navigation.update(gps_data.latitude, gps_data.longitude, gps_data.altitude);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskFlightControl(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(FLIGHT_CONTROL_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();
    bool prev_armed = false;

    for (;;) {
        ApplyPendingPIDCommandIfNeeded();
        arming_update();
        const bool armed = is_armed();

        if (!armed) {
            motormixer_compute(0.0f, 0.0f, 0.0f, 0.0f);
            prev_armed = false;
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // Force flight mode re-init on the first tick after arming.
        if (!prev_armed) {
            g_flight_mode_initialized = false;
        }
        prev_armed = true;

        const FlightMode desired_mode = DetermineFlightMode();
        const FlightMode effective_mode = ResolveFlightModeFallback(desired_mode);
        if (!g_flight_mode_initialized || effective_mode != active_flight_mode) {
            active_flight_mode = effective_mode;
            InitializeFlightMode(active_flight_mode);
            g_flight_mode_initialized = true;
        }

        RunFlightMode(active_flight_mode);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskLoRaRx(void *pvParameters) {
    if (!LORA_LOGGING_ENABLED) {
        vTaskDelete(NULL);
        return;
    }

    uint8_t buffer[sizeof(LoRaPIDCommandPacket)] = {};

    for (;;) {
        const size_t bytes_read = lora_receive(buffer, sizeof(buffer));
        if (bytes_read == sizeof(LoRaPIDCommandPacket)) {
            LoRaPIDCommandPacket packet = {};
            memcpy(&packet, buffer, sizeof(packet));

            if (!IsValidPIDCommand(packet)) {
                Serial.println("[LoRa RX] Ignored invalid PID command packet.");
            } else {
                LoRaPIDAckPacket cached_ack = {};
                if (GetCachedPIDAck(packet.sequence, cached_ack)) {
                    Serial.printf("[LoRa RX] Duplicate PID command seq=%u. Resending ACK.\n",
                                  static_cast<unsigned>(packet.sequence));
                    SendPIDAck(cached_ack, "duplicate");
                } else {
                    QueuePendingPIDCommand(packet);
                    Serial.printf("[LoRa RX] Queued PID command seq=%u mask=0x%02X\n",
                                  static_cast<unsigned>(packet.sequence),
                                  static_cast<unsigned>(packet.axis_mask));
                }
            }
        } else if (bytes_read > 0U) {
            Serial.printf("[LoRa RX] Ignored unexpected packet of %u bytes.\n",
                          static_cast<unsigned>(bytes_read));
        }

        vTaskDelay(pdMS_TO_TICKS(LORA_RX_TASK_PERIOD_MS));
    }
}

void TaskTelemetryTx(void *pvParameters) {
    if (!LORA_LOGGING_ENABLED) {
        vTaskDelete(NULL);
        return;
    }

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(TELEMETRY_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        const telemetrydata snapshot = BuildTelemetrySnapshot();
        (void)telemetry_send(snapshot);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskSDLog(void *pvParameters) {
    if (!SD_LOGGING_ENABLED) {
        vTaskDelete(NULL);
        return;
    }

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(SD_LOG_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        const telemetrydata snapshot = BuildTelemetrySnapshot();
        SD_Logger_LogData(snapshot);

        // Periodically flush the file to prevent data loss on power failure
        static int flush_counter = 0;
        if (++flush_counter >= 20) { // Flush every 1 second at 20Hz
            SD_Logger_Flush();
            flush_counter = 0;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskStatusLED(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(STATUS_LED_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        StatusLED_Update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial); 

    // Shared SPI bus safety: deselect peripherals before driver init.
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);

    StatusLED_Init();
    
    if (!SensorBus_Init()) {
        Serial.println("Sensor I2C bus init failed.");
    }

    pwm_init();
    motormixer_init();
    rx_init();
    arming_init();
    IMU_Init();
    RunStartupIMUCalibrationIfEnabled();
    Barometer_Init();
    Airspeed_Init();
    GPS_Init();
    navigation.restart_mission();

    if (LORA_LOGGING_ENABLED) {        
        if (!lora_init()) {
            Serial.println("LoRa init failed.");
        }
        else {
            Serial.println("LoRa init successful.");
        }
    }
    else {
        Serial.println("LoRa Logging Disabled by config.");
    }

    if (SD_LOGGING_ENABLED) {
        if (SD_Logger_Init()) {
            if (SD_Logger_CreateNewLog()) {
                SD_Logger_WriteHeader();
            }
        } else {
            // If init fails, the logger functions have checks for a valid file handle,
            // so the logging task will safely do nothing.
            Serial.println("SD Card Init Failed. Logging will be disabled.");
        }
    }
    else {
        Serial.println("SD Logging Disabled by config.");
    }

    LoadPIDTuningsFromConfig();

    xTaskCreatePinnedToCore(
        TaskIMURead,
        "IMU_Task",
        IMU_TASK_STACK_SIZE,
        NULL,
        IMU_TASK_PRIORITY,
        NULL,
        IMU_TASK_CORE
    );

    xTaskCreatePinnedToCore(
        TaskBarometerRead,
        "Baro_Task",
        BARO_TASK_STACK_SIZE,
        NULL,
        BARO_TASK_PRIORITY,
        NULL,
        BARO_TASK_CORE
    );

    xTaskCreatePinnedToCore(
        TaskAirspeedRead,
        "Airspeed_Task",
        AIRSPEED_TASK_STACK_SIZE,
        NULL,
        AIRSPEED_TASK_PRIORITY,
        NULL,
        AIRSPEED_TASK_CORE
    );

    xTaskCreatePinnedToCore(
        TaskGPSRead,            // Function to implement the task
        "GPS_Task",             // Name of the task
        GPS_TASK_STACK_SIZE,    // Stack size in words
        NULL,                   // Task input parameter
        GPS_TASK_PRIORITY,      // Priority
        NULL,                   // Task handle
        GPS_TASK_CORE           // Pin to Core 1
    );

    xTaskCreatePinnedToCore(
        TaskFlightControl,
        "FlightCtrl_Task",
        FLIGHT_CONTROL_TASK_STACK_SIZE,
        NULL,
        FLIGHT_CONTROL_TASK_PRIORITY,
        NULL,
        FLIGHT_CONTROL_TASK_CORE
    );

    xTaskCreatePinnedToCore(
        TaskLoRaRx,
        "LoRa_RX_Task",
        LORA_RX_TASK_STACK_SIZE,
        NULL,
        LORA_RX_TASK_PRIORITY,
        NULL,
        LORA_RX_TASK_CORE
    );

    xTaskCreatePinnedToCore(
        TaskTelemetryTx,
        "Telemetry_Task",
        TELEMETRY_TASK_STACK_SIZE,
        NULL,
        TELEMETRY_TASK_PRIORITY,
        NULL,
        TELEMETRY_TASK_CORE
    );

    if (SD_LOGGING_ENABLED) {
        xTaskCreatePinnedToCore(
            TaskSDLog,
            "SD_Log_Task",
            SD_LOG_TASK_STACK_SIZE,
            NULL,
            SD_LOG_TASK_PRIORITY,
            NULL,
            SD_LOG_TASK_CORE
        );
    }

    xTaskCreatePinnedToCore(
        TaskStatusLED,
        "Status_LED_Task",
        STATUS_LED_TASK_STACK_SIZE,
        NULL,
        STATUS_LED_TASK_PRIORITY,
        NULL,
        STATUS_LED_TASK_CORE
    );
}

void loop() {
    // Core 0 handles the default loop(). We will just use it to print data.

    if (IMU_DEBUG_OUTPUT_ENABLED) {
        if (currentIMU.healthy) {
            Serial.printf("Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f\n", currentIMU.roll, currentIMU.pitch, currentIMU.yaw);
        }
    }

    if (AIRSPEED_DEBUG_OUTPUT_ENABLED) {
        if (airspeed_data.healthy) {
            Serial.printf("Airspeed: %6.2f m/s | Pressure: %6.2f Pa\n", airspeed_data.airspeed_mps, airspeed_data.pressure_pa);
        }
    }

    if (BARO_DEBUG_OUTPUT_ENABLED) {
        if (baro_data.healthy) {
            Serial.printf("Baro Altitude: %6.2f m | Pressure: %7.2f hPa\n", baro_data.altitude, baro_data.pressure);
        }
    }

    if (GPS_DEBUG_OUTPUT_ENABLED) {
        if (gps_data.healthy) {
            Serial.printf("GPS Lat: %f | Lon: %f | Alt: %f | Speed: %f | Heading: %f | Satellites: %d\n", gps_data.latitude, gps_data.longitude, gps_data.altitude, gps_data.speed, gps_data.heading, gps_data.satellites);
        } else {
            Serial.println("GPS reading unhealthy");
        }
    }

    if(RX_DEBUG_OUTPUT_ENABLED) {
        Serial.printf("RC Data - Throttle: %u | Elevator: %u | Aileron: %u | Rudder: %u | FlightMode PWM: %u (Mode: %d)\n",
                      rc_data.throttle_pwm, rc_data.elevator_pwm, rc_data.aileron_pwm, rc_data.rudder_pwm, rc_data.flightmode_pwm, (int)DetermineFlightMode());
    }

    delay(50); 
}
