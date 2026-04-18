#include "flight/pid_tuning.h"

#include <Arduino.h>
#include <ArduinoJson.h>

#include <math.h>
#include <string.h>

#include "config.h"
#include "hal/comms/lora.h"
#include "math/pid.h"

extern PIDController roll_pid;
extern PIDController pitch_pid;
extern PIDController yaw_pid;
extern PIDController altitude_pid;
extern PIDController headingerror_pid;

namespace {

struct PIDAxisConfig {
    float kp;
    float ki;
    float kd;
    float max_output;
    float max_integral;
};

struct PIDRuntimeConfig {
    uint32_t version;
    PIDAxisConfig roll;
    PIDAxisConfig pitch;
    PIDAxisConfig yaw;
    PIDAxisConfig altitude;
    PIDAxisConfig headingerror;
};

PIDRuntimeConfig g_pid_config = {};
bool g_pid_config_initialized = false;
volatile uint32_t g_telemetry_holdoff_until_ms = 0;
uint32_t g_last_non_json_rx_log_ms = 0;

bool telemetry_holdoff_active(uint32_t now_ms) {
    return static_cast<int32_t>(g_telemetry_holdoff_until_ms - now_ms) > 0;
}

void refresh_telemetry_holdoff_window() {
    if (PID_TUNING_TELEMETRY_HOLDOFF_MS == 0UL) {
        return;
    }

    g_telemetry_holdoff_until_ms = millis() + PID_TUNING_TELEMETRY_HOLDOFF_MS;
}

bool payload_looks_like_json(const uint8_t *data, size_t length) {
    if (data == nullptr || length == 0) {
        return false;
    }

    size_t first = 0;
    while (first < length && (data[first] == ' ' || data[first] == '\t' ||
                              data[first] == '\r' || data[first] == '\n')) {
        ++first;
    }

    if (first >= length || data[first] != '{') {
        return false;
    }

    size_t last = length;
    while (last > first && (data[last - 1] == ' ' || data[last - 1] == '\t' ||
                            data[last - 1] == '\r' || data[last - 1] == '\n')) {
        --last;
    }

    return last > first && data[last - 1] == '}';
}

void send_tuning_ack(const char *status,
                     const char *reason,
                     uint32_t version,
                     bool changed,
                     uint32_t cmd_id) {
    char ack_payload[PID_TUNING_LORA_MAX_PAYLOAD_BYTES + 1] = {};
    const int ack_len = snprintf(
        ack_payload,
        sizeof(ack_payload),
        "{\"type\":\"pid_tuning_ack\",\"status\":\"%s\",\"reason\":\"%s\",\"version\":%lu,\"changed\":%s,\"cmd_id\":%lu}",
        status,
        reason,
        static_cast<unsigned long>(version),
        changed ? "true" : "false",
        static_cast<unsigned long>(cmd_id));

    if (ack_len <= 0 ||
        static_cast<size_t>(ack_len) > PID_TUNING_LORA_MAX_PAYLOAD_BYTES) {
        return;
    }

    (void)lora_send(reinterpret_cast<const uint8_t *>(ack_payload),
                    static_cast<size_t>(ack_len));
}

bool is_numeric_field(JsonVariantConst value) {
    return value.is<float>() || value.is<double>() || value.is<int>() ||
           value.is<long>() || value.is<unsigned int>() ||
           value.is<unsigned long>();
}

bool parse_float_field(JsonObjectConst object,
                       const char *key,
                       float &destination,
                       bool &changed) {
    JsonVariantConst value = object[key];
    if (value.isNull()) {
        return false;
    }

    if (!is_numeric_field(value)) {
        return false;
    }

    const float parsed = value.as<float>();
    if (!isfinite(parsed)) {
        return false;
    }

    if (destination != parsed) {
        destination = parsed;
        changed = true;
    }

    return true;
}

bool axis_config_is_valid(const PIDAxisConfig &axis) {
    const bool gains_valid =
        isfinite(axis.kp) && isfinite(axis.ki) && isfinite(axis.kd) &&
        axis.kp >= -1000.0f && axis.kp <= 1000.0f &&
        axis.ki >= -1000.0f && axis.ki <= 1000.0f &&
        axis.kd >= -1000.0f && axis.kd <= 1000.0f;

    const bool limits_valid =
        isfinite(axis.max_output) && isfinite(axis.max_integral) &&
        axis.max_output > 0.0f && axis.max_output <= 5000.0f &&
        axis.max_integral >= 0.0f && axis.max_integral <= 5000.0f;

    return gains_valid && limits_valid;
}

bool runtime_config_is_valid(const PIDRuntimeConfig &config) {
    return axis_config_is_valid(config.roll) &&
           axis_config_is_valid(config.pitch) &&
           axis_config_is_valid(config.yaw) &&
           axis_config_is_valid(config.altitude) &&
           axis_config_is_valid(config.headingerror);
}

void load_default_config(PIDRuntimeConfig &config) {
    config.version = 1;
    config.roll = {roll_kp, roll_ki, roll_kd, max_roll_output, max_roll_integral};
    config.pitch =
        {pitch_kp, pitch_ki, pitch_kd, max_pitch_output, max_pitch_integral};
    config.yaw = {yaw_kp, yaw_ki, yaw_kd, max_yaw_output, max_yaw_integral};
    config.altitude = {alt_kp, alt_ki, alt_kd, max_alt_output, max_alt_integral};
    config.headingerror = {headingerror_kp,
                           headingerror_ki,
                           headingerror_kd,
                           max_headingerror_output,
                           max_headingerror_integral};
}

void apply_axis_config(PIDController &controller,
                       const PIDAxisConfig &axis,
                       bool reset_integral_state) {
    controller.setTunings(axis.kp, axis.ki, axis.kd);
    controller.setLimits(axis.max_output, axis.max_integral);

    if (reset_integral_state) {
        controller.PIDreset();
    }
}

void apply_runtime_config(const PIDRuntimeConfig &config, bool reset_integral_state) {
    apply_axis_config(roll_pid, config.roll, reset_integral_state);
    apply_axis_config(pitch_pid, config.pitch, reset_integral_state);
    apply_axis_config(yaw_pid, config.yaw, reset_integral_state);
    apply_axis_config(altitude_pid, config.altitude, reset_integral_state);
    apply_axis_config(headingerror_pid, config.headingerror, reset_integral_state);
}

bool update_axis_from_object(JsonObjectConst object,
                             PIDAxisConfig &axis,
                             bool &changed) {
    bool has_supported_field = false;

    has_supported_field |= parse_float_field(object, "kp", axis.kp, changed);
    has_supported_field |= parse_float_field(object, "ki", axis.ki, changed);
    has_supported_field |= parse_float_field(object, "kd", axis.kd, changed);
    has_supported_field |=
        parse_float_field(object, "max_output", axis.max_output, changed);
    has_supported_field |=
        parse_float_field(object, "max_integral", axis.max_integral, changed);

    return has_supported_field;
}

PIDAxisConfig *axis_config_by_name(PIDRuntimeConfig &config, const char *axis_name) {
    if (axis_name == nullptr) {
        return nullptr;
    }

    if (strcmp(axis_name, "roll") == 0) {
        return &config.roll;
    }
    if (strcmp(axis_name, "pitch") == 0) {
        return &config.pitch;
    }
    if (strcmp(axis_name, "yaw") == 0) {
        return &config.yaw;
    }
    if (strcmp(axis_name, "altitude") == 0 || strcmp(axis_name, "alt") == 0) {
        return &config.altitude;
    }
    if (strcmp(axis_name, "headingerror") == 0 ||
        strcmp(axis_name, "heading_error") == 0 ||
        strcmp(axis_name, "heading") == 0) {
        return &config.headingerror;
    }

    return nullptr;
}

bool merge_axis_tree(JsonObjectConst object,
                     PIDRuntimeConfig &config,
                     bool &changed) {
    bool recognized = false;

    if (object.containsKey("roll") && object["roll"].is<JsonObjectConst>()) {
        recognized = true;
        (void)update_axis_from_object(object["roll"].as<JsonObjectConst>(),
                                      config.roll,
                                      changed);
    }

    if (object.containsKey("pitch") && object["pitch"].is<JsonObjectConst>()) {
        recognized = true;
        (void)update_axis_from_object(object["pitch"].as<JsonObjectConst>(),
                                      config.pitch,
                                      changed);
    }

    if (object.containsKey("yaw") && object["yaw"].is<JsonObjectConst>()) {
        recognized = true;
        (void)update_axis_from_object(
            object["yaw"].as<JsonObjectConst>(), config.yaw, changed);
    }

    if (object.containsKey("altitude") &&
        object["altitude"].is<JsonObjectConst>()) {
        recognized = true;
        (void)update_axis_from_object(object["altitude"].as<JsonObjectConst>(),
                                      config.altitude,
                                      changed);
    }

    if (object.containsKey("headingerror") &&
        object["headingerror"].is<JsonObjectConst>()) {
        recognized = true;
        (void)update_axis_from_object(
            object["headingerror"].as<JsonObjectConst>(),
            config.headingerror,
            changed);
    }

    return recognized;
}

bool merge_single_axis_update(JsonObjectConst root,
                              PIDRuntimeConfig &config,
                              bool &changed) {
    const char *axis_name = root["axis"] | "";
    PIDAxisConfig *axis = axis_config_by_name(config, axis_name);
    if (axis == nullptr) {
        return false;
    }

    return update_axis_from_object(root, *axis, changed);
}

bool merge_json_into_config(JsonObjectConst root,
                            PIDRuntimeConfig &config,
                            bool &changed) {
    bool recognized_any_field = false;

    if (root.containsKey("version") &&
        (root["version"].is<uint32_t>() || root["version"].is<unsigned long>() ||
         root["version"].is<unsigned int>() || root["version"].is<int>())) {
        recognized_any_field = true;
        config.version = root["version"].as<uint32_t>();
    }

    if (root.containsKey("pid") && root["pid"].is<JsonObjectConst>()) {
        recognized_any_field = true;
        (void)merge_axis_tree(
            root["pid"].as<JsonObjectConst>(), config, changed);
    }

    if (merge_axis_tree(root, config, changed)) {
        recognized_any_field = true;
    }

    if (root.containsKey("axis")) {
        recognized_any_field = true;
        (void)merge_single_axis_update(root, config, changed);
    }

    return recognized_any_field;
}

} // namespace

void PIDTuning_Init() {
    if (g_pid_config_initialized) {
        return;
    }

    load_default_config(g_pid_config);
    apply_runtime_config(g_pid_config, false);
    g_pid_config_initialized = true;
}

bool PIDTuning_ApplyLoRaUpdate(const uint8_t *data, size_t length) {
    if (!PID_TUNING_ENABLED || data == nullptr || length == 0 ||
        length > PID_TUNING_LORA_MAX_PAYLOAD_BYTES) {
        return false;
    }

    if (!g_pid_config_initialized) {
        PIDTuning_Init();
    }

    if (!payload_looks_like_json(data, length)) {
        // Ignore non-JSON packets (such as unrelated radio traffic).
        const uint32_t now_ms = millis();
        if ((now_ms - g_last_non_json_rx_log_ms) > 1000UL) {
            g_last_non_json_rx_log_ms = now_ms;
            Serial.printf(
                "PID tuning: ignored non-JSON LoRa payload (len=%u, first=0x%02X 0x%02X).\n",
                static_cast<unsigned>(length),
                static_cast<unsigned>(length > 0 ? data[0] : 0U),
                static_cast<unsigned>(length > 1 ? data[1] : 0U));
        }
        return false;
    }

    // Give LoRa command/ACK exchange temporary priority over telemetry TX.
    refresh_telemetry_holdoff_window();

    char payload[PID_TUNING_LORA_MAX_PAYLOAD_BYTES + 1];
    memcpy(payload, data, length);
    payload[length] = '\0';

    Serial.printf("PID tuning: LoRa command RX (%u bytes): %s\n",
                  static_cast<unsigned>(length),
                  payload);

    DynamicJsonDocument doc(PID_TUNING_JSON_DOC_CAPACITY);
    DeserializationError error = deserializeJson(doc, payload);
    if (error || !doc.is<JsonObjectConst>()) {
        Serial.printf("PID tuning: LoRa JSON parse failed: %s\n", error.c_str());
        send_tuning_ack("nack", "json_parse", g_pid_config.version, false, 0);
        return false;
    }

    uint32_t cmd_id = 0;
    JsonObjectConst root = doc.as<JsonObjectConst>();
    JsonVariantConst cmd_id_value = root["cmd_id"];
    if (!cmd_id_value.isNull() && is_numeric_field(cmd_id_value)) {
        const long parsed_cmd_id = cmd_id_value.as<long>();
        if (parsed_cmd_id > 0) {
            cmd_id = static_cast<uint32_t>(parsed_cmd_id);
        }
    }

    PIDRuntimeConfig next = g_pid_config;
    bool changed = false;
    if (!merge_json_into_config(root, next, changed)) {
        send_tuning_ack(
            "nack", "unknown_fields", g_pid_config.version, false, cmd_id);
        return false;
    }

    if (!changed) {
        send_tuning_ack("ack", "no_change", g_pid_config.version, false, cmd_id);
        return false;
    }

    if (!runtime_config_is_valid(next)) {
        Serial.println("PID tuning: rejected LoRa update due to invalid limits.");
        send_tuning_ack(
            "nack", "invalid_limits", g_pid_config.version, false, cmd_id);
        return false;
    }

    next.version = g_pid_config.version + 1;
    g_pid_config = next;
    apply_runtime_config(g_pid_config, true);

    Serial.printf("PID tuning: LoRa update applied (version=%lu).\n",
                  static_cast<unsigned long>(g_pid_config.version));

    send_tuning_ack("ack", "applied", g_pid_config.version, true, cmd_id);
    return true;
}

bool PIDTuning_ShouldHoldTelemetryTx() {
    if (!PID_TUNING_ENABLED || PID_TUNING_TELEMETRY_HOLDOFF_MS == 0UL) {
        return false;
    }

    return telemetry_holdoff_active(millis());
}
