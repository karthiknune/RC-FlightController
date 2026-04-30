#include "hal/sensors/imu.h"

#include <cmath>

#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

#include "config.h"
#include "hal/sensors/sensor_bus.h"
#include "math/utils.h"

Adafruit_ICM20948 icm;

namespace
{

    constexpr uint8_t kIMUAddress = ICM20948_I2CADDR_DEFAULT;
    constexpr float kRadToDeg = 57.2957795f;
    constexpr float kDegToRad = PI / 180.0f;
    constexpr float kMinMagAxisSpanUt = 25.0f;

    uint32_t g_last_init_attempt_ms = 0;
    bool g_imu_ready = false;
    bool g_imu_missing_logged = false;
    bool g_orientation_seeded = false;
    uint32_t g_last_time_us = 0;
    float g_roll_deg = 0.0f;
    float g_pitch_deg = 0.0f;
    float g_yaw_deg = 0.0f;
    float g_gyro_bias_x_dps = 0.0f;
    float g_gyro_bias_y_dps = 0.0f;
    float g_gyro_bias_z_dps = 0.0f;
    float g_yaw_fusion_bias_dps = 0.0f;
    bool g_mag_heading_seeded = false;
    float g_mag_heading_cos = 1.0f;
    float g_mag_heading_sin = 0.0f;

    struct Vec3
    {
        float x;
        float y;
        float z;
    };

    float WrapDegrees(float angle_deg)
    {
        return math::wrap_heading_error(angle_deg);
    }

    Vec3 MakeVec3(float x, float y, float z)
    {
        return {x, y, z};
    }

    float Dot(const Vec3 &a, const Vec3 &b)
    {
        return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
    }

    Vec3 Cross(const Vec3 &a, const Vec3 &b)
    {
        return {
            (a.y * b.z) - (a.z * b.y),
            (a.z * b.x) - (a.x * b.z),
            (a.x * b.y) - (a.y * b.x)};
    }

    float Norm(const Vec3 &v)
    {
        return sqrtf(Dot(v, v));
    }

    bool Normalize(Vec3 &v)
    {
        const float magnitude = Norm(v);
        if (magnitude <= 1.0e-6f)
        {
            return false;
        }

        v.x /= magnitude;
        v.y /= magnitude;
        v.z /= magnitude;
        return true;
    }

    bool ProbeIMULocked()
    {
        Wire.beginTransmission(kIMUAddress);
        return Wire.endTransmission(true) == 0;
    }

    void ResetOrientationState()
    {
        g_last_time_us = 0;
        g_orientation_seeded = false;
        g_roll_deg = 0.0f;
        g_pitch_deg = 0.0f;
        g_yaw_deg = 0.0f;
        g_yaw_fusion_bias_dps = 0.0f;
        g_mag_heading_seeded = false;
        g_mag_heading_cos = 1.0f;
        g_mag_heading_sin = 0.0f;
    }

    float ApplyBodyFrameAxis(float value, float axis_sign)
    {
        return value * axis_sign;
    }

    float ComputeAccelCorrectionGain(const IMUData_raw &data)
    {
        const float accel_norm = sqrtf((data.accel_x * data.accel_x) +
                                       (data.accel_y * data.accel_y) +
                                       (data.accel_z * data.accel_z));
        const float accel_error = fabsf(accel_norm - IMU_ACCEL_1G_MPS2);

        if (accel_error >= IMU_ACCEL_TRUST_ERROR_MPS2)
        {
            return IMU_ACCEL_MIN_TRUST;
        }

        const float normalized_error = accel_error / IMU_ACCEL_TRUST_ERROR_MPS2;
        const float trust = 1.0f - normalized_error;
        return IMU_ACCEL_MIN_TRUST + ((1.0f - IMU_ACCEL_MIN_TRUST) * trust);
    }

    float ComputeMagCorrectionGain(const IMUData_raw &data, float horizontal_ratio)
    {
        const float mag_norm = sqrtf((data.mag_x * data.mag_x) +
                                     (data.mag_y * data.mag_y) +
                                     (data.mag_z * data.mag_z));
        if (mag_norm < 1.0f)
        {
            return IMU_MAG_MIN_TRUST;
        }

        if (horizontal_ratio <= IMU_MAG_MIN_HORIZONTAL_RATIO)
        {
            return IMU_MAG_MIN_TRUST;
        }

        const float normalized_ratio =
            (horizontal_ratio - IMU_MAG_MIN_HORIZONTAL_RATIO) /
            (1.0f - IMU_MAG_MIN_HORIZONTAL_RATIO);
        const float clamped_ratio = math::clamp_value(normalized_ratio, 0.0f, 1.0f);
        return IMU_MAG_MIN_TRUST + ((1.0f - IMU_MAG_MIN_TRUST) * clamped_ratio);
    }

    void AlignMagnetometerToIMUFrame(const sensors_event_t &m,
                                     float &aligned_x,
                                     float &aligned_y,
                                     float &aligned_z)
    {
        aligned_x = m.magnetic.x * IMU_MAG_SENSOR_ALIGN_X_SIGN;
        aligned_y = m.magnetic.y * IMU_MAG_SENSOR_ALIGN_Y_SIGN;
        aligned_z = m.magnetic.z * IMU_MAG_SENSOR_ALIGN_Z_SIGN;
    }

    void PopulateIMUData(const sensors_event_t &a,
                         const sensors_event_t &g,
                         const sensors_event_t &m,
                         IMUData_raw &data)
    {
        data.accel_x = ApplyBodyFrameAxis(a.acceleration.x, IMU_BODY_FRAME_X_SIGN);
        data.accel_y = ApplyBodyFrameAxis(a.acceleration.y, IMU_BODY_FRAME_Y_SIGN);
        data.accel_z = ApplyBodyFrameAxis(a.acceleration.z, IMU_BODY_FRAME_Z_SIGN);

        const float gyro_x_dps = ApplyBodyFrameAxis(g.gyro.x * kRadToDeg, IMU_BODY_FRAME_X_SIGN);
        const float gyro_y_dps = ApplyBodyFrameAxis(g.gyro.y * kRadToDeg, IMU_BODY_FRAME_Y_SIGN);
        const float gyro_z_dps = ApplyBodyFrameAxis(g.gyro.z * kRadToDeg, IMU_BODY_FRAME_Z_SIGN);

        data.gyro_x = gyro_x_dps - g_gyro_bias_x_dps;
        data.gyro_y = gyro_y_dps - g_gyro_bias_y_dps;
        data.gyro_z = gyro_z_dps - g_gyro_bias_z_dps;

        float aligned_mag_x = 0.0f;
        float aligned_mag_y = 0.0f;
        float aligned_mag_z = 0.0f;
        AlignMagnetometerToIMUFrame(m, aligned_mag_x, aligned_mag_y, aligned_mag_z);

        data.mag_x = ApplyBodyFrameAxis((aligned_mag_x - IMU_MAG_OFFSET_X) * IMU_MAG_SCALE_X, IMU_BODY_FRAME_X_SIGN);
        data.mag_y = ApplyBodyFrameAxis((aligned_mag_y - IMU_MAG_OFFSET_Y) * IMU_MAG_SCALE_Y, IMU_BODY_FRAME_Y_SIGN);
        data.mag_z = ApplyBodyFrameAxis((aligned_mag_z - IMU_MAG_OFFSET_Z) * IMU_MAG_SCALE_Z, IMU_BODY_FRAME_Z_SIGN);
    }

    float ComputeAccelRollDeg(const IMUData_raw &data)
    {
        return (atan2f(data.accel_y, -data.accel_z) * kRadToDeg) - IMU_LEVEL_ROLL_OFFSET_DEG;
    }

    float ComputeAccelPitchDeg(const IMUData_raw &data)
    {
        return (atan2f(data.accel_x,
                       sqrtf((data.accel_y * data.accel_y) + (data.accel_z * data.accel_z))) *
                kRadToDeg) -
               IMU_LEVEL_PITCH_OFFSET_DEG;
    }

    Vec3 ComputeDownVectorFromLeveledAttitude(float roll_deg, float pitch_deg)
    {
        const float roll_rad = roll_deg * kDegToRad;
        const float pitch_rad = pitch_deg * kDegToRad;
        const float sin_roll = sinf(roll_rad);
        const float cos_roll = cosf(roll_rad);
        const float sin_pitch = sinf(pitch_rad);
        const float cos_pitch = cosf(pitch_rad);

        return {
            sin_pitch,
            sin_roll * cos_pitch,
            cos_roll * cos_pitch};
    }

    bool ComputeTiltCompensatedYawDeg(const IMUData_raw &data,
                                      float roll_deg,
                                      float pitch_deg,
                                      float &yaw_deg,
                                      float &horizontal_ratio)
    {
        const Vec3 mag_body = {data.mag_x, data.mag_y, data.mag_z};
        const float mag_norm = Norm(mag_body);
        if (mag_norm < 1.0f)
        {
            horizontal_ratio = 0.0f;
            return false;
        }

        Vec3 down_body = ComputeDownVectorFromLeveledAttitude(roll_deg, pitch_deg);
        if (!Normalize(down_body))
        {
            horizontal_ratio = 0.0f;
            return false;
        }

        Vec3 horizontal_north_body = {
            mag_body.x - (down_body.x * Dot(mag_body, down_body)),
            mag_body.y - (down_body.y * Dot(mag_body, down_body)),
            mag_body.z - (down_body.z * Dot(mag_body, down_body))};

        const float horizontal_norm = Norm(horizontal_north_body);
        horizontal_ratio = horizontal_norm / mag_norm;
        if (horizontal_norm <= 1.0e-6f || horizontal_ratio <= IMU_MAG_MIN_HORIZONTAL_RATIO)
        {
            return false;
        }

        if (!Normalize(horizontal_north_body))
        {
            return false;
        }

        Vec3 horizontal_right_body = Cross(horizontal_north_body, down_body);
        if (!Normalize(horizontal_right_body))
        {
            return false;
        }

        // Heading is the body-forward axis projected onto the local north/right plane.
        const float forward_on_north = horizontal_north_body.x;
        const float forward_on_right = horizontal_right_body.x;
        yaw_deg = atan2f(forward_on_right, forward_on_north) * kRadToDeg;
        yaw_deg = WrapDegrees(yaw_deg);
        return true;
    }

    float LowPassHeadingDeg(float measured_heading_deg, float dt, float measurement_gain)
    {
        const float base_alpha = IMU_MAG_HEADING_LPF_TIME_CONSTANT /
                                 (IMU_MAG_HEADING_LPF_TIME_CONSTANT + dt);
        const float blend = (1.0f - base_alpha) * measurement_gain;
        const float measured_cos = cosf(measured_heading_deg * kDegToRad);
        const float measured_sin = sinf(measured_heading_deg * kDegToRad);

        if (!g_mag_heading_seeded)
        {
            g_mag_heading_cos = measured_cos;
            g_mag_heading_sin = measured_sin;
            g_mag_heading_seeded = true;
        }
        else
        {
            g_mag_heading_cos = ((1.0f - blend) * g_mag_heading_cos) + (blend * measured_cos);
            g_mag_heading_sin = ((1.0f - blend) * g_mag_heading_sin) + (blend * measured_sin);

            const float mag = sqrtf((g_mag_heading_cos * g_mag_heading_cos) +
                                    (g_mag_heading_sin * g_mag_heading_sin));
            if (mag > 1.0e-6f)
            {
                g_mag_heading_cos /= mag;
                g_mag_heading_sin /= mag;
            }
        }

        return atan2f(g_mag_heading_sin, g_mag_heading_cos) * kRadToDeg;
    }

    void UpdateOrientation(IMUData_raw &data)
    {
        const uint32_t now_us = micros();
        float dt = 0.0f;
        if (g_last_time_us != 0)
        {
            dt = static_cast<float>(now_us - g_last_time_us) / 1000000.0f;
        }
        g_last_time_us = now_us;

        const float accel_roll = ComputeAccelRollDeg(data);
        const float accel_pitch = ComputeAccelPitchDeg(data);

        float measured_mag_yaw = 0.0f;
        float horizontal_ratio = 0.0f;

        if (!g_orientation_seeded)
        {
            g_roll_deg = accel_roll;
            g_pitch_deg = accel_pitch;
            if (ComputeTiltCompensatedYawDeg(data, g_roll_deg, g_pitch_deg, measured_mag_yaw, horizontal_ratio))
            {
                g_yaw_deg = measured_mag_yaw;
                g_mag_heading_cos = cosf(measured_mag_yaw * kDegToRad);
                g_mag_heading_sin = sinf(measured_mag_yaw * kDegToRad);
                g_mag_heading_seeded = true;
            }
            else
            {
                g_yaw_deg = 0.0f;
            }

            g_orientation_seeded = true;
            data.roll = g_roll_deg;
            data.pitch = g_pitch_deg;
            data.yaw = g_yaw_deg;
            return;
        }

        // Skip integration if loop time is invalid (prevents large jumps during blocking tasks)
        if (dt <= 0.0f || dt > 0.5f)
        {
            data.roll = g_roll_deg;
            data.pitch = g_pitch_deg;
            data.yaw = g_yaw_deg;
            return;
        }

        // Convert current Euler angles to radians
        const float phi = g_roll_deg * kDegToRad;
        const float theta = g_pitch_deg * kDegToRad;
        const float sin_phi = sinf(phi);
        const float cos_phi = cosf(phi);
        const float cos_theta = cosf(theta);
        const float tan_theta = tanf(theta);

        // Prevent singularity at 90 degree pitch (gimbal lock)
        float safe_cos_theta = cos_theta;
        if (fabsf(safe_cos_theta) < 0.01f)
        {
            safe_cos_theta = (safe_cos_theta < 0.0f) ? -0.01f : 0.01f;
        }

        // Full non-linear Euler kinematics from body rates in the FRD aircraft frame.
        const float phi_dot = data.gyro_x + tan_theta * (data.gyro_y * sin_phi + data.gyro_z * cos_phi);
        const float theta_dot = data.gyro_y * cos_phi - data.gyro_z * sin_phi;
        const float psi_dot = (data.gyro_y * sin_phi + data.gyro_z * cos_phi) / safe_cos_theta;

        // Predict roll/pitch/yaw from gyro rates first.
        const float gyro_roll = g_roll_deg + (phi_dot * dt);
        const float gyro_pitch = g_pitch_deg + (theta_dot * dt);
        const float gyro_yaw = WrapDegrees(g_yaw_deg + ((psi_dot - g_yaw_fusion_bias_dps) * dt));

        // Correct roll/pitch back toward gravity using measured specific force when it
        // still looks close to 1 g. Offsets are already applied in the measurement.
        const float accel_correction_gain = ComputeAccelCorrectionGain(data);
        const float base_alpha_xy = IMU_FILTER_TIME_CONSTANT_XY / (IMU_FILTER_TIME_CONSTANT_XY + dt);
        const float accel_blend = (1.0f - base_alpha_xy) * accel_correction_gain;

        g_roll_deg = gyro_roll + (accel_blend * WrapDegrees(accel_roll - gyro_roll));
        g_pitch_deg = gyro_pitch + (accel_blend * (accel_pitch - gyro_pitch));
        g_yaw_deg = gyro_yaw;

        // Tilt-compensate magnetometer using the fused roll/pitch estimate. This keeps
        // yaw stable even while the aircraft is banked because the mag vector is first
        // projected onto the level horizontal plane instead of relying on a fragile
        // direct trig heading expression.
        if (ComputeTiltCompensatedYawDeg(data, g_roll_deg, g_pitch_deg, measured_mag_yaw, horizontal_ratio))
        {
            const float mag_correction_gain = ComputeMagCorrectionGain(data, horizontal_ratio);
            const float filtered_mag_yaw = LowPassHeadingDeg(measured_mag_yaw, dt, mag_correction_gain);
            const float base_alpha_yaw = IMU_FILTER_TIME_CONSTANT_Z / (IMU_FILTER_TIME_CONSTANT_Z + dt);
            const float yaw_blend = (1.0f - base_alpha_yaw) * mag_correction_gain;
            const float yaw_error = WrapDegrees(filtered_mag_yaw - g_yaw_deg);

            g_yaw_deg = WrapDegrees(g_yaw_deg + (yaw_blend * yaw_error));
            g_yaw_fusion_bias_dps -= IMU_YAW_BIAS_CORRECTION_GAIN * yaw_error * dt * mag_correction_gain;
            g_yaw_fusion_bias_dps = math::clamp_value(g_yaw_fusion_bias_dps,
                                                      -IMU_YAW_MAX_GYRO_BIAS_DPS,
                                                      IMU_YAW_MAX_GYRO_BIAS_DPS);
        }

        data.roll = g_roll_deg;
        data.pitch = g_pitch_deg;
        data.yaw = g_yaw_deg;
    }

    void UpdateIMUAvailability(bool available)
    {
        if (available)
        {
            if (!g_imu_ready && SENSOR_STATUS_LOGGING_ENABLED)
            {
                Serial.println("ICM-20948 available.");
            }

            g_imu_ready = true;
            g_imu_missing_logged = false;
            return;
        }

        if ((g_imu_ready || !g_imu_missing_logged) && SENSOR_STATUS_LOGGING_ENABLED)
        {
            Serial.println("ICM-20948 unavailable. Continuing without IMU data.");
        }

        g_imu_ready = false;
        g_imu_missing_logged = true;
        ResetOrientationState();
    }

    bool ReadIMUEvents(sensors_event_t &a,
                       sensors_event_t &g,
                       sensors_event_t &temp,
                       sensors_event_t &m)
    {
        if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS)))
        {
            return false;
        }

        if (!ProbeIMULocked())
        {
            SensorBus_Unlock();
            UpdateIMUAvailability(false);
            return false;
        }

        icm.getEvent(&a, &g, &temp, &m);
        SensorBus_Unlock();
        return true;
    }

    bool TryInitializeIMU()
    {
        const uint32_t now_ms = millis();
        if (g_last_init_attempt_ms != 0 &&
            (now_ms - g_last_init_attempt_ms) < SENSOR_RECONNECT_INTERVAL_MS)
        {
            return g_imu_ready;
        }

        g_last_init_attempt_ms = now_ms;

        if (!SensorBus_Init())
        {
            UpdateIMUAvailability(false);
            return false;
        }

        bool begin_ok = false;
        if (SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS)))
        {
            begin_ok = icm.begin_I2C(kIMUAddress, &Wire);
            if (begin_ok)
            {
                icm.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
                icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
                icm.setMagDataRate(AK09916_MAG_DATARATE_50_HZ);
            }
            SensorBus_Unlock();
        }

        if (!begin_ok)
        {
            UpdateIMUAvailability(false);
            return false;
        }

        ResetOrientationState();
        UpdateIMUAvailability(true);
        return true;
    }

} // namespace

void IMU_Init()
{
    (void)TryInitializeIMU();
}

bool IMU_Calibrate_Gyro()
{
    if (!g_imu_ready && !TryInitializeIMU())
    {
        return false;
    }

    Serial.println("Calibrating gyroscope... Keep the aircraft still.");

    double sum_gx = 0.0;
    double sum_gy = 0.0;
    double sum_gz = 0.0;
    int captured_samples = 0;

    for (int sample_index = 0; sample_index < IMU_GYRO_CALIBRATION_SAMPLES; ++sample_index)
    {
        sensors_event_t a = {};
        sensors_event_t g = {};
        sensors_event_t temp = {};
        sensors_event_t m = {};

        if (!ReadIMUEvents(a, g, temp, m))
        {
            delay(IMU_GYRO_CALIBRATION_SAMPLE_DELAY_MS);
            continue;
        }

        sum_gx += static_cast<double>(ApplyBodyFrameAxis(g.gyro.x * kRadToDeg, IMU_BODY_FRAME_X_SIGN));
        sum_gy += static_cast<double>(ApplyBodyFrameAxis(g.gyro.y * kRadToDeg, IMU_BODY_FRAME_Y_SIGN));
        sum_gz += static_cast<double>(ApplyBodyFrameAxis(g.gyro.z * kRadToDeg, IMU_BODY_FRAME_Z_SIGN));
        ++captured_samples;

        delay(IMU_GYRO_CALIBRATION_SAMPLE_DELAY_MS);
    }

    if (captured_samples == 0)
    {
        Serial.println("Gyro calibration failed: no valid samples captured.");
        return false;
    }

    g_gyro_bias_x_dps = static_cast<float>(sum_gx / captured_samples);
    g_gyro_bias_y_dps = static_cast<float>(sum_gy / captured_samples);
    g_gyro_bias_z_dps = static_cast<float>(sum_gz / captured_samples);
    ResetOrientationState();

    Serial.printf(
        "Gyro bias [deg/s] -> X: %.2f | Y: %.2f | Z: %.2f\n",
        g_gyro_bias_x_dps,
        g_gyro_bias_y_dps,
        g_gyro_bias_z_dps);

    return true;
}

bool IMU_Run_Level_Calibration(float &roll_offset_deg, float &pitch_offset_deg)
{
    roll_offset_deg = 0.0f;
    pitch_offset_deg = 0.0f;

    if (!g_imu_ready && !TryInitializeIMU())
    {
        return false;
    }

    Serial.println();
    Serial.println("========================================");
    Serial.println("   AIRCRAFT LEVEL CALIBRATION MODE      ");
    Serial.println("========================================");
    Serial.println("Hold the aircraft steady in level flight attitude.");

    for (int seconds_remaining = 5; seconds_remaining > 0; --seconds_remaining)
    {
        Serial.printf("Starting in %d...\n", seconds_remaining);
        delay(1000);
    }

    double sum_roll = 0.0;
    double sum_pitch = 0.0;
    int captured_samples = 0;

    Serial.println("Recording samples...");
    for (int sample_index = 0; sample_index < IMU_LEVEL_CALIBRATION_SAMPLES; ++sample_index)
    {
        IMUData_raw sample = {};
        IMU_Read(sample);
        if (!sample.healthy)
        {
            delay(IMU_LEVEL_CALIBRATION_SAMPLE_DELAY_MS);
            continue;
        }

        sum_roll += static_cast<double>(g_roll_deg);
        sum_pitch += static_cast<double>(g_pitch_deg);
        ++captured_samples;

        if ((sample_index + 1) % 50 == 0)
        {
            Serial.print("#");
        }

        delay(IMU_LEVEL_CALIBRATION_SAMPLE_DELAY_MS);
    }

    if (captured_samples == 0)
    {
        Serial.println();
        Serial.println("Level calibration failed: no valid IMU samples captured.");
        return false;
    }

    roll_offset_deg = static_cast<float>(sum_roll / captured_samples);
    pitch_offset_deg = static_cast<float>(sum_pitch / captured_samples);

    Serial.println();
    Serial.println();
    Serial.println("--- CALIBRATION RESULTS ---");
    Serial.printf("Average Roll Error : %6.2f degrees\n", roll_offset_deg);
    Serial.printf("Average Pitch Error: %6.2f degrees\n", pitch_offset_deg);
    Serial.println("---------------------------");
    Serial.println("Copy these values into include/config.h:");
    Serial.printf("constexpr float IMU_LEVEL_ROLL_OFFSET_DEG = %6.2ff;\n", roll_offset_deg);
    Serial.printf("constexpr float IMU_LEVEL_PITCH_OFFSET_DEG = %6.2ff;\n", pitch_offset_deg);
    Serial.println("This affects the production IMU estimator used by IMU_Read() in src/hal/sensors/imu.cpp");
    Serial.println("and printed by test/main.cpp when IMU_DEBUG_OUTPUT_ENABLED is true.");
    Serial.println("========================================");
    Serial.println();

    return true;
}

bool IMU_Run_Mag_Calibration(float &offset_x, float &offset_y, float &offset_z,
                             float &scale_x, float &scale_y, float &scale_z)
{
    offset_x = 0.0f;
    offset_y = 0.0f;
    offset_z = 0.0f;
    scale_x = 1.0f;
    scale_y = 1.0f;
    scale_z = 1.0f;

    if (!g_imu_ready && !TryInitializeIMU())
    {
        return false;
    }

    Serial.println();
    Serial.println("========================================");
    Serial.println("   MAGNETOMETER CALIBRATION MODE        ");
    Serial.println("========================================");
    Serial.println("Rotate and tumble the aircraft in ALL directions (figure 8).");
    Serial.println("Try to expose every axis to both + and - directions.");

    for (int seconds_remaining = 5; seconds_remaining > 0; --seconds_remaining)
    {
        Serial.printf("Starting in %d...\n", seconds_remaining);
        delay(1000);
    }

    float min_x = 100000.0f, max_x = -100000.0f;
    float min_y = 100000.0f, max_y = -100000.0f;
    float min_z = 100000.0f, max_z = -100000.0f;

    int captured_samples = 0;
    constexpr int kMagCalibrationSamples = 1500;
    constexpr int kMagSampleDelayMs = 20;

    Serial.println("Recording samples... Keep tumbling!");
    for (int sample_index = 0; sample_index < kMagCalibrationSamples; ++sample_index)
    {
        sensors_event_t a = {};
        sensors_event_t g = {};
        sensors_event_t temp = {};
        sensors_event_t m = {};

        if (!ReadIMUEvents(a, g, temp, m))
        {
            delay(kMagSampleDelayMs);
            continue;
        }

        if (m.magnetic.x != 0.0f || m.magnetic.y != 0.0f || m.magnetic.z != 0.0f)
        {
            float aligned_mag_x = 0.0f;
            float aligned_mag_y = 0.0f;
            float aligned_mag_z = 0.0f;
            AlignMagnetometerToIMUFrame(m, aligned_mag_x, aligned_mag_y, aligned_mag_z);

            if (aligned_mag_x < min_x)
                min_x = aligned_mag_x;
            if (aligned_mag_x > max_x)
                max_x = aligned_mag_x;
            if (aligned_mag_y < min_y)
                min_y = aligned_mag_y;
            if (aligned_mag_y > max_y)
                max_y = aligned_mag_y;
            if (aligned_mag_z < min_z)
                min_z = aligned_mag_z;
            if (aligned_mag_z > max_z)
                max_z = aligned_mag_z;
            ++captured_samples;
        }

        if ((sample_index + 1) % 50 == 0)
        {
            Serial.print("#");
        }

        delay(kMagSampleDelayMs);
    }

    if (captured_samples == 0)
    {
        Serial.println("\nMag calibration failed: no valid samples captured.");
        return false;
    }

    offset_x = (max_x + min_x) / 2.0f;
    offset_y = (max_y + min_y) / 2.0f;
    offset_z = (max_z + min_z) / 2.0f;

    const float radius_x = (max_x - min_x) / 2.0f;
    const float radius_y = (max_y - min_y) / 2.0f;
    const float radius_z = (max_z - min_z) / 2.0f;
    const float span_x = max_x - min_x;
    const float span_y = max_y - min_y;
    const float span_z = max_z - min_z;
    const float avg_radius = (radius_x + radius_y + radius_z) / 3.0f;

    if (radius_x > 0.0f)
        scale_x = avg_radius / radius_x;
    if (radius_y > 0.0f)
        scale_y = avg_radius / radius_y;
    if (radius_z > 0.0f)
        scale_z = avg_radius / radius_z;

    const bool poor_x_coverage = span_x < kMinMagAxisSpanUt;
    const bool poor_y_coverage = span_y < kMinMagAxisSpanUt;
    const bool poor_z_coverage = span_z < kMinMagAxisSpanUt;

    Serial.println("\n\n--- CALIBRATION RESULTS ---");
    Serial.printf("Samples captured  : %d\n", captured_samples);
    Serial.printf("Axis span [uT]    : X: %6.2f | Y: %6.2f | Z: %6.2f\n", span_x, span_y, span_z);
    Serial.printf("Hard-Iron Offsets : X: %6.2f | Y: %6.2f | Z: %6.2f\n", offset_x, offset_y, offset_z);
    Serial.printf("Soft-Iron Scales  : X: %6.2f | Y: %6.2f | Z: %6.2f\n", scale_x, scale_y, scale_z);
    if (poor_x_coverage || poor_y_coverage || poor_z_coverage)
    {
        Serial.println("WARNING: magnetometer coverage looks weak on at least one axis.");
        Serial.println("Re-run calibration with larger rotations before trusting these values.");
    }
    Serial.println("---------------------------");
    Serial.println("Copy these values into include/config.h:");
    Serial.printf("constexpr float IMU_MAG_OFFSET_X = %6.2ff;\n", offset_x);
    Serial.printf("constexpr float IMU_MAG_OFFSET_Y = %6.2ff;\n", offset_y);
    Serial.printf("constexpr float IMU_MAG_OFFSET_Z = %6.2ff;\n", offset_z);
    Serial.printf("constexpr float IMU_MAG_SCALE_X = %6.3ff;\n", scale_x);
    Serial.printf("constexpr float IMU_MAG_SCALE_Y = %6.3ff;\n", scale_y);
    Serial.printf("constexpr float IMU_MAG_SCALE_Z = %6.3ff;\n", scale_z);
    Serial.println("After copying them into include/config.h, set IMU_RUN_MAG_CALIBRATION back to false.");
    Serial.println("These values feed the production IMU estimator in src/hal/sensors/imu.cpp");
    Serial.println("and therefore the attitude printed in test/main.cpp and used by the flight stack.");
    Serial.println("========================================\n");

    return true;
}

void IMU_Read(IMUData_raw &data)
{
    if (!g_imu_ready && !TryInitializeIMU())
    {
        data.healthy = false;
        return;
    }

    sensors_event_t a = {};
    sensors_event_t g = {};
    sensors_event_t temp = {};
    sensors_event_t m = {};

    if (!ReadIMUEvents(a, g, temp, m))
    {
        data.healthy = false;
        return;
    }

    PopulateIMUData(a, g, m, data);
    UpdateOrientation(data);
    data.healthy = true;
    UpdateIMUAvailability(true);

    // Serial.printf("Mag X: %.2f | Mag Y: %.2f | Mag Z: %.2f\n",
    //           m.magnetic.x, m.magnetic.y, m.magnetic.z);
}
