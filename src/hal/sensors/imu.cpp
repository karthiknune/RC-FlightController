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
    constexpr float kMinMagAxisSpanUt = 25.0f;

    uint32_t g_last_time_us = 0;
    uint32_t g_last_mag_time_us = 0;
    float g_last_mag_x = 0.0f;
    float g_last_mag_y = 0.0f;
    float g_last_mag_z = 0.0f;

    uint32_t g_last_init_attempt_ms = 0;
    bool g_imu_ready = false;
    bool g_imu_missing_logged = false;
    bool g_orientation_seeded = false;
    float g_roll_deg = 0.0f;
    float g_pitch_deg = 0.0f;
    float g_yaw_deg = 0.0f;
    float g_gyro_bias_x_dps = 0.0f;
    float g_gyro_bias_y_dps = 0.0f;
    float g_gyro_bias_z_dps = 0.0f;
    float g_yaw_kalman_bias_dps = 0.0f;
    float g_yaw_kalman_p00 = 1.0f;
    float g_yaw_kalman_p01 = 0.0f;
    float g_yaw_kalman_p10 = 0.0f;
    float g_yaw_kalman_p11 = 1.0f;

    bool ProbeIMULocked()
    {
        Wire.beginTransmission(kIMUAddress);
        return Wire.endTransmission(true) == 0;
    }

    void ResetOrientationState()
    {
        g_last_time_us = 0;
        g_last_mag_time_us = 0;
        g_last_mag_x = 0.0f;
        g_last_mag_y = 0.0f;
        g_last_mag_z = 0.0f;
        g_orientation_seeded = false;
        g_roll_deg = 0.0f;
        g_pitch_deg = 0.0f;
        g_yaw_deg = 0.0f;
        g_yaw_kalman_bias_dps = 0.0f;
        g_yaw_kalman_p00 = 1.0f;
        g_yaw_kalman_p01 = 0.0f;
        g_yaw_kalman_p10 = 0.0f;
        g_yaw_kalman_p11 = 1.0f;
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

    float ComputeMagCorrectionGain(const IMUData_raw &data, float roll_deg, float pitch_deg)
    {
        const float mag_norm = sqrtf((data.mag_x * data.mag_x) +
                                     (data.mag_y * data.mag_y) +
                                     (data.mag_z * data.mag_z));
        if (mag_norm < 1.0f)
        {
            return IMU_MAG_MIN_TRUST;
        }

        const float roll_abs = fabsf(roll_deg);
        const float pitch_abs = fabsf(pitch_deg);
        const float tilt_abs = (roll_abs > pitch_abs) ? roll_abs : pitch_abs;
        if (tilt_abs >= IMU_MAG_MAX_TILT_DEG)
        {
            return IMU_MAG_MIN_TRUST;
        }

        const float tilt_scale = 1.0f - (tilt_abs / IMU_MAG_MAX_TILT_DEG);
        return IMU_MAG_MIN_TRUST + ((1.0f - IMU_MAG_MIN_TRUST) * tilt_scale);
    }

    float UpdateYawKalman(float measured_yaw_deg, float gyro_yaw_rate_dps, float dt, float measurement_gain)
    {
        const float rate = gyro_yaw_rate_dps - g_yaw_kalman_bias_dps;
        g_yaw_deg = math::wrap_heading_error(g_yaw_deg + (dt * rate));

        g_yaw_kalman_p00 += dt * (dt * g_yaw_kalman_p11 - g_yaw_kalman_p01 - g_yaw_kalman_p10 + IMU_YAW_KALMAN_Q_ANGLE);
        g_yaw_kalman_p01 -= dt * g_yaw_kalman_p11;
        g_yaw_kalman_p10 -= dt * g_yaw_kalman_p11;
        g_yaw_kalman_p11 += IMU_YAW_KALMAN_Q_BIAS * dt;

        const float effective_r = IMU_YAW_KALMAN_R_MEASURE / measurement_gain;
        const float innovation = math::wrap_heading_error(measured_yaw_deg - g_yaw_deg);
        const float s = g_yaw_kalman_p00 + effective_r;
        const float k0 = g_yaw_kalman_p00 / s;
        const float k1 = g_yaw_kalman_p10 / s;

        g_yaw_deg = math::wrap_heading_error(g_yaw_deg + (k0 * innovation));
        g_yaw_kalman_bias_dps += k1 * innovation;

        const float p00_temp = g_yaw_kalman_p00;
        const float p01_temp = g_yaw_kalman_p01;

        g_yaw_kalman_p00 -= k0 * p00_temp;
        g_yaw_kalman_p01 -= k0 * p01_temp;
        g_yaw_kalman_p10 -= k1 * p00_temp;
        g_yaw_kalman_p11 -= k1 * p01_temp;

        return g_yaw_deg;
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

    void UpdateOrientation(IMUData_raw &data)
    {
        const uint32_t now_us = micros();
        float dt = 0.0f;
        if (g_last_time_us != 0)
        {
            dt = static_cast<float>(now_us - g_last_time_us) / 1000000.0f;
        }
        g_last_time_us = now_us;

        // Standard FRD (Forward-Right-Down) Euler Angles
        const float accel_roll = atan2f(-data.accel_y, -data.accel_z) * 180.0f / PI;
        const float accel_pitch = atan2f(data.accel_x, sqrtf((data.accel_y * data.accel_y) + (data.accel_z * data.accel_z))) * 180.0f / PI;

        // Convert current Euler angles to radians
        const float phi = g_roll_deg * (PI / 180.0f);
        const float theta = g_pitch_deg * (PI / 180.0f);

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

        // Full Non-Linear Euler Kinematics for Gyro Rates (FRD)
        // Prevents cross-coupling during steep banking and climbing maneuvers.
        const float phi_dot = data.gyro_x + tan_theta * (data.gyro_y * sin_phi + data.gyro_z * cos_phi);
        const float theta_dot = data.gyro_y * cos_phi - data.gyro_z * sin_phi;
        const float psi_dot = (data.gyro_y * sin_phi + data.gyro_z * cos_phi) / safe_cos_theta;

        // Tilt-Compensated Magnetometer Yaw
        const float leveled_roll_rad = (g_roll_deg - IMU_LEVEL_ROLL_OFFSET_DEG) * (PI / 180.0f);
        const float leveled_pitch_rad = (g_pitch_deg - IMU_LEVEL_PITCH_OFFSET_DEG) * (PI / 180.0f);

        const float cr = cosf(leveled_roll_rad);
        const float sr = sinf(leveled_roll_rad);
        const float cp = cosf(leveled_pitch_rad);
        const float sp = sinf(leveled_pitch_rad);

        const float mag_x_h = data.mag_x * cp + data.mag_y * sr * sp + data.mag_z * cr * sp;
        const float mag_y_h = data.mag_y * cr - data.mag_z * sr;

        // Positive yaw should follow aircraft right-turn convention with the current FRD mapping.
        const float mag_heading = atan2f(mag_y_h, mag_x_h) * 180.0f / PI;

        const float epsilon = 0.001f;
        const bool mag_is_fresh = (fabsf(data.mag_x - g_last_mag_x) > epsilon ||
                                   fabsf(data.mag_y - g_last_mag_y) > epsilon ||
                                   fabsf(data.mag_z - g_last_mag_z) > epsilon);

        if (!g_orientation_seeded)
        {
            g_roll_deg = accel_roll;
            g_pitch_deg = accel_pitch;
            g_yaw_deg = mag_heading;
            g_orientation_seeded = true;

            g_last_mag_time_us = now_us;
            g_last_mag_x = data.mag_x;
            g_last_mag_y = data.mag_y;
            g_last_mag_z = data.mag_z;
            g_last_time_us = now_us;

            data.roll = -(g_roll_deg - IMU_LEVEL_ROLL_OFFSET_DEG);
            data.pitch = g_pitch_deg - IMU_LEVEL_PITCH_OFFSET_DEG;
            data.yaw = g_yaw_deg;
            return;
        }

        // Skip integration if loop time is invalid (prevents large jumps during blocking tasks)
        if (dt <= 0.0f || dt > 0.5f)
        {
            g_last_time_us = now_us;
            data.roll = -(g_roll_deg - IMU_LEVEL_ROLL_OFFSET_DEG);
            data.pitch = g_pitch_deg - IMU_LEVEL_PITCH_OFFSET_DEG;
            data.yaw = g_yaw_deg;
            return;
        }

        // Integrate non-linear gyro rates
        const float gyro_roll = g_roll_deg + (phi_dot * dt);
        const float gyro_pitch = g_pitch_deg + (theta_dot * dt);
        const float gyro_yaw = g_yaw_deg + (psi_dot * dt);

        // Dynamic filter weights based on actual loop time
        const float accel_correction_gain = ComputeAccelCorrectionGain(data);
        const float base_alpha_xy = IMU_FILTER_TIME_CONSTANT_XY / (IMU_FILTER_TIME_CONSTANT_XY + dt);
        const float accel_blend = (1.0f - base_alpha_xy) * accel_correction_gain;

        g_roll_deg = ((1.0f - accel_blend) * gyro_roll) + (accel_blend * accel_roll);
        g_pitch_deg = ((1.0f - accel_blend) * gyro_pitch) + (accel_blend * accel_pitch);
        g_yaw_deg = math::wrap_heading_error(gyro_yaw);

        if (mag_is_fresh)
        {
            g_last_mag_time_us = now_us;
            g_last_mag_x = data.mag_x;
            g_last_mag_y = data.mag_y;
            g_last_mag_z = data.mag_z;
        }

        // Always apply mag correction using the last known heading
        const float mag_correction_gain = ComputeMagCorrectionGain(data, g_roll_deg, g_pitch_deg);
        if (mag_correction_gain > 0.0f)
        {
            g_yaw_deg = UpdateYawKalman(mag_heading, psi_dot, dt, mag_correction_gain);
        }

        data.roll = -(g_roll_deg - IMU_LEVEL_ROLL_OFFSET_DEG);
        data.pitch = g_pitch_deg - IMU_LEVEL_PITCH_OFFSET_DEG;
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
