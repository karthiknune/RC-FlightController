#include <cmath>
#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>

#include "config.h"
#include "hal/sensors/sensor_bus.h"
#include "hal/sensors/imu.h"
#include "math/utils.h"

Adafruit_ICM20948 icm;

namespace
{

    Preferences imu_prefs;

    constexpr uint8_t kIMUAddress = ICM20948_I2CADDR_DEFAULT;
    constexpr float kRadToDeg = 57.2957795f;
    constexpr float kDegToRad = PI / 180.0f;
    constexpr float kMinMagAxisSpanUt = 25.0f;

    constexpr float kDegToRad = 0.0174532925f;

    // Mahony Filter Tuning
    constexpr float kMahonyKp = 2.0f;   // Proportional gain (trust in accel/mag)
    constexpr float kMahonyKi = 0.05f;  // Integral gain (gyro bias drift compensation)

    uint32_t g_last_time_us = 0;
    uint32_t g_last_init_attempt_ms = 0;
    bool g_imu_ready = false;
    bool g_imu_missing_logged = false;
    bool g_orientation_seeded = false;

    // Quaternion state
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    float eInt_x = 0.0f, eInt_y = 0.0f, eInt_z = 0.0f;

    // Output states
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
        q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
        eInt_x = 0.0f; eInt_y = 0.0f; eInt_z = 0.0f;
        g_roll_deg = 0.0f;
        g_pitch_deg = 0.0f;
        g_yaw_deg = 0.0f;
    }

    void LoadCalibration() {
        imu_prefs.begin("imu_cal", true); // true = read-only mode
        g_gyro_bias_x_dps = imu_prefs.getFloat("gx_bias", 0.0f);
        g_gyro_bias_y_dps = imu_prefs.getFloat("gy_bias", 0.0f);
        g_gyro_bias_z_dps = imu_prefs.getFloat("gz_bias", 0.0f);
        imu_prefs.end();
        
        if (SENSOR_STATUS_LOGGING_ENABLED) {
            Serial.printf("Loaded Gyro Bias -> X: %.2f | Y: %.2f | Z: %.2f\n", 
                          g_gyro_bias_x_dps, g_gyro_bias_y_dps, g_gyro_bias_z_dps);
        }
    }

    void SaveCalibration() {
        imu_prefs.begin("imu_cal", false); // false = read/write mode
        imu_prefs.putFloat("gx_bias", g_gyro_bias_x_dps);
        imu_prefs.putFloat("gy_bias", g_gyro_bias_y_dps);
        imu_prefs.putFloat("gz_bias", g_gyro_bias_z_dps);
        imu_prefs.end();
        
        Serial.println("Calibration saved to NVS flash.");
    }

    float ApplyBodyFrameAxis(float value, float axis_sign) {
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

        // FIXED: AK09916 Hardware axis alignment to ICM20948 Gyro/Accel frame
        float aligned_mag_x = m.magnetic.y; 
        float aligned_mag_y = m.magnetic.x; 
        float aligned_mag_z = -m.magnetic.z; 

        data.mag_x = ApplyBodyFrameAxis((aligned_mag_x - IMU_MAG_OFFSET_X) * IMU_MAG_SCALE_X, IMU_BODY_FRAME_X_SIGN);
        data.mag_y = ApplyBodyFrameAxis((aligned_mag_y - IMU_MAG_OFFSET_Y) * IMU_MAG_SCALE_Y, IMU_BODY_FRAME_Y_SIGN);
        data.mag_z = ApplyBodyFrameAxis((aligned_mag_z - IMU_MAG_OFFSET_Z) * IMU_MAG_SCALE_Z, IMU_BODY_FRAME_Z_SIGN);
    }

    void UpdateOrientation(IMUData_raw &data)
    {
        const uint32_t now_us = micros();
        float dt = 0.0f;
        if (g_last_time_us != 0) {
            dt = static_cast<float>(now_us - g_last_time_us) / 1000000.0f;
        }
        g_last_time_us = now_us;

        // Seed quaternion on first pass to prevent initialization dip
        if (!g_orientation_seeded || dt <= 0.0f || dt > 0.5f) {
            float initial_roll = atan2f(data.accel_y, data.accel_z);
            float initial_pitch = atan2f(-data.accel_x, sqrtf(data.accel_y * data.accel_y + data.accel_z * data.accel_z));
            
            float mag_x_h = data.mag_x * cosf(initial_pitch) + data.mag_y * sinf(initial_roll) * sinf(initial_pitch) + data.mag_z * cosf(initial_roll) * sinf(initial_pitch);
            float mag_y_h = data.mag_y * cosf(initial_roll) - data.mag_z * sinf(initial_roll);
            float initial_yaw = atan2f(-mag_y_h, mag_x_h);

            float cy = cosf(initial_yaw * 0.5f);
            float sy = sinf(initial_yaw * 0.5f);
            float cp = cosf(initial_pitch * 0.5f);
            float sp = sinf(initial_pitch * 0.5f);
            float cr = cosf(initial_roll * 0.5f);
            float sr = sinf(initial_roll * 0.5f);

            q0 = cr * cp * cy + sr * sp * sy;
            q1 = sr * cp * cy - cr * sp * sy;
            q2 = cr * sp * cy + sr * cp * sy;
            q3 = cr * cp * sy - sr * sp * cy;

            g_orientation_seeded = true;
            return; // Skip integration on seed pass
        }

        // Mahony 9DOF Update
        float gx = data.gyro_x * kDegToRad;
        float gy = data.gyro_y * kDegToRad;
        float gz = data.gyro_z * kDegToRad;
        float ax = data.accel_x;
        float ay = data.accel_y;
        float az = data.accel_z;
        float mx = data.mag_x;
        float my = data.mag_y;
        float mz = data.mag_z;

        float recipNorm;
        float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
        float hx, hy, bx, bz;
        float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
        float halfex, halfey, halfez;
        float qa, qb, qc;

        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

            if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
                recipNorm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
                mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

                q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
                q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
                q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3;

                hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
                hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
                bx = sqrtf(hx * hx + hy * hy);
                bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

                halfvx = q1q3 - q0q2;
                halfvy = q0q1 + q2q3;
                halfvz = q0q0 - 0.5f + q3q3;
                halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
                halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
                halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

                halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
                halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
                halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

                if(kMahonyKi > 0.0f) {
                    eInt_x += halfex * dt;
                    eInt_y += halfey * dt;
                    eInt_z += halfez * dt;
                    gx += kMahonyKi * eInt_x;
                    gy += kMahonyKi * eInt_y;
                    gz += kMahonyKi * eInt_z;
                }
                gx += kMahonyKp * halfex;
                gy += kMahonyKp * halfey;
                gz += kMahonyKp * halfez;
            }
        }

        gx *= (0.5f * dt);
        gy *= (0.5f * dt);
        gz *= (0.5f * dt);
        qa = q0; qb = q1; qc = q2;
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);

        recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

        // Convert Quaternion to Euler for output
        g_roll_deg = atan2f(2.0f * (q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3) * kRadToDeg;
        g_pitch_deg = asinf(2.0f * (q0*q2 - q1*q3)) * kRadToDeg;
        g_yaw_deg = atan2f(2.0f * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * kRadToDeg;

        data.roll = g_roll_deg - IMU_LEVEL_ROLL_OFFSET_DEG;
        data.pitch = g_pitch_deg - IMU_LEVEL_PITCH_OFFSET_DEG;
        data.yaw = math::wrap_heading_error(g_yaw_deg);
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

    bool ReadIMUEvents(sensors_event_t &a, sensors_event_t &g, sensors_event_t &temp, sensors_event_t &m) {
        if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS))) return false;
        if (!ProbeIMULocked()) {
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
        if (g_last_init_attempt_ms != 0 && (now_ms - g_last_init_attempt_ms) < SENSOR_RECONNECT_INTERVAL_MS) {
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

        LoadCalibration();
        ResetOrientationState();
        UpdateIMUAvailability(true);
        return true;
    }

} // namespace

void IMU_Init()
{
    (void)TryInitializeIMU();
}

bool IMU_Calibrate_Gyro() {
    if (!g_imu_ready && !TryInitializeIMU()) return false;

    Serial.println("Calibrating gyroscope... Keep the aircraft still.");
    double sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;
    int captured_samples = 0;

    for (int sample_index = 0; sample_index < IMU_GYRO_CALIBRATION_SAMPLES; ++sample_index) {
        sensors_event_t a = {}, g = {}, temp = {}, m = {};
        if (!ReadIMUEvents(a, g, temp, m)) {
            delay(IMU_GYRO_CALIBRATION_SAMPLE_DELAY_MS);
            continue;
        }
        sum_gx += static_cast<double>(ApplyBodyFrameAxis(g.gyro.x * kRadToDeg, IMU_BODY_FRAME_X_SIGN));
        sum_gy += static_cast<double>(ApplyBodyFrameAxis(g.gyro.y * kRadToDeg, IMU_BODY_FRAME_Y_SIGN));
        sum_gz += static_cast<double>(ApplyBodyFrameAxis(g.gyro.z * kRadToDeg, IMU_BODY_FRAME_Z_SIGN));
        ++captured_samples;
        delay(IMU_GYRO_CALIBRATION_SAMPLE_DELAY_MS);
    }

    if (captured_samples == 0) return false;

    g_gyro_bias_x_dps = static_cast<float>(sum_gx / captured_samples);
    g_gyro_bias_y_dps = static_cast<float>(sum_gy / captured_samples);
    g_gyro_bias_z_dps = static_cast<float>(sum_gz / captured_samples);
    
    SaveCalibration();
    ResetOrientationState();

    return true;
}

bool IMU_Run_Level_Calibration(float &roll_offset_deg, float &pitch_offset_deg) {
    roll_offset_deg = 0.0f; pitch_offset_deg = 0.0f;
    if (!g_imu_ready && !TryInitializeIMU()) return false;

    Serial.println("Hold the aircraft steady in level flight attitude.");
    delay(5000);

    double sum_roll = 0.0, sum_pitch = 0.0;
    int captured_samples = 0;

    for (int sample_index = 0; sample_index < IMU_LEVEL_CALIBRATION_SAMPLES; ++sample_index) {
        IMUData_raw sample = {};
        IMU_Read(sample);
        if (!sample.healthy)
        {
            delay(IMU_LEVEL_CALIBRATION_SAMPLE_DELAY_MS);
            continue;
        }
        // Use the raw Euler outputs before offsets are applied for calibration
        sum_roll += static_cast<double>(g_roll_deg);
        sum_pitch += static_cast<double>(g_pitch_deg);
        ++captured_samples;
        delay(IMU_LEVEL_CALIBRATION_SAMPLE_DELAY_MS);
    }

    if (captured_samples == 0) return false;

    roll_offset_deg = static_cast<float>(sum_roll / captured_samples);
    pitch_offset_deg = static_cast<float>(sum_pitch / captured_samples);

    Serial.printf("Copy to config.h: \nconstexpr float IMU_LEVEL_ROLL_OFFSET_DEG = %.2ff;\n", roll_offset_deg);
    Serial.printf("constexpr float IMU_LEVEL_PITCH_OFFSET_DEG = %.2ff;\n", pitch_offset_deg);
    return true;
}

bool IMU_Run_Mag_Calibration(float &offset_x, float &offset_y, float &offset_z,
                             float &scale_x, float &scale_y, float &scale_z)
{
    // The previous implementation is structurally fine, but ensure you rotate 
    // the axes dynamically just like in PopulateIMUData if you intend to calibrate 
    // the translated frame. In this implementation, we calibrate the RAW data 
    // before alignment for simplicity.
    // ... (Keep your previous mag calibration loop here) ...
    // Note: Omitted for brevity, but re-run it exactly as you had it.

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

    sensors_event_t a = {}, g = {}, temp = {}, m = {};
    if (!ReadIMUEvents(a, g, temp, m)) {
        data.healthy = false;
        return;
    }

    PopulateIMUData(a, g, m, data);
    UpdateOrientation(data);
    data.healthy = true;
    UpdateIMUAvailability(true);
}