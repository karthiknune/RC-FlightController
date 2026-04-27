#include <Arduino.h>

#include <cmath>

#include "config.h"
#include "hal/sensors/imu.h"
#include "hal/sensors/sensor_bus.h"
#include "math/utils.h"

namespace
{

    constexpr float kDegToRad = PI / 180.0f;
    constexpr float kRadToDeg = 180.0f / PI;
    constexpr uint32_t kImuReadPeriodMs = 10;
    constexpr uint32_t kRawPreviewDurationMs = 12000;
    constexpr uint32_t kRawPreviewPrintPeriodMs = 200;
    constexpr uint32_t kPhasePrepDurationMs = 5000;
    constexpr uint32_t kPhaseRecordDurationMs = 10000;
    constexpr uint32_t kPhaseRecordSettleIgnoreMs = 1000;
    IMUData_raw g_imu_data = {};

    struct TestPhaseDef
    {
        const char *name;
        const char *instructions;
    };

    TestPhaseDef g_phases[] = {
        {"FLAT", "Keep board FLAT and STEADY on the desk."},
        {"NOSE_UP", "Point NOSE UP (Pitch +90) and hold steady."},
        {"NOSE_DOWN", "Point NOSE DOWN (Pitch -90) and hold steady."},
        {"ROLL_RIGHT", "Roll RIGHT 90 deg (Right wing down) and hold steady."},
        {"ROLL_LEFT", "Roll LEFT 90 deg (Left wing down) and hold steady."},
        {"YAW_RIGHT", "Keep flat, but turn/Yaw RIGHT 90 deg and hold steady."},
        {"YAW_LEFT", "Keep flat, but turn/Yaw LEFT 90 deg and hold steady."}};

    constexpr int kNumPhases = sizeof(g_phases) / sizeof(g_phases[0]);

    float WrapDegrees(float angle_deg)
    {
        return math::wrap_heading_error(angle_deg);
    }

    float Normalize360(float angle_deg)
    {
        while (angle_deg < 0.0f)
        {
            angle_deg += 360.0f;
        }
        while (angle_deg >= 360.0f)
        {
            angle_deg -= 360.0f;
        }
        return angle_deg;
    }

    float SafeAsinf(float value)
    {
        if (value > 1.0f)
        {
            value = 1.0f;
        }
        else if (value < -1.0f)
        {
            value = -1.0f;
        }
        return asinf(value);
    }

    struct Orientation
    {
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
    };

    Orientation MakeOrientation(float roll, float pitch, float yaw)
    {
        Orientation output;
        output.roll = roll;
        output.pitch = pitch;
        output.yaw = yaw;
        return output;
    }

    struct AxisStats
    {
        double sum = 0.0;
        float min = 1000.0f;
        float max = -1000.0f;
    };

    struct OrientationStats
    {
        AxisStats roll = {};
        AxisStats pitch = {};
        AxisStats yaw = {};
        AxisStats yaw_delta_from_flat = {};
        int count = 0;
    };

    void UpdateAxisStats(AxisStats &stats, float value)
    {
        stats.sum += value;
        if (value < stats.min)
        {
            stats.min = value;
        }
        if (value > stats.max)
        {
            stats.max = value;
        }
    }

    void UpdateOrientationStats(OrientationStats &stats, const Orientation &sample, float baseline_yaw)
    {
        UpdateAxisStats(stats.roll, sample.roll);
        UpdateAxisStats(stats.pitch, sample.pitch);
        UpdateAxisStats(stats.yaw, sample.yaw);
        UpdateAxisStats(stats.yaw_delta_from_flat, WrapDegrees(sample.yaw - baseline_yaw));
        stats.count++;
    }

    float Average(const AxisStats &stats, int count)
    {
        return (count > 0) ? static_cast<float>(stats.sum / count) : 0.0f;
    }

    enum FilterIndex
    {
        FILTER_BUILTIN = 0,
        FILTER_KALMAN = 1,
        FILTER_MAG_TILT = 2,
        FILTER_COUNT = 3
    };

    const char *kFilterNames[FILTER_COUNT] = {
        "BUILTIN_COMPLEMENTARY",
        "KALMAN",
        "MAG_TILT_ONLY"};

    struct KalmanAngle
    {
        float angle = 0.0f;
        float bias = 0.0f;
        float rate = 0.0f;
        float p00 = 1.0f;
        float p01 = 0.0f;
        float p10 = 0.0f;
        float p11 = 1.0f;
        float q_angle = 0.02f;
        float q_bias = 0.003f;
        float r_measure = 0.7f;

        void Reset(float initial_angle)
        {
            angle = initial_angle;
            bias = 0.0f;
            rate = 0.0f;
            p00 = 1.0f;
            p01 = 0.0f;
            p10 = 0.0f;
            p11 = 1.0f;
        }

        float Update(float measured_angle, float gyro_rate, float dt)
        {
            rate = gyro_rate - bias;
            angle += dt * rate;

            p00 += dt * (dt * p11 - p01 - p10 + q_angle);
            p01 -= dt * p11;
            p10 -= dt * p11;
            p11 += q_bias * dt;

            const float innovation = measured_angle - angle;
            const float s = p00 + r_measure;
            const float k0 = p00 / s;
            const float k1 = p10 / s;

            angle += k0 * innovation;
            bias += k1 * innovation;

            const float p00_temp = p00;
            const float p01_temp = p01;

            p00 -= k0 * p00_temp;
            p01 -= k0 * p01_temp;
            p10 -= k1 * p00_temp;
            p11 -= k1 * p01_temp;

            return angle;
        }

        float UpdateWrapped(float measured_angle, float gyro_rate, float dt)
        {
            rate = gyro_rate - bias;
            angle = WrapDegrees(angle + (dt * rate));

            p00 += dt * (dt * p11 - p01 - p10 + q_angle);
            p01 -= dt * p11;
            p10 -= dt * p11;
            p11 += q_bias * dt;

            const float innovation = WrapDegrees(measured_angle - angle);
            const float s = p00 + r_measure;
            const float k0 = p00 / s;
            const float k1 = p10 / s;

            angle = WrapDegrees(angle + (k0 * innovation));
            bias += k1 * innovation;

            const float p00_temp = p00;
            const float p01_temp = p01;

            p00 -= k0 * p00_temp;
            p01 -= k0 * p01_temp;
            p10 -= k1 * p00_temp;
            p11 -= k1 * p01_temp;

            return angle;
        }
    };

    struct KalmanOrientationFilter
    {
        bool seeded = false;
        KalmanAngle roll = {};
        KalmanAngle pitch = {};
        KalmanAngle yaw = {};

        void Reset()
        {
            seeded = false;
            roll = {};
            pitch = {};
            yaw = {};
            yaw.r_measure = 4.0f;
            yaw.q_angle = 0.08f;
            yaw.q_bias = 0.01f;
        }

        Orientation Update(const IMUData_raw &sample, float dt, float accel_roll, float accel_pitch, float mag_yaw)
        {
            if (!seeded)
            {
                roll.Reset(accel_roll);
                pitch.Reset(accel_pitch);
                yaw.Reset(mag_yaw);
                yaw.r_measure = 4.0f;
                yaw.q_angle = 0.08f;
                yaw.q_bias = 0.01f;
                seeded = true;
            }

            Orientation output = {};
            output.roll = roll.Update(accel_roll, sample.gyro_x, dt);
            output.pitch = pitch.Update(accel_pitch, sample.gyro_y, dt);
            output.yaw = yaw.UpdateWrapped(mag_yaw, sample.gyro_z, dt);
            return output;
        }
    };

    KalmanOrientationFilter g_kalman = {};
    OrientationStats g_stats[kNumPhases][FILTER_COUNT] = {};
    bool g_flat_baseline_valid[FILTER_COUNT] = {};
    float g_flat_baseline_yaw[FILTER_COUNT] = {};

    struct DerivedReadings
    {
        float accel_roll = 0.0f;
        float accel_pitch = 0.0f;
        float mag_yaw = 0.0f;
        float mag_heading_flat = 0.0f;
    };

    DerivedReadings ComputeDerivedReadings(const IMUData_raw &sample)
    {
        DerivedReadings readings = {};

        readings.accel_roll = atan2f(-sample.accel_y, -sample.accel_z) * kRadToDeg;
        readings.accel_pitch = atan2f(sample.accel_x,
                                      sqrtf((sample.accel_y * sample.accel_y) + (sample.accel_z * sample.accel_z))) *
                               kRadToDeg;

        const float roll_rad = readings.accel_roll * kDegToRad;
        const float pitch_rad = readings.accel_pitch * kDegToRad;
        const float cr = cosf(roll_rad);
        const float sr = sinf(roll_rad);
        const float cp = cosf(pitch_rad);
        const float sp = sinf(pitch_rad);

        const float mag_x_h = sample.mag_x * cp + sample.mag_y * sr * sp + sample.mag_z * cr * sp;
        const float mag_y_h = sample.mag_y * cr - sample.mag_z * sr;

        readings.mag_yaw = WrapDegrees(atan2f(mag_y_h, mag_x_h) * kRadToDeg);
        readings.mag_heading_flat = Normalize360(atan2f(sample.mag_y, sample.mag_x) * kRadToDeg);
        return readings;
    }

    struct FilterOutputs
    {
        Orientation builtin = {};
        Orientation kalman = {};
        Orientation mag_tilt = {};
    };

    FilterOutputs UpdateFilters(const IMUData_raw &sample, float dt)
    {
        const DerivedReadings derived = ComputeDerivedReadings(sample);

        FilterOutputs outputs = {};
        outputs.builtin = MakeOrientation(sample.roll, sample.pitch, sample.yaw);
        outputs.kalman = g_kalman.Update(sample, dt, derived.accel_roll, derived.accel_pitch, derived.mag_yaw);
        outputs.mag_tilt = MakeOrientation(derived.accel_roll, derived.accel_pitch, derived.mag_yaw);
        return outputs;
    }

    enum TestState
    {
        STATE_WAITING = 0,
        STATE_RAW_PREVIEW = 1,
        STATE_PHASE_PREP = 2,
        STATE_PHASE_RECORD = 3,
        STATE_COMPLETE = 4
    };

    TestState g_state = STATE_WAITING;
    int g_current_phase = -1;
    uint32_t g_state_start_ms = 0;
    uint32_t g_last_imu_read_ms = 0;
    uint32_t g_last_raw_print_ms = 0;
    uint32_t g_last_sample_us = 0;
    int g_last_countdown_second = -1;

    void ResetFilterState()
    {
        g_kalman.Reset();
        g_last_sample_us = 0;
        for (int i = 0; i < FILTER_COUNT; ++i)
        {
            g_flat_baseline_valid[i] = false;
            g_flat_baseline_yaw[i] = 0.0f;
        }
    }

    void PrintRawPreviewHeader()
    {
        Serial.println();
        Serial.println("RAW SENSOR PREVIEW");
        Serial.println("Move the board slowly through roll, pitch, and yaw.");
        Serial.println("Watch that accel responds to gravity direction, gyro spikes on motion,");
        Serial.println("and magnetometer heading changes mainly with yaw while the board stays flat.");
        Serial.println("Streaming at 5 Hz for 12 seconds before the orientation phases begin.");
        Serial.println();
    }

    void PrintRawPreviewSample(const IMUData_raw &sample)
    {
        const DerivedReadings derived = ComputeDerivedReadings(sample);
        const float mag_norm = sqrtf((sample.mag_x * sample.mag_x) + (sample.mag_y * sample.mag_y) + (sample.mag_z * sample.mag_z));

        Serial.printf("t=%5.1fs | ACC [m/s^2] %7.2f %7.2f %7.2f | GYRO [deg/s] %7.2f %7.2f %7.2f\n",
                      (millis() - g_state_start_ms) / 1000.0f,
                      sample.accel_x, sample.accel_y, sample.accel_z,
                      sample.gyro_x, sample.gyro_y, sample.gyro_z);
        Serial.printf("         | MAG mapped %7.2f %7.2f %7.2f | |B|=%7.2f | flat hdg=%6.1f | tilt hdg=%6.1f\n",
                      sample.mag_x, sample.mag_y, sample.mag_z,
                      mag_norm,
                      derived.mag_heading_flat,
                      Normalize360(derived.mag_yaw));
    }

    void PrintPhaseStart(int phase_index)
    {
        Serial.printf("\n>>> PHASE %d/%d: %s\n", phase_index + 1, kNumPhases, g_phases[phase_index].instructions);
        Serial.println("You have 5 seconds to position the board...");
        g_last_countdown_second = -1;
    }

    void PrintCountdown(uint32_t elapsed_ms)
    {
        const int second = elapsed_ms / 1000;
        if (second != g_last_countdown_second)
        {
            g_last_countdown_second = second;
            if (second >= 0 && second < 5)
            {
                Serial.printf("%d... ", 5 - second);
            }
        }
    }

    void PrintFilterSummary(const char *filter_name, const OrientationStats &stats)
    {
        const float avg_roll = Average(stats.roll, stats.count);
        const float avg_pitch = Average(stats.pitch, stats.count);
        const float avg_yaw = Average(stats.yaw, stats.count);
        const float avg_yaw_delta = Average(stats.yaw_delta_from_flat, stats.count);

        Serial.printf("  <%s>\n", filter_name);
        Serial.printf("    Samples: %d\n", stats.count);
        Serial.printf("    Roll : Avg %7.2f  (Min: %7.2f, Max: %7.2f)\n", avg_roll, stats.roll.min, stats.roll.max);
        Serial.printf("    Pitch: Avg %7.2f  (Min: %7.2f, Max: %7.2f)\n", avg_pitch, stats.pitch.min, stats.pitch.max);
        Serial.printf("    Yaw  : Avg %7.2f  (Min: %7.2f, Max: %7.2f)\n", avg_yaw, stats.yaw.min, stats.yaw.max);
        Serial.printf("    dYaw : Avg %7.2f  (Min: %7.2f, Max: %7.2f) vs FLAT\n",
                      avg_yaw_delta,
                      stats.yaw_delta_from_flat.min,
                      stats.yaw_delta_from_flat.max);
    }

    void PrintFinalSummary()
    {
        Serial.println("\n============================================");
        Serial.println("               TEST COMPLETE                ");
        Serial.println("============================================");
        Serial.println("Please copy and paste the following output:\n");

        for (int phase_index = 0; phase_index < kNumPhases; ++phase_index)
        {
            Serial.printf("[%s]\n", g_phases[phase_index].name);
            for (int filter_index = 0; filter_index < FILTER_COUNT; ++filter_index)
            {
                PrintFilterSummary(kFilterNames[filter_index], g_stats[phase_index][filter_index]);
            }
            Serial.println();
        }

        Serial.println("Yaw sanity notes:");
        Serial.println("  - During YAW_RIGHT, dYaw should be about +90 deg from FLAT.");
        Serial.println("  - During YAW_LEFT, dYaw should be about -90 deg from FLAT.");
        Serial.println("  - During ROLL_LEFT/ROLL_RIGHT, MAG_TILT_ONLY dYaw should stay near 0 deg.");
        Serial.println("  - Large yaw changes during pure roll usually mean magnetometer axis/sign mapping is still wrong.");
        Serial.println();
        Serial.println("What to update after this test:");
        Serial.println("  1. Mounting/sign conventions live in include/config.h:");
        Serial.println("     IMU_BODY_FRAME_* and IMU_MAG_SENSOR_ALIGN_*");
        Serial.println("  2. Calibration values copied from startup calibrators go in include/config.h:");
        Serial.println("     IMU_LEVEL_* and IMU_MAG_OFFSET_*/IMU_MAG_SCALE_*");
        Serial.println("  3. Production fusion used by test/main.cpp lives in src/hal/sensors/imu.cpp");
        Serial.println("     via IMU_Read(), declared in include/hal/sensors/imu.h");
        Serial.println("  4. IMUData_raw roll/pitch/yaw in include/datatypes.h are the fused outputs");
        Serial.println("     consumed by test/main.cpp and the flight-control stack.");
        Serial.println("  5. If YAW_RIGHT/YAW_LEFT are wrong but MAG_TILT_ONLY is right, tune yaw fusion in");
        Serial.println("     src/hal/sensors/imu.cpp. If MAG_TILT_ONLY is wrong, fix config/calibration first.");
        Serial.println("============================================");
    }

} // namespace

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
    }

    Serial.println("\n============================================");
    Serial.println("   IMU RAW + ORIENTATION COMPARISON TEST    ");
    Serial.println("============================================");

    if (!SensorBus_Init())
    {
        Serial.println("Sensor I2C bus init failed.");
    }

    IMU_Init();

    Serial.println("IMU initialized.");
    Serial.println("Press Enter in the serial monitor to begin.");
}

void loop()
{
    const uint32_t now_ms = millis();

    if ((now_ms - g_last_imu_read_ms) >= kImuReadPeriodMs)
    {
        g_last_imu_read_ms = now_ms;
        IMU_Read(g_imu_data);

        if (g_imu_data.healthy && g_state >= STATE_RAW_PREVIEW && g_state <= STATE_PHASE_RECORD)
        {
            const uint32_t now_us = micros();
            float dt = 0.01f;
            if (g_last_sample_us != 0)
            {
                dt = static_cast<float>(now_us - g_last_sample_us) / 1000000.0f;
            }
            g_last_sample_us = now_us;

            if (dt > 0.0f && dt < 0.2f)
            {
                const FilterOutputs outputs = UpdateFilters(g_imu_data, dt);

                if (g_state == STATE_PHASE_RECORD && g_current_phase >= 0 && g_current_phase < kNumPhases)
                {
                    const uint32_t phase_record_elapsed_ms = now_ms - g_state_start_ms;

                    if (g_current_phase == 0)
                    {
                        g_flat_baseline_yaw[FILTER_BUILTIN] = outputs.builtin.yaw;
                        g_flat_baseline_yaw[FILTER_KALMAN] = outputs.kalman.yaw;
                        g_flat_baseline_yaw[FILTER_MAG_TILT] = outputs.mag_tilt.yaw;
                        g_flat_baseline_valid[FILTER_BUILTIN] = true;
                        g_flat_baseline_valid[FILTER_KALMAN] = true;
                        g_flat_baseline_valid[FILTER_MAG_TILT] = true;
                    }

                    const float builtin_baseline = g_flat_baseline_valid[FILTER_BUILTIN] ? g_flat_baseline_yaw[FILTER_BUILTIN] : outputs.builtin.yaw;
                    const float kalman_baseline = g_flat_baseline_valid[FILTER_KALMAN] ? g_flat_baseline_yaw[FILTER_KALMAN] : outputs.kalman.yaw;
                    const float mag_tilt_baseline = g_flat_baseline_valid[FILTER_MAG_TILT] ? g_flat_baseline_yaw[FILTER_MAG_TILT] : outputs.mag_tilt.yaw;

                    if (phase_record_elapsed_ms >= kPhaseRecordSettleIgnoreMs)
                    {
                        UpdateOrientationStats(g_stats[g_current_phase][FILTER_BUILTIN], outputs.builtin, builtin_baseline);
                        UpdateOrientationStats(g_stats[g_current_phase][FILTER_KALMAN], outputs.kalman, kalman_baseline);
                        UpdateOrientationStats(g_stats[g_current_phase][FILTER_MAG_TILT], outputs.mag_tilt, mag_tilt_baseline);
                    }

                    if (((g_current_phase == 5) || (g_current_phase == 6)) &&
                        ((g_stats[g_current_phase][FILTER_MAG_TILT].count % 100) == 0))
                    {
                        Serial.printf("%s dbg | MagXYZ %7.2f %7.2f %7.2f | MAG_TILT yaw %7.2f | dYaw %7.2f\n",
                                      g_phases[g_current_phase].name,
                                      g_imu_data.mag_x,
                                      g_imu_data.mag_y,
                                      g_imu_data.mag_z,
                                      outputs.mag_tilt.yaw,
                                      WrapDegrees(outputs.mag_tilt.yaw - mag_tilt_baseline));
                    }
                }
            }
        }
    }

    if (g_state == STATE_WAITING)
    {
        if (Serial.available())
        {
            while (Serial.available())
            {
                Serial.read();
            }

            ResetFilterState();
            g_state = STATE_RAW_PREVIEW;
            g_state_start_ms = now_ms;
            g_last_raw_print_ms = 0;
            PrintRawPreviewHeader();
        }
        return;
    }

    if (g_state == STATE_RAW_PREVIEW)
    {
        if (g_imu_data.healthy && (now_ms - g_last_raw_print_ms) >= kRawPreviewPrintPeriodMs)
        {
            g_last_raw_print_ms = now_ms;
            PrintRawPreviewSample(g_imu_data);
        }

        if ((now_ms - g_state_start_ms) >= kRawPreviewDurationMs)
        {
            g_current_phase = 0;
            g_state = STATE_PHASE_PREP;
            g_state_start_ms = now_ms;
            PrintPhaseStart(g_current_phase);
        }
        return;
    }

    if (g_state == STATE_PHASE_PREP)
    {
        const uint32_t elapsed_ms = now_ms - g_state_start_ms;
        PrintCountdown(elapsed_ms);

        if (elapsed_ms >= kPhasePrepDurationMs)
        {
            g_state = STATE_PHASE_RECORD;
            g_state_start_ms = now_ms;
            Serial.println("\nRECORDING for 10 seconds. HOLD STEADY!");
        }
        return;
    }

    if (g_state == STATE_PHASE_RECORD)
    {
        const uint32_t elapsed_ms = now_ms - g_state_start_ms;
        if (elapsed_ms >= kPhaseRecordDurationMs)
        {
            Serial.println("DONE recording this phase.");
            g_current_phase++;

            if (g_current_phase < kNumPhases)
            {
                g_state = STATE_PHASE_PREP;
                g_state_start_ms = now_ms;
                PrintPhaseStart(g_current_phase);
            }
            else
            {
                g_state = STATE_COMPLETE;
                PrintFinalSummary();
            }
        }
        return;
    }
}
