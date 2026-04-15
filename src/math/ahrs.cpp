
#include "math/ahrs.h"
#include "config.h"
#include "hal/sensors/imu.h"

#include <Arduino.h>
#include <math.h>

// ============================================================================
// INTERNAL AHRS STATE
// ============================================================================

// Current fused attitude estimate (degrees).
static float roll_deg_fused  = 0.0f;
static float pitch_deg_fused = 0.0f;
static float yaw_deg_fused   = 0.0f;

// Latest raw IMU sample used to compute the above attitude.
static imu_sample_t last_imu_sample = {};

// Flag to ensure ahrs_init() was called.
static bool ahrs_initialized = false;


// ============================================================================
// SMALL MATH HELPERS
// ============================================================================

static float wrap_angle_360(float angle_deg)
{
    while (angle_deg >= 360.0f) angle_deg -= 360.0f;
    while (angle_deg < 0.0f)   angle_deg += 360.0f;
    return angle_deg;
}

static float wrap_angle_180(float angle_deg)
{
    while (angle_deg > 180.0f)   angle_deg -= 360.0f;
    while (angle_deg <= -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

static float rad_to_deg(float rad) { return rad * 180.0f / PI; }
static float deg_to_rad(float deg) { return deg * PI / 180.0f; }


// ============================================================================
// ACCELEROMETER -> ROLL / PITCH
// ============================================================================

static void accel_to_roll_pitch(float ax_g, float ay_g, float az_g,
                                float &roll_deg, float &pitch_deg)
{
    // Reject extreme values to avoid NaNs during transients.
    if (fabs(ax_g) > 4.0f || fabs(ay_g) > 4.0f || fabs(az_g) > 4.0f) {
        return;
    }

    const float roll_rad  = atan2f(ay_g, az_g);
    const float pitch_rad = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g));

    roll_deg  = rad_to_deg(roll_rad);
    pitch_deg = rad_to_deg(pitch_rad);
}


// ============================================================================
// MAGNETOMETER -> HEADING / YAW
// ============================================================================
// Tilt-compensated yaw using magnetometer + current roll/pitch.
// ============================================================================

static float mag_to_heading_yaw(float mx_uT, float my_uT, float mz_uT,
                                float roll_deg, float pitch_deg)
{
    const float roll_rad  = deg_to_rad(roll_deg);
    const float pitch_rad = deg_to_rad(pitch_deg);

    const float mx_comp = mx_uT * cosf(pitch_rad) + mz_uT * sinf(pitch_rad);
    const float my_comp =
        mx_uT * sinf(roll_rad) * sinf(pitch_rad) +
        my_uT * cosf(roll_rad) -
        mz_uT * sinf(roll_rad) * cosf(pitch_rad);

    float yaw_rad = atan2f(-my_comp, mx_comp);
    float yaw_deg = rad_to_deg(yaw_rad);

    return wrap_angle_360(yaw_deg);
}


// ============================================================================
// PUBLIC AHRS API
// ============================================================================

void ahrs_init()
{
    roll_deg_fused  = 0.0f;
    pitch_deg_fused = 0.0f;
    yaw_deg_fused   = 0.0f;

    last_imu_sample = {};
    ahrs_initialized = true;
}


bool ahrs_update(float dt_s)
{
    if (!ahrs_initialized) {
        return false;
    }

    // If dt_s is nonsense, fall back to a nominal update period from config.h.
    if (dt_s <= 0.0f) {
        dt_s = ahrs_default_dt_s;
    }

    // ------------------------------------------------------------------------
    // 1) Read raw IMU sample from hardware layer.
    // ------------------------------------------------------------------------
    imu_sample_t sample;
    if (!imu_read_sample(sample)) {
        return false;
    }
    last_imu_sample = sample;

    // ------------------------------------------------------------------------
    // 2) Estimate roll/pitch from accelerometer.
    // ------------------------------------------------------------------------
    float roll_deg_accel  = 0.0f;
    float pitch_deg_accel = 0.0f;

    accel_to_roll_pitch(
        sample.ax_g,
        sample.ay_g,
        sample.az_g,
        roll_deg_accel,
        pitch_deg_accel
    );

    // ------------------------------------------------------------------------
    // 3) Integrate gyro rates (deg/s * s = deg).
    // ------------------------------------------------------------------------
    float roll_deg_gyro  = roll_deg_fused  + sample.gx_dps * dt_s;
    float pitch_deg_gyro = pitch_deg_fused + sample.gy_dps * dt_s;
    float yaw_deg_gyro   = yaw_deg_fused   + sample.gz_dps * dt_s;

    yaw_deg_gyro = wrap_angle_360(yaw_deg_gyro);

    // ------------------------------------------------------------------------
    // 4) Estimate yaw from magnetometer.
    // ------------------------------------------------------------------------
    float yaw_deg_mag = mag_to_heading_yaw(
        sample.mx_uT,
        sample.my_uT,
        sample.mz_uT,
        roll_deg_fused,
        pitch_deg_fused
    );

    // ------------------------------------------------------------------------
    // 5) Complementary fusion for roll/pitch.
    // ------------------------------------------------------------------------
    roll_deg_fused =
        roll_pitch_alpha * roll_deg_gyro +
        (1.0f - roll_pitch_alpha) * roll_deg_accel;

    pitch_deg_fused =
        roll_pitch_alpha * pitch_deg_gyro +
        (1.0f - roll_pitch_alpha) * pitch_deg_accel;

    // ------------------------------------------------------------------------
    // 6) Complementary fusion for yaw.
    // ------------------------------------------------------------------------
    float yaw_err = yaw_deg_mag - yaw_deg_gyro;
    yaw_err = wrap_angle_180(yaw_err);

    yaw_deg_fused = yaw_deg_gyro + (1.0f - yaw_alpha) * yaw_err;
    yaw_deg_fused = wrap_angle_360(yaw_deg_fused);

    return true;
}


attitude_rpy_t ahrs_get_attitude()
{
    attitude_rpy_t att;
    att.roll_deg  = roll_deg_fused;
    att.pitch_deg = pitch_deg_fused;
    att.yaw_deg   = yaw_deg_fused;
    return att;
}


imu_sample_t ahrs_get_last_raw_sample()
{
    return last_imu_sample;
}
