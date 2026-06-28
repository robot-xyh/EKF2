/*
 * This file adapts Betaflight's position estimator design to the Orange Pi
 * user-space sensor program.
 *
 * Betaflight is free software under the GNU General Public License version 3
 * or later. This adapted educational/research module preserves that licensing
 * for the Betaflight-derived estimator logic.
 *
 * Source baseline:
 * https://github.com/betaflight/betaflight
 * commit 25a7f7417164d88c0db833db00af2066857e1258
 */
#include "bf_position_estimator.h"
#include "bf_position_filter.h"

#include <math.h>
#include <string.h>

#define Q_ACCEL_XY             50000.0f
#define Q_ACCEL_Z              20000.0f
#define INITIAL_POS_VAR        10000.0f
#define INITIAL_VEL_VAR        10000.0f
#define R_GPS_POS_BASE           500.0f
#define R_GPS_VEL_BASE           100.0f
#define R_GPS_ALT_BASE         60000.0f
#define R_BARO_ALT              1500.0f
#define GPS_DOP_MIN_VALID        100U
#define GPS_DOP_UNKNOWN_R_SCALE  100.0f
#define MEASUREMENT_TIMEOUT_US   2000000ULL
#define EARTH_RADIUS_CM          637100000.0

static bf_position_kalman_t kf_east;
static bf_position_kalman_t kf_north;
static bf_position_kalman_t kf_up;
static bfnav_output_t output;

static uint8_t xy_enabled;
static uint64_t last_xy_measurement_us;
static uint64_t last_z_measurement_us;

static uint8_t gps_origin_set;
static int32_t gps_origin_lat_e7;
static int32_t gps_origin_lon_e7;
static double gps_origin_lat_rad;
static float gps_alt_offset_cm;
static uint8_t gps_alt_offset_set;

static float baro_alt_offset_cm;
static uint8_t baro_offset_set;

static uint64_t elapsed_us(uint64_t now, uint64_t then)
{
    return (now >= then) ? (now - then) : 0;
}

static float constrain_float(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static float gps_r(float base_r, uint16_t dop)
{
    if (dop < GPS_DOP_MIN_VALID) {
        return base_r * GPS_DOP_UNKNOWN_R_SCALE;
    }

    const float dop_scale = (float)dop * 0.01f;
    return base_r * dop_scale * dop_scale;
}

static void gps_relative_position_cm(int32_t lat_e7, int32_t lon_e7, float *east_cm, float *north_cm)
{
    const double lat = (double)lat_e7 * 1e-7 * M_PI / 180.0;
    const double lon = (double)lon_e7 * 1e-7 * M_PI / 180.0;
    const double origin_lat = (double)gps_origin_lat_e7 * 1e-7 * M_PI / 180.0;
    const double origin_lon = (double)gps_origin_lon_e7 * 1e-7 * M_PI / 180.0;

    *north_cm = (float)((lat - origin_lat) * EARTH_RADIUS_CM);
    *east_cm = (float)((lon - origin_lon) * cos(gps_origin_lat_rad) * EARTH_RADIUS_CM);
}

void BFNav_Init(void)
{
    BF_KalmanInit(&kf_east, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, Q_ACCEL_XY);
    BF_KalmanInit(&kf_north, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, Q_ACCEL_XY);
    BF_KalmanInit(&kf_up, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, Q_ACCEL_Z);

    memset(&output, 0, sizeof(output));
    xy_enabled = 0;
    last_xy_measurement_us = 0;
    last_z_measurement_us = 0;
    gps_origin_set = 0;
    gps_origin_lat_e7 = 0;
    gps_origin_lon_e7 = 0;
    gps_origin_lat_rad = 0.0;
    gps_alt_offset_cm = 0.0f;
    gps_alt_offset_set = 0;
    baro_alt_offset_cm = 0.0f;
    baro_offset_set = 0;
}

void BFNav_ResetXY(void)
{
    BF_KalmanInit(&kf_east, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, Q_ACCEL_XY);
    BF_KalmanInit(&kf_north, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, Q_ACCEL_XY);
    output.position_enu_cm[0] = 0.0f;
    output.position_enu_cm[1] = 0.0f;
    output.velocity_enu_cms[0] = 0.0f;
    output.velocity_enu_cms[1] = 0.0f;
    output.valid_xy = 0;
    output.trust_xy = 0.0f;
    xy_enabled = 0;
    gps_origin_set = 0;
    last_xy_measurement_us = 0;
}

void BFNav_ResetZ(void)
{
    BF_KalmanInit(&kf_up, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, Q_ACCEL_Z);
    output.position_enu_cm[2] = 0.0f;
    output.velocity_enu_cms[2] = 0.0f;
    output.valid_z = 0;
    output.trust_z = 0.0f;
    gps_alt_offset_cm = 0.0f;
    gps_alt_offset_set = 0;
    baro_alt_offset_cm = 0.0f;
    baro_offset_set = 0;
    last_z_measurement_us = 0;
}

void BFNav_Update(const bfnav_input_t *in, bfnav_output_t *out)
{
    if (in == 0) {
        return;
    }

    float dt = in->dt_s;
    if (dt <= 0.0f || dt > 0.2f) {
        dt = 0.01f;
    }

    if (in->gps_valid && !gps_origin_set) {
        gps_origin_set = 1;
        gps_origin_lat_e7 = in->gps_lat_e7;
        gps_origin_lon_e7 = in->gps_lon_e7;
        gps_origin_lat_rad = (double)gps_origin_lat_e7 * 1e-7 * M_PI / 180.0;
        output.origin_lat_e7 = gps_origin_lat_e7;
        output.origin_lon_e7 = gps_origin_lon_e7;
        output.origin_valid = 1;
        xy_enabled = 1;
    }

    BF_KalmanPredict(&kf_up, dt, in->accel_enu_cmss[2]);

    if (xy_enabled) {
        BF_KalmanPredict(&kf_east, dt, in->accel_enu_cmss[0]);
        BF_KalmanPredict(&kf_north, dt, in->accel_enu_cmss[1]);
    }

    output.gps_contributing = (in->gps_valid && gps_origin_set) ? 1 : 0;

    if (in->gps_valid && in->gps_new && gps_origin_set) {
        float gps_east_cm = 0.0f;
        float gps_north_cm = 0.0f;
        gps_relative_position_cm(in->gps_lat_e7, in->gps_lon_e7, &gps_east_cm, &gps_north_cm);

        const float r_pos = gps_r(R_GPS_POS_BASE, in->gps_pdop);
        const float r_vel = gps_r(R_GPS_VEL_BASE, in->gps_pdop);
        BF_KalmanUpdatePosition(&kf_east, gps_east_cm, r_pos);
        BF_KalmanUpdatePosition(&kf_north, gps_north_cm, r_pos);
        BF_KalmanUpdateVelocity(&kf_east, in->gps_vel_enu_cms[0], r_vel);
        BF_KalmanUpdateVelocity(&kf_north, in->gps_vel_enu_cms[1], r_vel);

        if (!gps_alt_offset_set) {
            gps_alt_offset_cm = in->gps_alt_cm;
            gps_alt_offset_set = 1;
        }
        BF_KalmanUpdatePosition(&kf_up, in->gps_alt_cm - gps_alt_offset_cm,
                                gps_r(R_GPS_ALT_BASE, in->gps_pdop));

        last_xy_measurement_us = in->time_us;
        last_z_measurement_us = in->time_us;
    }

    if (in->baro_valid) {
        if (!baro_offset_set) {
            baro_alt_offset_cm = in->baro_alt_cm;
            baro_offset_set = 1;
        }
        BF_KalmanUpdatePosition(&kf_up, in->baro_alt_cm - baro_alt_offset_cm, R_BARO_ALT);
        last_z_measurement_us = in->time_us;
    }

    output.position_enu_cm[0] = BF_KalmanGetPosition(&kf_east);
    output.position_enu_cm[1] = BF_KalmanGetPosition(&kf_north);
    output.position_enu_cm[2] = BF_KalmanGetPosition(&kf_up);
    output.velocity_enu_cms[0] = BF_KalmanGetVelocity(&kf_east);
    output.velocity_enu_cms[1] = BF_KalmanGetVelocity(&kf_north);
    output.velocity_enu_cms[2] = BF_KalmanGetVelocity(&kf_up);

    output.valid_xy = (xy_enabled && last_xy_measurement_us > 0 &&
                       elapsed_us(in->time_us, last_xy_measurement_us) < MEASUREMENT_TIMEOUT_US) ? 1 : 0;
    output.valid_z = (last_z_measurement_us > 0 &&
                      elapsed_us(in->time_us, last_z_measurement_us) < MEASUREMENT_TIMEOUT_US) ? 1 : 0;

    const float xy_var = 0.5f * (BF_KalmanGetPositionVariance(&kf_east) +
                                 BF_KalmanGetPositionVariance(&kf_north));
    output.trust_xy = constrain_float(1.0f / (1.0f + xy_var / 10000.0f), 0.0f, 1.0f);
    output.trust_z = constrain_float(1.0f / (1.0f + BF_KalmanGetPositionVariance(&kf_up) / 10000.0f),
                                     0.0f, 1.0f);

    if (out != 0) {
        *out = output;
    }
}

const bfnav_output_t *BFNav_GetOutput(void)
{
    return &output;
}
