/*
 * Small deterministic tests for the Betaflight-style navigation adapter.
 */
#include "bf_position_estimator.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int nearf(float a, float b, float tol)
{
    return fabsf(a - b) <= tol;
}

int main(void)
{
    BFNav_Init();

    bfnav_input_t in;
    bfnav_output_t out;
    memset(&in, 0, sizeof(in));
    memset(&out, 0, sizeof(out));

    for (int i = 0; i < 100; i++) {
        in.time_us = (uint64_t)(i + 1) * 10000ULL;
        in.dt_s = 0.01f;
        in.baro_valid = 1;
        in.baro_alt_cm = 1000.0f;
        BFNav_Update(&in, &out);
    }

    if (!out.valid_z || !nearf(out.position_enu_cm[2], 0.0f, 5.0f)) {
        fprintf(stderr, "Z convergence/validity failed: valid=%u z=%.2f\n",
                out.valid_z, out.position_enu_cm[2]);
        return 1;
    }

    memset(&in, 0, sizeof(in));
    in.time_us = 1100000ULL;
    in.dt_s = 0.01f;
    in.gps_valid = 1;
    in.gps_new = 1;
    in.gps_lat_e7 = 300000000;
    in.gps_lon_e7 = 1140000000;
    in.gps_alt_cm = 1000.0f;
    in.gps_pdop = 150;
    BFNav_Update(&in, &out);

    in.time_us += 100000ULL;
    in.gps_lon_e7 += 10; /* about 0.96 m east at 30 deg latitude */
    in.gps_vel_enu_cms[0] = 50.0f;
    BFNav_Update(&in, &out);

    if (!out.valid_xy || out.position_enu_cm[0] <= 0.0f) {
        fprintf(stderr, "XY GPS update failed: valid=%u east=%.2f\n",
                out.valid_xy, out.position_enu_cm[0]);
        return 1;
    }

    in.gps_valid = 0;
    in.gps_new = 0;
    in.time_us += 3000000ULL;
    BFNav_Update(&in, &out);
    if (out.valid_xy) {
        fprintf(stderr, "XY timeout failed\n");
        return 1;
    }

    printf("bfnav_test: OK\n");
    return 0;
}
