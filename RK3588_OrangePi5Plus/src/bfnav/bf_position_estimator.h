/*
 * Betaflight-style position estimator adapter for this repository.
 *
 * The Kalman structure and tuning constants are derived from Betaflight
 * GPLv3 code. See bf_position_filter.* for source attribution.
 */
#ifndef __BF_POSITION_ESTIMATOR_H
#define __BF_POSITION_ESTIMATOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t time_us;
    float dt_s;
    float accel_enu_cmss[3];      /* East, North, Up linear acceleration */

    uint8_t gps_valid;
    uint8_t gps_new;
    int32_t gps_lat_e7;
    int32_t gps_lon_e7;
    float gps_alt_cm;             /* MSL altitude */
    float gps_vel_enu_cms[3];     /* East, North, Up */
    uint16_t gps_pdop;            /* DOP * 100 */

    uint8_t baro_valid;
    float baro_alt_cm;            /* pressure altitude, absolute or relative */
} bfnav_input_t;

typedef struct {
    float position_enu_cm[3];
    float velocity_enu_cms[3];
    float trust_xy;
    float trust_z;
    uint8_t valid_xy;
    uint8_t valid_z;
    uint8_t gps_contributing;
    uint8_t origin_valid;
    int32_t origin_lat_e7;
    int32_t origin_lon_e7;
} bfnav_output_t;

void BFNav_Init(void);
void BFNav_ResetXY(void);
void BFNav_ResetZ(void);
void BFNav_Update(const bfnav_input_t *in, bfnav_output_t *out);
const bfnav_output_t *BFNav_GetOutput(void);

#ifdef __cplusplus
}
#endif

#endif /* __BF_POSITION_ESTIMATOR_H */
