/********************************************************************************
 * File  : ekf2_nav.h
 * Brief : C API around the PX4 ECL EKF.
 ********************************************************************************/
#ifndef __EKF2_NAV_H
#define __EKF2_NAV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int32_t lat_e7;
    int32_t lon_e7;
    int32_t alt_msl_mm;
    float vel_n_m_s;
    float vel_e_m_s;
    float vel_d_m_s;
    float hacc_m;
    float vacc_m;
    float sacc_m_s;
    float pdop;
    uint8_t fix_type;
    uint8_t nsats;
    uint8_t vel_ned_valid;
} ekf2_gps_data_t;

typedef struct {
    float quat[4];
    float euler_deg[3];
    float pos_ned_m[3];
    float vel_ned_m_s[3];
    float gyro_bias_rad_s[3];
    float accel_bias_m_s2[3];
    uint8_t attitude_valid;
    uint8_t local_position_valid;
    uint8_t global_position_valid;
    uint8_t tilt_align;
    uint8_t yaw_align;
    uint8_t gps_fusing;
    uint8_t baro_fusing;
    uint8_t mag_fusing;
    uint16_t gps_check_fail;
    uint32_t control_status;
    uint32_t fault_status;
    uint16_t innovation_fail_status;
} ekf2_nav_output_t;

void EKF2Nav_Init(void);
void EKF2Nav_SetIMU(uint64_t time_us, const float delta_ang_rad[3],
                    const float delta_vel_m_s[3], float dt_s);
void EKF2Nav_SetMag(uint64_t time_us, const float mag_gauss[3]);
void EKF2Nav_SetBaro(uint64_t time_us, float pressure_alt_m);
void EKF2Nav_SetGps(uint64_t time_us, const ekf2_gps_data_t *gps);
int  EKF2Nav_Update(ekf2_nav_output_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __EKF2_NAV_H */
