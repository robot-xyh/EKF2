/********************************************************************************
 * File  : ekf2_nav.cpp
 * Brief : Thin C wrapper around PX4 ECL EKF.
 ********************************************************************************/
#include "ekf2_nav.h"

#include "bsp.h"

#include <EKF/ekf.h>
#include <matrix/math.hpp>

#include <cmath>
#include <cstring>

using estimator::baroSample;
using estimator::gps_message;
using estimator::imuSample;
using estimator::magSample;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

static Ekf s_ekf;
static bool s_configured = false;

static Vector3f vec3_from_array(const float v[3])
{
    return Vector3f(v[0], v[1], v[2]);
}

static void copy_vec3(float out[3], const Vector3f &v)
{
    out[0] = v(0);
    out[1] = v(1);
    out[2] = v(2);
}

void EKF2Nav_Init(void)
{
    parameters *p = s_ekf.getParamHandle();
    p->fusion_mode = MASK_USE_GPS;
    p->vdist_sensor_type = VDIST_SENSOR_BARO;
    p->mag_fusion_type = MAG_FUSE_TYPE_AUTO;
    p->gps_pos_body = Vector3f(EKF_GPS_POS_X_M, EKF_GPS_POS_Y_M, EKF_GPS_POS_Z_M);
    p->imu_pos_body.zero();
    p->mag_delay_ms = 0.0f;
    p->baro_delay_ms = 0.0f;
    p->gps_delay_ms = 110.0f;
    s_ekf.set_is_fixed_wing(false);
    s_configured = true;
}

void EKF2Nav_SetIMU(uint64_t time_us, const float delta_ang_rad[3],
                    const float delta_vel_m_s[3], float dt_s)
{
    if (!s_configured) {
        EKF2Nav_Init();
    }

    imuSample sample {};
    sample.time_us = time_us;
    sample.delta_ang = vec3_from_array(delta_ang_rad);
    sample.delta_vel = vec3_from_array(delta_vel_m_s);
    sample.delta_ang_dt = dt_s;
    sample.delta_vel_dt = dt_s;
    sample.delta_vel_clipping[0] = false;
    sample.delta_vel_clipping[1] = false;
    sample.delta_vel_clipping[2] = false;

    s_ekf.setIMUData(sample);
    s_ekf.set_in_air_status(true);
}

void EKF2Nav_SetMag(uint64_t time_us, const float mag_gauss[3])
{
    magSample sample {};
    sample.time_us = time_us;
    sample.mag = vec3_from_array(mag_gauss);
    s_ekf.setMagData(sample);
}

void EKF2Nav_SetBaro(uint64_t time_us, float pressure_alt_m)
{
    baroSample sample {};
    sample.time_us = time_us;
    sample.hgt = pressure_alt_m;
    s_ekf.setBaroData(sample);
}

void EKF2Nav_SetGps(uint64_t time_us, const ekf2_gps_data_t *gps)
{
    if (gps == nullptr) {
        return;
    }

    gps_message sample {};
    sample.time_usec = time_us;
    sample.lat = gps->lat_e7;
    sample.lon = gps->lon_e7;
    sample.alt = gps->alt_msl_mm;
    sample.yaw = NAN;
    sample.yaw_offset = NAN;
    sample.fix_type = gps->fix_type;
    sample.eph = gps->hacc_m;
    sample.epv = gps->vacc_m;
    sample.sacc = gps->sacc_m_s;
    sample.vel_m_s = std::sqrt(gps->vel_n_m_s * gps->vel_n_m_s +
                               gps->vel_e_m_s * gps->vel_e_m_s);
    sample.vel_ned = Vector3f(gps->vel_n_m_s, gps->vel_e_m_s, gps->vel_d_m_s);
    sample.vel_ned_valid = gps->vel_ned_valid;
    sample.nsats = gps->nsats;
    sample.pdop = gps->pdop;
    s_ekf.setGpsData(sample);
}

int EKF2Nav_Update(ekf2_nav_output_t *out)
{
    bool updated = s_ekf.update();

    if (out != nullptr) {
        std::memset(out, 0, sizeof(*out));

        const Quatf &q = s_ekf.getQuaternion();
        out->quat[0] = q(0);
        out->quat[1] = q(1);
        out->quat[2] = q(2);
        out->quat[3] = q(3);

        Eulerf euler(q);
        out->euler_deg[0] = euler(0) * RAD2DEG;
        out->euler_deg[1] = euler(1) * RAD2DEG;
        out->euler_deg[2] = euler(2) * RAD2DEG;
        if (out->euler_deg[2] < 0.0f) {
            out->euler_deg[2] += 360.0f;
        }

        copy_vec3(out->pos_ned_m, s_ekf.getPosition());
        copy_vec3(out->vel_ned_m_s, s_ekf.getVelocity());
        copy_vec3(out->gyro_bias_rad_s, s_ekf.getGyroBias());
        copy_vec3(out->accel_bias_m_s2, s_ekf.getAccelBias());

        out->attitude_valid = s_ekf.attitude_valid() ? 1 : 0;
        out->local_position_valid = (s_ekf.local_position_is_valid() &&
                                     s_ekf.isHorizontalAidingActive()) ? 1 : 0;
        out->global_position_valid = s_ekf.global_position_is_valid() ? 1 : 0;
        out->tilt_align = s_ekf.control_status_flags().tilt_align ? 1 : 0;
        out->yaw_align = s_ekf.control_status_flags().yaw_align ? 1 : 0;
        out->gps_fusing = s_ekf.control_status_flags().gps ? 1 : 0;
        out->baro_fusing = s_ekf.control_status_flags().baro_hgt ? 1 : 0;
        out->mag_fusing = (s_ekf.control_status_flags().mag_hdg ||
                           s_ekf.control_status_flags().mag_3D) ? 1 : 0;
        out->control_status = s_ekf.control_status().value;
        out->fault_status = s_ekf.fault_status().value;
        out->innovation_fail_status = s_ekf.innov_check_fail_status().value;
        s_ekf.get_gps_check_status(&out->gps_check_fail);
    }

    return updated ? 1 : 0;
}
