/*
 * This file is derived from Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Source baseline:
 * https://github.com/betaflight/betaflight
 * commit 25a7f7417164d88c0db833db00af2066857e1258
 */
#include "bf_position_filter.h"

void BF_KalmanInit(bf_position_kalman_t *kf, float initial_pos, float initial_vel,
                   float initial_pos_var, float initial_vel_var, float q_accel)
{
    kf->x[0] = initial_pos;
    kf->x[1] = initial_vel;
    kf->P[0][0] = initial_pos_var;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = initial_vel_var;
    kf->Q_accel = q_accel;
}

void BF_KalmanPredict(bf_position_kalman_t *kf, float dt, float accel)
{
    const float dt2 = dt * dt;
    const float half_dt2 = 0.5f * dt2;

    kf->x[0] += kf->x[1] * dt + half_dt2 * accel;
    kf->x[1] += dt * accel;

    const float q = kf->Q_accel;
    const float p00 = kf->P[0][0];
    const float p01 = kf->P[0][1];
    const float p10 = kf->P[1][0];
    const float p11 = kf->P[1][1];

    kf->P[0][0] = p00 + dt * (p10 + p01) + dt2 * p11 + 0.25f * dt2 * dt2 * q;
    kf->P[0][1] = p01 + dt * p11 + half_dt2 * dt * q;
    kf->P[1][0] = p10 + dt * p11 + half_dt2 * dt * q;
    kf->P[1][1] = p11 + dt2 * q;
}

void BF_KalmanUpdatePosition(bf_position_kalman_t *kf, float measured_pos, float r)
{
    const float p00 = kf->P[0][0];
    const float p10 = kf->P[1][0];
    const float s = p00 + r;
    if (s < 1e-9f) {
        return;
    }

    const float s_inv = 1.0f / s;
    const float k0 = p00 * s_inv;
    const float k1 = p10 * s_inv;
    const float y = measured_pos - kf->x[0];

    kf->x[0] += k0 * y;
    kf->x[1] += k1 * y;

    const float i_k0 = 1.0f - k0;
    const float p01 = kf->P[0][1];
    const float p11 = kf->P[1][1];

    kf->P[0][0] = i_k0 * p00;
    kf->P[0][1] = i_k0 * p01;
    kf->P[1][0] = p10 - k1 * p00;
    kf->P[1][1] = p11 - k1 * p01;
}

void BF_KalmanUpdateVelocity(bf_position_kalman_t *kf, float measured_vel, float r)
{
    const float p01 = kf->P[0][1];
    const float p11 = kf->P[1][1];
    const float s = p11 + r;
    if (s < 1e-9f) {
        return;
    }

    const float s_inv = 1.0f / s;
    const float k0 = p01 * s_inv;
    const float k1 = p11 * s_inv;
    const float y = measured_vel - kf->x[1];

    kf->x[0] += k0 * y;
    kf->x[1] += k1 * y;

    const float p00 = kf->P[0][0];
    const float p10 = kf->P[1][0];
    const float i_k1 = 1.0f - k1;

    kf->P[0][0] = p00 - k0 * p10;
    kf->P[0][1] = p01 - k0 * p11;
    kf->P[1][0] = i_k1 * p10;
    kf->P[1][1] = i_k1 * p11;
}
