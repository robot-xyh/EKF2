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
#ifndef __BF_POSITION_FILTER_H
#define __BF_POSITION_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x[2];       /* [0]=position cm, [1]=velocity cm/s */
    float P[2][2];    /* covariance */
    float Q_accel;    /* acceleration process noise: (cm/s^2)^2 */
} bf_position_kalman_t;

void BF_KalmanInit(bf_position_kalman_t *kf, float initial_pos, float initial_vel,
                   float initial_pos_var, float initial_vel_var, float q_accel);
void BF_KalmanPredict(bf_position_kalman_t *kf, float dt, float accel);
void BF_KalmanUpdatePosition(bf_position_kalman_t *kf, float measured_pos, float r);
void BF_KalmanUpdateVelocity(bf_position_kalman_t *kf, float measured_vel, float r);

static inline float BF_KalmanGetPosition(const bf_position_kalman_t *kf) { return kf->x[0]; }
static inline float BF_KalmanGetVelocity(const bf_position_kalman_t *kf) { return kf->x[1]; }
static inline float BF_KalmanGetPositionVariance(const bf_position_kalman_t *kf) { return kf->P[0][0]; }
static inline float BF_KalmanGetVelocityVariance(const bf_position_kalman_t *kf) { return kf->P[1][1]; }

#ifdef __cplusplus
}
#endif

#endif /* __BF_POSITION_FILTER_H */
