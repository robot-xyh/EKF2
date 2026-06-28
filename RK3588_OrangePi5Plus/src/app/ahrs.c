/********************************************************************************
 * File  : ahrs.c
 * Brief : Mahony AHRS (9轴) 实现。源自经典 Madgwick/Mahony 开源实现,
 *         与 Betaflight/Cleanflight 的互补滤波思路一致。
 ********************************************************************************/
#include "ahrs.h"
#include "bsp.h"     /* RAD2DEG */
#include <math.h>

/* 互补滤波增益: Kp 越大越信任加速度/磁力计(收敛快但抖), Ki 修正陀螺零偏 */
#define TWO_KP      (2.0f * 0.5f)
#define TWO_KI      (2.0f * 0.005f)

static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

static float inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

void AHRS_Init(ahrs_t *a)
{
    a->q0 = 1.0f; a->q1 = 0.0f; a->q2 = 0.0f; a->q3 = 0.0f;
    a->roll = a->pitch = a->yaw = 0.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
}

void AHRS_Update(ahrs_t *a,
                 float gx, float gy, float gz,
                 float ax, float ay, float az,
                 float mx, float my, float mz,
                 float dt)
{
    float q0 = a->q0, q1 = a->q1, q2 = a->q2, q3 = a->q3;
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

    int useMag = !((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f));

    /* 仅当加速度有效时做校正 */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        if (useMag) {
            float hx, hy, bx, bz;
            float halfwx, halfwy, halfwz;
            float q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3;
            float q1q1 = q1*q1, q1q2 = q1*q2, q1q3 = q1*q3;
            float q2q2 = q2*q2, q2q3 = q2*q3, q3q3 = q3*q3;

            recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

            /* 把磁向量旋到参考系, 估计地磁水平分量 */
            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            bx = sqrtf(hx * hx + hy * hy);
            bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

            halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

            halfex += (my * halfwz - mz * halfwy);
            halfey += (mz * halfwx - mx * halfwz);
            halfez += (mx * halfwy - my * halfwx);
        }

        /* 重力方向估计 */
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        halfex += (ay * halfvz - az * halfvy);
        halfey += (az * halfvx - ax * halfvz);
        halfez += (ax * halfvy - ay * halfvx);
    }

    /* 误差 -> PI 修正陀螺 */
    if (halfex != 0.0f || halfey != 0.0f || halfez != 0.0f) {
        if (TWO_KI > 0.0f) {
            integralFBx += TWO_KI * halfex * dt;
            integralFBy += TWO_KI * halfey * dt;
            integralFBz += TWO_KI * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        gx += TWO_KP * halfex;
        gy += TWO_KP * halfey;
        gz += TWO_KP * halfez;
    }

    /* 四元数积分 */
    gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    recipNorm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

    a->q0 = q0; a->q1 = q1; a->q2 = q2; a->q3 = q3;

    /* 四元数 -> 欧拉角 */
    a->roll  = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * RAD2DEG;
    float sinp = 2.0f * (q0*q2 - q3*q1);
    if (sinp > 1.0f)  sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    a->pitch = asinf(sinp) * RAD2DEG;
    a->yaw   = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * RAD2DEG;
    if (a->yaw < 0.0f) {
        a->yaw += 360.0f;
    }
}
