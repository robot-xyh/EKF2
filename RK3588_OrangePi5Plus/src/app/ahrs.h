/********************************************************************************
 * File  : ahrs.h
 * Brief : Mahony 9轴 互补滤波姿态解算 (飞控同款)。
 *         输入: 陀螺(rad/s) + 加速度(任意一致单位) + 磁力计(任意一致单位)
 *         输出: 四元数 + 欧拉角(roll/pitch/yaw, 度)
 ********************************************************************************/
#ifndef __AHRS_H
#define __AHRS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float q0, q1, q2, q3;       /* 四元数 */
    float roll, pitch, yaw;     /* 欧拉角(度), yaw: 0~360 */
} ahrs_t;

void AHRS_Init(ahrs_t *a);

/* 9轴更新(带磁力计 yaw 校正)。mx/my/mz 传 0 则退化为 6轴(yaw 仅靠陀螺积分)。
 * dt: 距上次更新的秒数。 */
void AHRS_Update(ahrs_t *a,
                 float gx, float gy, float gz,
                 float ax, float ay, float az,
                 float mx, float my, float mz,
                 float dt);

#ifdef __cplusplus
}
#endif

#endif /* __AHRS_H */
