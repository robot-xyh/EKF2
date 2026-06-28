/********************************************************************************
 * File  : sensor_check.c
 * Brief : 主流程 —— 识别 -> 陀螺零偏/地面气压校准 -> 实时刷新仪表盘:
 *           IMU 九轴原始数据 + Mahony 解算姿态角/航向 + 气压高度。
 *         AHRS 高速循环(尽量快), 显示按固定频率刷新, 像飞控地面站。
 ********************************************************************************/
#include "sensor_check.h"
#include "bsp.h"
#include "spi.h"
#include "sensors.h"
#include "ahrs.h"
#include "altitude.h"
#include "delay.h"

#include <stdio.h>
#include <signal.h>
#include <math.h>

#define PRINT_PERIOD_SEC    0.10        /* 仪表盘刷新 10Hz */
#define GYRO_CAL_SAMPLES    600         /* 陀螺零偏取样数(需静止) */
#define BARO_CAL_SAMPLES    40          /* 地面气压取样数 */

static volatile sig_atomic_t s_run = 1;

static void on_sigint(int sig)
{
    (void)sig;
    s_run = 0;
}

static void detect_print(const char *name, const sensor_result_t *r)
{
    if (r->id_ok) {
        printf("  [ OK ] %-16s addr=0x%02X  ID=0x%02X\n", name, r->address, r->id);
    } else if (r->present) {
        printf("  [ ?? ] %-16s addr=0x%02X  ID=0x%02X (ID mismatch)\n",
               name, r->address, r->id);
    } else {
        printf("  [FAIL] %-16s no response\n", name);
    }
}

int SensorCheck_Run(void)
{
    sensor_result_t imu_r, mag_r, baro_r;
    ahrs_t ahrs;
    float gyro_bias[3] = { 0, 0, 0 };

    signal(SIGINT,  on_sigint);
    signal(SIGTERM, on_sigint);

    /* -------- 识别(慢速 SPI) -------- */
    ICM40608_Detect(&imu_r);
    QMC5883P_Detect(&mag_r);
    DPS310_Detect(&baro_r);

    printf("\n============================================================\n");
    printf(" Orange Pi 5 Plus (RK3588) Sensor + AHRS\n");
    printf(" IMU : %s   MAG/BARO : %s\n", IMU_SPI_DEV, SENSOR_I2C_DEV);
    printf(" --- Detect ---\n");
    detect_print("IMU  ICM40608",   &imu_r);
    detect_print("MAG  QMC5883P",   &mag_r);
    detect_print("BARO LQTP001HTA", &baro_r);
    printf(" Detected: %d / 3\n",
           (imu_r.id_ok?1:0) + (mag_r.id_ok?1:0) + (baro_r.id_ok?1:0));
    printf("============================================================\n");

    if (!imu_r.id_ok) {
        printf(" 致命: IMU 未识别, 无法解算姿态。检查 SPI 接线/overlay。\n");
        return -1;
    }

    /* -------- 初始化 -------- */
    ICM40608_Init();                    /* 内部会切到高速 SPI */
    if (mag_r.id_ok)  QMC5883P_Init();
    if (baro_r.id_ok) DPS310_Init();
    AHRS_Init(&ahrs);

    /* -------- 陀螺零偏校准(请保持静止) -------- */
    printf("\n 校准陀螺零偏中, 请保持板子静止 ...\n");
    {
        long sum[3] = { 0, 0, 0 };
        for (int i = 0; i < GYRO_CAL_SAMPLES && s_run; i++) {
            imu_data_t d;
            ICM40608_Read(&d);
            sum[0] += d.gyro[0];
            sum[1] += d.gyro[1];
            sum[2] += d.gyro[2];
            Delay_Ms(2);
        }
        gyro_bias[0] = (float)sum[0] / GYRO_CAL_SAMPLES;
        gyro_bias[1] = (float)sum[1] / GYRO_CAL_SAMPLES;
        gyro_bias[2] = (float)sum[2] / GYRO_CAL_SAMPLES;
    }

    /* -------- 地面气压基准 -------- */
    if (baro_r.id_ok) {
        float psum = 0.0f; int n = 0;
        for (int i = 0; i < BARO_CAL_SAMPLES && s_run; i++) {
            int32_t p = 0, t = 0;
            if (DPS310_Read(&p, &t)) { psum += (float)p; n++; }
            Delay_Ms(20);
        }
        if (n > 0) {
            Altitude_SetGround(psum / n);
        }
    }

    /* -------- 实时循环 -------- */
    double t_prev   = Now_Sec();
    double t_print  = t_prev;
    unsigned long cnt = 0;
    float loop_hz = 0.0f;

    /* 显示用缓存(打印频率低于循环频率) */
    int   acc_mg[3] = {0,0,0};
    float gyr_dps[3] = {0,0,0};
    int16_t mag_raw[3] = {0,0,0};
    float imu_temp = 0.0f;
    int32_t baro_p = 0, baro_t = 0;

    while (s_run) {
        imu_data_t d;
        ICM40608_Read(&d);

        double t_now = Now_Sec();
        float dt = (float)(t_now - t_prev);
        t_prev = t_now;
        if (dt <= 0.0f || dt > 0.2f) dt = 0.005f;   /* 容错 */

        /* 单位换算 */
        float gx = ((float)d.gyro[0] - gyro_bias[0]) / GYRO_LSB_PER_DPS;
        float gy = ((float)d.gyro[1] - gyro_bias[1]) / GYRO_LSB_PER_DPS;
        float gz = ((float)d.gyro[2] - gyro_bias[2]) / GYRO_LSB_PER_DPS;
        float ax = (float)d.acc[0] / ACC_LSB_PER_G;
        float ay = (float)d.acc[1] / ACC_LSB_PER_G;
        float az = (float)d.acc[2] / ACC_LSB_PER_G;

        float mx = 0, my = 0, mz = 0;
        if (mag_r.id_ok) {
            int16_t m[3];
            if (QMC5883P_Read(m)) {
                mag_raw[0]=m[0]; mag_raw[1]=m[1]; mag_raw[2]=m[2];
                mx = (float)m[0]; my = (float)m[1]; mz = (float)m[2];
            }
        }

        AHRS_Update(&ahrs,
                    gx * DEG2RAD, gy * DEG2RAD, gz * DEG2RAD,
                    ax, ay, az,
                    mx, my, mz,
                    dt);

        cnt++;

        /* 缓存显示值 */
        acc_mg[0]=(int)(ax*1000); acc_mg[1]=(int)(ay*1000); acc_mg[2]=(int)(az*1000);
        gyr_dps[0]=gx; gyr_dps[1]=gy; gyr_dps[2]=gz;
        imu_temp = (float)d.temp / 132.48f + 25.0f;
        if (baro_r.id_ok) {
            DPS310_Read(&baro_p, &baro_t);
        }

        /* 按频率刷新仪表盘 */
        if ((t_now - t_print) >= PRINT_PERIOD_SEC) {
            float period = (float)(t_now - t_print);
            loop_hz = (period > 0) ? (cnt / period) : 0.0f;
            cnt = 0;
            t_print = t_now;

            float press_hpa = baro_p / 100.0f;
            float alt_abs = baro_r.id_ok ? Altitude_FromPressure((float)baro_p) : 0.0f;
            float alt_rel = baro_r.id_ok ? Altitude_Relative((float)baro_p)     : 0.0f;

            printf("\033[2J\033[H");      /* 清屏 + 光标归位 */
            printf(" ====== Orange Pi 5 Plus  Sensor / AHRS Dashboard ======\n");
            printf("  loop: %6.0f Hz\n", loop_hz);
            printf("  ---------------- IMU  ICM40608 ----------------\n");
            printf("   ACC [mg ] : %+6d %+6d %+6d\n", acc_mg[0], acc_mg[1], acc_mg[2]);
            printf("   GYR [dps] : %+7.2f %+7.2f %+7.2f\n", gyr_dps[0], gyr_dps[1], gyr_dps[2]);
            printf("   TEMP      : %6.2f C\n", imu_temp);
            printf("  ---------------- MAG  QMC5883P ----------------\n");
            if (mag_r.id_ok)
                printf("   MAG [raw] : %+6d %+6d %+6d\n", mag_raw[0], mag_raw[1], mag_raw[2]);
            else
                printf("   MAG       : n/a\n");
            printf("  ---------------- BARO DPS310  -----------------\n");
            if (baro_r.id_ok) {
                printf("   PRESS     : %9.2f hPa\n", press_hpa);
                printf("   TEMP      : %6.2f C\n", baro_t / 100.0f);
                printf("   ALT (abs) : %8.2f m\n", alt_abs);
                printf("   ALT (rel) : %+8.2f m\n", alt_rel);
            } else {
                printf("   BARO      : n/a\n");
            }
            printf("  ========= ATTITUDE  (Mahony 9-axis) ===========\n");
            printf("   ROLL      : %+8.2f deg\n", ahrs.roll);
            printf("   PITCH     : %+8.2f deg\n", ahrs.pitch);
            printf("   YAW/HEAD  : %8.2f deg\n", ahrs.yaw);
            printf("   QUAT      : %+.3f %+.3f %+.3f %+.3f\n",
                   ahrs.q0, ahrs.q1, ahrs.q2, ahrs.q3);
            printf(" =======================================================\n");
            printf("  Ctrl-C 退出\n");
            fflush(stdout);
        }
    }

    printf("\n 已停止。\n");
    return 0;
}
