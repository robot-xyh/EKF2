/*
 * Betaflight-style combined navigation dashboard.
 *
 * Uses Betaflight-derived GPLv3 position estimator logic for educational and
 * research use in this repository.
 */
#include "bf_position_estimator.h"

#include "ahrs.h"
#include "altitude.h"
#include "bsp.h"
#include "delay.h"
#include "sensors.h"
#include "uart.h"
#include "ubx_gps.h"

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>

#define PRINT_PERIOD_SEC 0.10
#define GYRO_CAL_SAMPLES 600
#define BARO_CAL_SAMPLES 40

static volatile sig_atomic_t s_run = 1;

static void on_signal(int sig)
{
    (void)sig;
    s_run = 0;
}

static float pick_axis(const float v[3], int src, float sign)
{
    if (src < 0 || src > 2) {
        return 0.0f;
    }
    return sign * v[src];
}

static void map_imu_axes(const float in[3], float out[3])
{
    out[0] = pick_axis(in, EKF_IMU_X_SRC, EKF_IMU_X_SIGN);
    out[1] = pick_axis(in, EKF_IMU_Y_SRC, EKF_IMU_Y_SIGN);
    out[2] = pick_axis(in, EKF_IMU_Z_SRC, EKF_IMU_Z_SIGN);
}

static void map_mag_axes(const float in[3], float out[3])
{
    out[0] = pick_axis(in, EKF_MAG_X_SRC, EKF_MAG_X_SIGN);
    out[1] = pick_axis(in, EKF_MAG_Y_SRC, EKF_MAG_Y_SIGN);
    out[2] = pick_axis(in, EKF_MAG_Z_SRC, EKF_MAG_Z_SIGN);
}

static void rotate_body_to_earth(const ahrs_t *a, const float body[3], float earth[3])
{
    const float q0 = a->q0;
    const float q1 = a->q1;
    const float q2 = a->q2;
    const float q3 = a->q3;

    const float r00 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    const float r01 = 2.0f * (q1 * q2 - q0 * q3);
    const float r02 = 2.0f * (q1 * q3 + q0 * q2);
    const float r10 = 2.0f * (q1 * q2 + q0 * q3);
    const float r11 = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    const float r12 = 2.0f * (q2 * q3 - q0 * q1);
    const float r20 = 2.0f * (q1 * q3 - q0 * q2);
    const float r21 = 2.0f * (q2 * q3 + q0 * q1);
    const float r22 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

    earth[0] = r00 * body[0] + r01 * body[1] + r02 * body[2];
    earth[1] = r10 * body[0] + r11 * body[1] + r12 * body[2];
    earth[2] = r20 * body[0] + r21 * body[1] + r22 * body[2];
}

static void ubx_send(int fd, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    if (fd < 0) {
        return;
    }

    uint8_t frame[128];
    if ((size_t)len + 8U > sizeof(frame)) {
        return;
    }

    frame[0] = 0xB5;
    frame[1] = 0x62;
    frame[2] = cls;
    frame[3] = id;
    frame[4] = (uint8_t)(len & 0xFF);
    frame[5] = (uint8_t)(len >> 8);
    if (len > 0) {
        memcpy(&frame[6], payload, len);
    }

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (uint16_t i = 2; i < (uint16_t)(6 + len); i++) {
        ck_a = (uint8_t)(ck_a + frame[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
    }
    frame[6 + len] = ck_a;
    frame[7 + len] = ck_b;
    UART_Write(fd, frame, (size_t)len + 8U);
}

static void gps_configure_ubx(int fd)
{
    const uint8_t cfg_rate[] = { 100, 0, 1, 0, 1, 0 };
    const uint8_t cfg_nav_pvt[] = { 0x01, 0x07, 0, 1, 0, 1, 0, 0 };
    ubx_send(fd, 0x06, 0x08, cfg_rate, sizeof(cfg_rate));
    ubx_send(fd, 0x06, 0x01, cfg_nav_pvt, sizeof(cfg_nav_pvt));
}

static int gps_poll(int fd, ubx_parser_t *parser, ubx_nav_pvt_t *fix)
{
    if (fd < 0) {
        return 0;
    }

    uint8_t buf[256];
    int got = 0;
    for (;;) {
        ssize_t n = UART_Read(fd, buf, sizeof(buf));
        if (n <= 0) {
            break;
        }
        if (UBX_ParseBuffer(parser, buf, (size_t)n, fix) > 0) {
            got = 1;
        }
        if (n < (ssize_t)sizeof(buf)) {
            break;
        }
    }
    return got;
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

int main(void)
{
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    if (BSP_Init() != 0) {
        fprintf(stderr, "BSP_Init failed: check SPI/I2C nodes and permissions.\n");
        return 1;
    }

    int gps_fd = UART_Open(GPS_UART_DEV, GPS_UART_BAUD);
    if (gps_fd >= 0) {
        gps_configure_ubx(gps_fd);
    }

    sensor_result_t imu_r, mag_r, baro_r;
    ICM40608_Detect(&imu_r);
    QMC5883P_Detect(&mag_r);
    DPS310_Detect(&baro_r);

    printf("\n============================================================\n");
    printf(" Orange Pi 5 Plus Sensor + Betaflight-style Nav\n");
    printf(" IMU: %s   MAG/BARO: %s   GPS: %s @ %d\n",
           IMU_SPI_DEV, SENSOR_I2C_DEV, GPS_UART_DEV, GPS_UART_BAUD);
    printf(" --- Detect ---\n");
    detect_print("IMU  ICM40608", &imu_r);
    detect_print("MAG  QMC5883P", &mag_r);
    detect_print("BARO DPS310", &baro_r);
    printf("  [%s] GPS UART\n", gps_fd >= 0 ? " OK " : "FAIL");
    printf("============================================================\n");

    if (!imu_r.id_ok) {
        fprintf(stderr, "Fatal: IMU not detected, BFNav cannot run.\n");
        UART_Close(gps_fd);
        BSP_Deinit();
        return 1;
    }

    ICM40608_Init();
    if (mag_r.id_ok) {
        QMC5883P_Init();
    }
    if (baro_r.id_ok) {
        DPS310_Init();
    }

    ahrs_t ahrs;
    AHRS_Init(&ahrs);
    BFNav_Init();

    printf("\nCalibrating gyro bias, keep the board still ...\n");
    float gyro_bias[3] = { 0.0f, 0.0f, 0.0f };
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

    if (baro_r.id_ok) {
        float psum = 0.0f;
        int n = 0;
        for (int i = 0; i < BARO_CAL_SAMPLES && s_run; i++) {
            int32_t p = 0, t = 0;
            if (DPS310_Read(&p, &t)) {
                psum += (float)p;
                n++;
            }
            Delay_Ms(20);
        }
        if (n > 0) {
            Altitude_SetGround(psum / n);
        }
    }

    ubx_parser_t gps_parser;
    ubx_nav_pvt_t gps_fix;
    UBX_Init(&gps_parser);
    memset(&gps_fix, 0, sizeof(gps_fix));
    uint8_t gps_seen = 0;
    uint8_t gps_new = 0;
    uint64_t last_gps_us = 0;

    bfnav_output_t nav_out;
    memset(&nav_out, 0, sizeof(nav_out));

    double t_prev = Now_Sec();
    double t_print = t_prev;
    unsigned long cnt = 0;
    float loop_hz = 0.0f;

    int acc_mg[3] = {0, 0, 0};
    float gyr_dps[3] = {0, 0, 0};
    int16_t mag_raw[3] = {0, 0, 0};
    int32_t baro_p = 0, baro_t = 0;
    float imu_temp = 0.0f;

    while (s_run) {
        imu_data_t d;
        ICM40608_Read(&d);

        double t_now = Now_Sec();
        float dt = (float)(t_now - t_prev);
        t_prev = t_now;
        if (dt <= 0.0f || dt > 0.2f) {
            dt = 0.005f;
        }
        uint64_t ts = (uint64_t)(t_now * 1000000.0);

        float gyro_sensor_dps[3] = {
            ((float)d.gyro[0] - gyro_bias[0]) / GYRO_LSB_PER_DPS,
            ((float)d.gyro[1] - gyro_bias[1]) / GYRO_LSB_PER_DPS,
            ((float)d.gyro[2] - gyro_bias[2]) / GYRO_LSB_PER_DPS
        };
        float acc_sensor_g[3] = {
            (float)d.acc[0] / ACC_LSB_PER_G,
            (float)d.acc[1] / ACC_LSB_PER_G,
            (float)d.acc[2] / ACC_LSB_PER_G
        };
        float gyro_body_dps[3];
        float acc_body_g[3];
        map_imu_axes(gyro_sensor_dps, gyro_body_dps);
        map_imu_axes(acc_sensor_g, acc_body_g);

        float mag_body_raw[3] = { 0.0f, 0.0f, 0.0f };
        if (mag_r.id_ok) {
            int16_t m[3];
            if (QMC5883P_Read(m)) {
                float mag_sensor[3] = { (float)m[0], (float)m[1], (float)m[2] };
                mag_raw[0] = m[0];
                mag_raw[1] = m[1];
                mag_raw[2] = m[2];
                map_mag_axes(mag_sensor, mag_body_raw);
            }
        }

        const float gyro_body_rad_s[3] = {
            gyro_body_dps[0] * DEG2RAD,
            gyro_body_dps[1] * DEG2RAD,
            gyro_body_dps[2] * DEG2RAD
        };
        AHRS_Update(&ahrs,
                    gyro_body_rad_s[0], gyro_body_rad_s[1], gyro_body_rad_s[2],
                    acc_body_g[0], acc_body_g[1], acc_body_g[2],
                    mag_body_raw[0], mag_body_raw[1], mag_body_raw[2],
                    dt);

        float acc_earth_g[3];
        rotate_body_to_earth(&ahrs, acc_body_g, acc_earth_g);

        bfnav_input_t nav_in;
        memset(&nav_in, 0, sizeof(nav_in));
        nav_in.time_us = ts;
        nav_in.dt_s = dt;
        nav_in.accel_enu_cmss[0] = acc_earth_g[0] * GRAVITY_MSS * 100.0f;
        nav_in.accel_enu_cmss[1] = acc_earth_g[1] * GRAVITY_MSS * 100.0f;
        nav_in.accel_enu_cmss[2] = (acc_earth_g[2] - 1.0f) * GRAVITY_MSS * 100.0f;

        if (baro_r.id_ok && DPS310_Read(&baro_p, &baro_t)) {
            nav_in.baro_valid = 1;
            nav_in.baro_alt_cm = Altitude_FromPressure((float)baro_p) * 100.0f;
        }

        gps_new = 0;
        if (gps_poll(gps_fd, &gps_parser, &gps_fix)) {
            gps_seen = 1;
            gps_new = 1;
            last_gps_us = ts;
        }
        if (gps_seen && gps_fix.valid_3d) {
            nav_in.gps_valid = 1;
            nav_in.gps_new = gps_new;
            nav_in.gps_lat_e7 = gps_fix.lat_e7;
            nav_in.gps_lon_e7 = gps_fix.lon_e7;
            nav_in.gps_alt_cm = gps_fix.h_msl_mm * 0.1f;
            nav_in.gps_vel_enu_cms[0] = gps_fix.vel_e_mm_s * 0.1f;
            nav_in.gps_vel_enu_cms[1] = gps_fix.vel_n_mm_s * 0.1f;
            nav_in.gps_vel_enu_cms[2] = -gps_fix.vel_d_mm_s * 0.1f;
            nav_in.gps_pdop = gps_fix.p_dop;
        }

        BFNav_Update(&nav_in, &nav_out);

        acc_mg[0] = (int)(acc_body_g[0] * 1000.0f);
        acc_mg[1] = (int)(acc_body_g[1] * 1000.0f);
        acc_mg[2] = (int)(acc_body_g[2] * 1000.0f);
        gyr_dps[0] = gyro_body_dps[0];
        gyr_dps[1] = gyro_body_dps[1];
        gyr_dps[2] = gyro_body_dps[2];
        imu_temp = (float)d.temp / 132.48f + 25.0f;
        cnt++;

        if ((t_now - t_print) >= PRINT_PERIOD_SEC) {
            const float period = (float)(t_now - t_print);
            loop_hz = (period > 0.0f) ? (cnt / period) : 0.0f;
            cnt = 0;
            t_print = t_now;

            const float alt_abs = baro_r.id_ok ? Altitude_FromPressure((float)baro_p) : 0.0f;
            const float alt_rel = baro_r.id_ok ? Altitude_Relative((float)baro_p) : 0.0f;
            const float gps_age = (gps_seen && last_gps_us > 0) ? ((ts - last_gps_us) * 1e-6f) : -1.0f;

            printf("\033[2J\033[H");
            printf(" ===== Orange Pi 5 Plus Sensor / Betaflight Nav =====\n");
            printf("  loop: %6.0f Hz\n", loop_hz);
            printf("  ---------------- IMU ICM40608 ---------------------\n");
            printf("   ACC [mg]  : %+6d %+6d %+6d\n", acc_mg[0], acc_mg[1], acc_mg[2]);
            printf("   GYR [dps] : %+7.2f %+7.2f %+7.2f\n", gyr_dps[0], gyr_dps[1], gyr_dps[2]);
            printf("   TEMP      : %6.2f C\n", imu_temp);
            printf("  ---------------- GPS UBX NAV-PVT ------------------\n");
            if (gps_seen) {
                printf("   FIX/SATS  : %u / %u   age: %.2f s\n", gps_fix.fix_type, gps_fix.num_sv, gps_age);
                printf("   LLH       : %.7f %.7f %.2f m\n",
                       gps_fix.lat_e7 * 1e-7, gps_fix.lon_e7 * 1e-7, gps_fix.h_msl_mm * 0.001f);
                printf("   PDOP/HACC : %.2f / %.2f m\n", gps_fix.p_dop * 0.01f, gps_fix.h_acc_mm * 0.001f);
            } else {
                printf("   GPS       : waiting for UBX NAV-PVT\n");
            }
            printf("  ---------------- MAG / BARO -----------------------\n");
            if (mag_r.id_ok)
                printf("   MAG [raw] : %+6d %+6d %+6d\n", mag_raw[0], mag_raw[1], mag_raw[2]);
            else
                printf("   MAG       : n/a\n");
            if (baro_r.id_ok) {
                printf("   PRESS     : %9.2f hPa   TEMP: %.2f C\n", baro_p / 100.0f, baro_t / 100.0f);
                printf("   ALT abs/rel: %8.2f / %+8.2f m\n", alt_abs, alt_rel);
            } else {
                printf("   BARO      : n/a\n");
            }
            printf("  ---------------- BF NAV ESTIMATE ------------------\n");
            printf("   POS ENU   : %+8.2f %+8.2f %+8.2f m\n",
                   nav_out.position_enu_cm[0] * 0.01f,
                   nav_out.position_enu_cm[1] * 0.01f,
                   nav_out.position_enu_cm[2] * 0.01f);
            printf("   VEL ENU   : %+8.2f %+8.2f %+8.2f m/s\n",
                   nav_out.velocity_enu_cms[0] * 0.01f,
                   nav_out.velocity_enu_cms[1] * 0.01f,
                   nav_out.velocity_enu_cms[2] * 0.01f);
            printf("   VALID/TRUST: XY=%u %.2f   Z=%u %.2f   GPS=%u ORG=%u\n",
                   nav_out.valid_xy, nav_out.trust_xy,
                   nav_out.valid_z, nav_out.trust_z,
                   nav_out.gps_contributing, nav_out.origin_valid);
            printf("  ---------------- Mahony Attitude ------------------\n");
            printf("   RPY [deg] : %+8.2f %+8.2f %8.2f\n", ahrs.roll, ahrs.pitch, ahrs.yaw);
            printf(" ====================================================\n");
            printf("  Ctrl-C exit\n");
            fflush(stdout);
        }
    }

    printf("\nStopped.\n");
    UART_Close(gps_fd);
    BSP_Deinit();
    return 0;
}
