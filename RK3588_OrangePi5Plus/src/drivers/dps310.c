/********************************************************************************
 * File  : dps310.c
 * Brief : 气压计 LQTP001HTA (DPS310/SPL07 协议族) I2C 驱动。
 *         校准系数读取 + 补偿公式与 CH32 版完全一致。
 *         参考 Betaflight: drivers/barometer/barometer_dps310.c
 ********************************************************************************/
#include "sensors.h"
#include "bsp.h"
#include "i2c.h"
#include "delay.h"

#define REG_PSR_B2      0x00    /* PSR_B2,B1,B0, TMP_B2,B1,B0 共6字节 */
#define REG_PRS_CFG     0x06
#define REG_TMP_CFG     0x07
#define REG_MEAS_CFG    0x08
#define REG_CFG_REG     0x09
#define REG_RESET       0x0C
#define REG_COEF        0x10
#define REG_COEF_SRCE   0x28

#define RESET_SOFT      0x09
#define MEAS_CTRL_CONT  0x07
#define PRS_CFG_VAL     0x54    /* 32Hz | 16x */
#define TMP_CFG_BASE    0x54
#define CFG_SHIFT_VAL   0x0C    /* T_SHIFT | P_SHIFT (16x 需要) */
#define COEF_SRCE_TMP   0x80

#define KT_KP           253952.0f   /* 16x 过采样缩放因子 */

static uint8_t baro_addr = DPS310_I2C_ADDR;
static uint8_t baro_id   = 0;

static struct {
    int32_t c0, c1;
    int32_t c00, c10, c01, c11, c20, c21, c30, c31, c40;
} calib;

static int32_t two_comp(uint32_t raw, uint8_t bits)
{
    if (raw & ((uint32_t)1 << (bits - 1))) {
        return (int32_t)raw - ((int32_t)1 << bits);
    }
    return (int32_t)raw;
}

void DPS310_Detect(sensor_result_t *r)
{
    const uint8_t addrs[2] = { DPS310_I2C_ADDR, DPS310_I2C_ADDR_ALT };
    uint8_t i, id;

    r->present = 0;
    r->id_ok   = 0;
    r->address = 0;
    r->id      = 0;

    for (i = 0; i < 2; i++) {
        if (!I2C_Probe(addrs[i])) {
            continue;
        }
        r->present = 1;
        r->address = addrs[i];

        if (I2C_ReadReg(addrs[i], DPS310_REG_ID, &id)) {
            r->id = id;
            if (id == DPS310_ID_VAL || id == SPL07_ID_VAL) {
                r->id_ok  = 1;
                baro_addr = addrs[i];
                baro_id   = id;
                return;
            }
        }
    }
}

void DPS310_Init(void)
{
    uint8_t coef[22];
    uint8_t len = (baro_id == SPL07_ID_VAL) ? 22 : 18;
    uint8_t i, srce, tmp_cfg;

    I2C_WriteReg(baro_addr, REG_RESET, RESET_SOFT);
    Delay_Ms(50);

    for (i = 0; i < len; ) {
        uint8_t chunk = ((len - i) > 9) ? 9 : (len - i);
        I2C_ReadRegs(baro_addr, REG_COEF + i, coef + i, chunk);
        i += chunk;
    }

    calib.c0  = two_comp(((uint32_t)coef[0] << 4) | ((coef[1] >> 4) & 0x0F), 12);
    calib.c1  = two_comp((((uint32_t)coef[1] & 0x0F) << 8) | coef[2], 12);
    calib.c00 = two_comp(((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | ((coef[5] >> 4) & 0x0F), 20);
    calib.c10 = two_comp((((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | coef[7], 20);
    calib.c01 = two_comp(((uint32_t)coef[8]  << 8) | coef[9],  16);
    calib.c11 = two_comp(((uint32_t)coef[10] << 8) | coef[11], 16);
    calib.c20 = two_comp(((uint32_t)coef[12] << 8) | coef[13], 16);
    calib.c21 = two_comp(((uint32_t)coef[14] << 8) | coef[15], 16);
    calib.c30 = two_comp(((uint32_t)coef[16] << 8) | coef[17], 16);

    if (baro_id == SPL07_ID_VAL) {
        calib.c31 = two_comp(((uint32_t)coef[18] << 4) | ((coef[19] >> 4) & 0x0F), 12);
        calib.c40 = two_comp((((uint32_t)coef[19] & 0x0F) << 8) | coef[20], 12);
    } else {
        calib.c31 = 0;
        calib.c40 = 0;
    }

    I2C_WriteReg(baro_addr, REG_PRS_CFG, PRS_CFG_VAL);

    if (baro_id == SPL07_ID_VAL) {
        tmp_cfg = TMP_CFG_BASE;
    } else {
        I2C_ReadReg(baro_addr, REG_COEF_SRCE, &srce);
        tmp_cfg = TMP_CFG_BASE | (srce & COEF_SRCE_TMP);
    }
    I2C_WriteReg(baro_addr, REG_TMP_CFG, tmp_cfg);

    I2C_WriteReg(baro_addr, REG_CFG_REG,  CFG_SHIFT_VAL);
    I2C_WriteReg(baro_addr, REG_MEAS_CFG, MEAS_CTRL_CONT);
    Delay_Ms(50);
}

uint8_t DPS310_Read(int32_t *pressure_pa, int32_t *temp_centi)
{
    uint8_t buf[6];
    int32_t Praw, Traw;
    float Praw_sc, Traw_sc, pressure, temperature;

    if (!I2C_ReadRegs(baro_addr, REG_PSR_B2, buf, 6)) {
        return 0;
    }

    Praw = two_comp(((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2], 24);
    Traw = two_comp(((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5], 24);

    Praw_sc = (float)Praw / KT_KP;
    Traw_sc = (float)Traw / KT_KP;

    pressure = (float)calib.c00
             + Praw_sc * ((float)calib.c10 + Praw_sc * ((float)calib.c20 + Praw_sc * (float)calib.c30))
             + Traw_sc * (float)calib.c01
             + Traw_sc * Praw_sc * ((float)calib.c11 + Praw_sc * (float)calib.c21);

    temperature = (float)calib.c0 * 0.5f + (float)calib.c1 * Traw_sc;

    if (pressure_pa) {
        *pressure_pa = (int32_t)pressure;
    }
    if (temp_centi) {
        *temp_centi = (int32_t)(temperature * 100.0f);
    }
    return 1;
}
