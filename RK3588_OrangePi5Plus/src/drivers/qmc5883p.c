/********************************************************************************
 * File  : qmc5883p.c
 * Brief : 磁力计 QMC5883P I2C 驱动 (逻辑同 CH32 版, 换 /dev/i2c-5)。
 *         参考 Betaflight: drivers/compass/compass_qmc5883.c
 ********************************************************************************/
#include "sensors.h"
#include "bsp.h"
#include "i2c.h"
#include "delay.h"

#define QMC5883P_REG_DATA       0x01    /* X_L,X_H,Y_L,Y_H,Z_L,Z_H */
#define QMC5883P_REG_CONF1      0x0A
#define QMC5883P_REG_CONF2      0x0B
#define QMC5883P_REG_SIGN       0x29
#define QMC5883P_SIGN_VAL       0x06
#define QMC5883P_CONF1_VAL      0x1B    /* 连续 | 100Hz | 8G | OSR1=8 */
#define QMC5883P_CONF2_VAL      0x08    /* OSR2=8 */

void QMC5883P_Detect(sensor_result_t *r)
{
    r->present = 0;
    r->id_ok   = 0;
    r->address = QMC5883P_I2C_ADDR;
    r->id      = 0;

    if (!I2C_Probe(QMC5883P_I2C_ADDR)) {
        return;
    }
    r->present = 1;

    uint8_t id;
    if (I2C_ReadReg(QMC5883P_I2C_ADDR, QMC5883P_REG_ID, &id)) {
        r->id = id;
        if (id == QMC5883P_ID_VAL) {
            r->id_ok = 1;
        }
    }
}

void QMC5883P_Init(void)
{
    I2C_WriteReg(QMC5883P_I2C_ADDR, QMC5883P_REG_SIGN,  QMC5883P_SIGN_VAL);
    I2C_WriteReg(QMC5883P_I2C_ADDR, QMC5883P_REG_CONF1, QMC5883P_CONF1_VAL);
    I2C_WriteReg(QMC5883P_I2C_ADDR, QMC5883P_REG_CONF2, QMC5883P_CONF2_VAL);
    Delay_Ms(10);
}

uint8_t QMC5883P_Read(int16_t mag[3])
{
    uint8_t buf[6];
    if (!I2C_ReadRegs(QMC5883P_I2C_ADDR, QMC5883P_REG_DATA, buf, 6)) {
        return 0;
    }
    mag[0] = (int16_t)((buf[1] << 8) | buf[0]);   /* 小端 */
    mag[1] = (int16_t)((buf[3] << 8) | buf[2]);
    mag[2] = (int16_t)((buf[5] << 8) | buf[4]);
    return 1;
}
