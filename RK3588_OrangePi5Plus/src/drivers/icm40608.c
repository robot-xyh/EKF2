/********************************************************************************
 * File  : icm40608.c
 * Brief : IMU ICM40608 (ICM426xx 家族) SPI 驱动。逻辑与 CH32 版一致,
 *         总线换成 spidev(见 bsp/spi.c), 片选为软件 GPIO1_A4。
 *         参考 Betaflight: drivers/accgyro/accgyro_spi_icm426xx.c
 ********************************************************************************/
#include "sensors.h"
#include "bsp.h"
#include "spi.h"
#include "delay.h"

/* ICM426xx Bank0 寄存器 */
#define ICM_RA_DEVICE_CONFIG    0x11   /* bit0 = SOFT_RESET */
#define ICM_RA_PWR_MGMT0        0x4E
#define ICM_RA_GYRO_CONFIG0     0x4F
#define ICM_RA_ACCEL_CONFIG0    0x50
#define ICM_RA_TEMP_DATA1       0x1D   /* 之后: TEMP(2) ACCEL(6) GYRO(6) */

#define ICM_SOFT_RESET          0x01
#define ICM_PWR_ALL_LN          0x0F   /* 加速度+陀螺 低噪声模式 */
#define ICM_CFG_2000DPS_1KHZ    0x06   /* FS_SEL=0(2000dps), ODR=1kHz */
#define ICM_CFG_16G_1KHZ        0x06   /* FS_SEL=0(16g),     ODR=1kHz */

/* 上电软复位: 让设备从任何坏状态恢复, 免去拔插。datasheet 复位需 >1ms */
static void icm_soft_reset(void)
{
    SPI_WriteReg(ICM_RA_DEVICE_CONFIG, ICM_SOFT_RESET);
    Delay_Ms(5);
}

void ICM40608_Detect(sensor_result_t *r)
{
    r->present = 0;
    r->id_ok   = 0;
    r->address = 0;
    r->id      = 0;

    icm_soft_reset();

    uint8_t id = SPI_ReadReg(ICM40608_REG_WHOAMI);
    r->id = id;

    if (id != 0x00 && id != 0xFF) {
        r->present = 1;
    }
    if (id == ICM40608_WHOAMI_VAL) {
        r->id_ok = 1;
    }
}

void ICM40608_Init(void)
{
    SPI_WriteReg(ICM_RA_PWR_MGMT0, ICM_PWR_ALL_LN);
    Delay_Ms(2);
    SPI_WriteReg(ICM_RA_GYRO_CONFIG0,  ICM_CFG_2000DPS_1KHZ);
    SPI_WriteReg(ICM_RA_ACCEL_CONFIG0, ICM_CFG_16G_1KHZ);
    Delay_Ms(5);

    SPI_SetSpeed(IMU_SPI_HZ_FAST);     /* 配置完成后切高速读数 */
}

uint8_t ICM40608_Read(imu_data_t *d)
{
    uint8_t buf[14];
    SPI_ReadBuf(ICM_RA_TEMP_DATA1, buf, 14);   /* 大端 */

    d->temp    = (int16_t)((buf[0]  << 8) | buf[1]);
    d->acc[0]  = (int16_t)((buf[2]  << 8) | buf[3]);
    d->acc[1]  = (int16_t)((buf[4]  << 8) | buf[5]);
    d->acc[2]  = (int16_t)((buf[6]  << 8) | buf[7]);
    d->gyro[0] = (int16_t)((buf[8]  << 8) | buf[9]);
    d->gyro[1] = (int16_t)((buf[10] << 8) | buf[11]);
    d->gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);

    return 1;
}
