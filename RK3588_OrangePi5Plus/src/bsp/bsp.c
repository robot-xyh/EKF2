/********************************************************************************
 * File  : bsp.c
 * Brief : 打开/关闭三条总线: spidev(IMU) + i2c(磁/气压) + gpio(软件CS)。
 ********************************************************************************/
#include "bsp.h"
#include "spi.h"
#include "i2c.h"
#include "gpio.h"

#include <stdio.h>

int BSP_Init(void)
{
#if !IMU_CS_KERNEL_MANAGED
    /* 1) 软件片选 GPIO1_A4, 空闲高 (方案A) */
    if (GPIO_OpenOutput(IMU_CS_CHIP_LABEL, IMU_CS_CHIP_FALLBACK,
                        IMU_CS_LINE, 1) != 0) {
        fprintf(stderr, "BSP: GPIO(CS) 打开失败\n");
        return -1;
    }
#endif

    /* 2) SPI (识别先用慢速). 方案B 下片选由内核 cs-gpios 自动处理 */
    if (SPI_Open(IMU_SPI_DEV, IMU_SPI_MODE, IMU_SPI_BITS, IMU_SPI_HZ_SLOW) != 0) {
        fprintf(stderr, "BSP: SPI 打开失败 (%s)\n", IMU_SPI_DEV);
        GPIO_Close();
        return -1;
    }

    /* 3) I2C */
    if (I2C_Open(SENSOR_I2C_DEV) != 0) {
        fprintf(stderr, "BSP: I2C 打开失败 (%s)\n", SENSOR_I2C_DEV);
        SPI_Close();
        GPIO_Close();
        return -1;
    }

    return 0;
}

void BSP_Deinit(void)
{
    I2C_Close();
    SPI_Close();
    GPIO_Close();
}
