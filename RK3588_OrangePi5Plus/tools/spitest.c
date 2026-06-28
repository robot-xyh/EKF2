/********************************************************************************
 * File  : spitest.c
 * Brief : SPI 底层自检 —— 连续读 ICM WHO_AM_I, 观察返回值规律, 定位问题:
 *           全 0xFF  -> MISO 一直高 (浮空/无应答)
 *           全 0x00  -> MISO 一直低 (片选没生效/无应答)
 *           0x39     -> 正常
 *           时好时坏 -> 时序/信号问题
 *         编译: make spitest    运行: sudo ./spitest
 ********************************************************************************/
#include "bsp.h"
#include "spi.h"
#include "delay.h"
#include <stdio.h>

int main(void)
{
    if (BSP_Init() != 0) {
        fprintf(stderr, "BSP_Init 失败\n");
        return 1;
    }

    printf("连续读 WHO_AM_I (0x75, 期望 0x39):\n");
    int ok = 0;
    for (int i = 0; i < 30; i++) {
        /* 每次先软复位, 再读几个寄存器 */
        SPI_WriteReg(0x11, 0x01);          /* DEVICE_CONFIG soft reset */
        Delay_Ms(5);

        uint8_t who   = SPI_ReadReg(0x75); /* WHO_AM_I */
        uint8_t cfg   = SPI_ReadReg(0x4E); /* PWR_MGMT0 复位后应=0x00 */
        printf("  #%02d  WHO_AM_I=0x%02X  PWR_MGMT0=0x%02X  %s\n",
               i, who, cfg, (who == 0x39) ? "<== OK" : "");
        if (who == 0x39) ok++;
        Delay_Ms(100);
    }
    printf("成功 %d / 30\n", ok);

    BSP_Deinit();
    return 0;
}
