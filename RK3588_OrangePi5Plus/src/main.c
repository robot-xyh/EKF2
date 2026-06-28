/********************************************************************************
 * File  : main.c
 * Brief : 入口 —— 打开总线 -> 跑实时传感器/AHRS 仪表盘 -> 退出关闭。
 ********************************************************************************/
#include "bsp.h"
#include "sensor_check.h"
#include <stdio.h>

int main(void)
{
    if (BSP_Init() != 0) {
        fprintf(stderr, "BSP_Init 失败: 检查是否有 /dev/spidev0.0 /dev/i2c-5 "
                        "以及运行权限(可用 sudo)。\n");
        return 1;
    }

    int ret = SensorCheck_Run();

    BSP_Deinit();
    return ret;
}
