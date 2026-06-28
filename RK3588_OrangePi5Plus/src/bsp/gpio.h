/********************************************************************************
 * File  : gpio.h
 * Brief : 用内核 /dev/gpiochip 字符设备(ioctl v1) 控制单个输出引脚。
 *         专用于 IMU 软件片选 CS = GPIO1_A4。无需 libgpiod 库。
 ********************************************************************************/
#ifndef __GPIO_H
#define __GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* 打开 chip(按 label 自动探测, 失败用 fallback 路径) 并把 line 申请为输出。
 * init_val: 初始电平(CS 空闲为高 -> 传 1)。返回 0 成功。 */
int  GPIO_OpenOutput(const char *chip_label, const char *fallback_path,
                     unsigned int line, int init_val);

/* 设置电平 0/1 */
void GPIO_Write(int value);

void GPIO_Close(void);

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_H */
