/********************************************************************************
 * File  : spi.h
 * Brief : spidev 封装 (SPI_NO_CS + 软件片选)。供 IMU 驱动调用,
 *         接口对齐 CH32 版: 读/写寄存器 + 连续读, 内部自动拉 CS。
 ********************************************************************************/
#ifndef __SPI_H
#define __SPI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 打开 spidev, 设置 mode/bits/speed; CS 由外部 gpio 模块提供。返回 0 成功 */
int  SPI_Open(const char *dev, uint8_t mode, uint8_t bits, uint32_t hz);
void SPI_SetSpeed(uint32_t hz);
void SPI_Close(void);

/* 一次全双工传输(len 字节), 内部软件 CS 低->传输->高。返回 0 成功 */
int  SPI_Transfer(const uint8_t *tx, uint8_t *rx, int len);

/* 便捷封装(IMU 用): 寄存器读写, 自动处理 读位(reg|0x80) */
uint8_t SPI_ReadReg(uint8_t reg);
void    SPI_WriteReg(uint8_t reg, uint8_t val);
void    SPI_ReadBuf(uint8_t reg, uint8_t *buf, int len);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */
