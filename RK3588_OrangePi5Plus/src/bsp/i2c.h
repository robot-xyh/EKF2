/********************************************************************************
 * File  : i2c.h
 * Brief : /dev/i2c-* 封装 (内核 I2C, 非 bit-bang)。接口对齐 CH32 的 soft_i2c:
 *         Probe / ReadReg / ReadRegs / WriteReg。
 ********************************************************************************/
#ifndef __I2C_H
#define __I2C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int  I2C_Open(const char *dev);     /* 返回 0 成功 */
void I2C_Close(void);

/* 全部返回 1=成功, 0=失败 (与 CH32 soft_i2c 语义一致) */
int  I2C_Probe(uint8_t addr);
int  I2C_ReadReg(uint8_t addr, uint8_t reg, uint8_t *val);
int  I2C_ReadRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
int  I2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t val);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H */
