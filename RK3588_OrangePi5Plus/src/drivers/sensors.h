/********************************************************************************
 * File  : sensors.h
 * Brief : 三个传感器驱动公共定义 (移植自 CH32 版, 寄存器/补偿逻辑一致)。
 ********************************************************************************/
#ifndef __SENSORS_H
#define __SENSORS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t present;    /* 1=总线上有器件应答 */
    uint8_t id_ok;      /* 1=ID 与期望一致 */
    uint8_t address;    /* 实际 7bit 地址 (SPI 为 0) */
    uint8_t id;         /* 实际读到的 ID */
} sensor_result_t;

/* IMU 一帧原始数据 (±16g, ±2000dps) */
typedef struct {
    int16_t acc[3];
    int16_t gyro[3];
    int16_t temp;
} imu_data_t;

/* ---- 识别 ---- */
void ICM40608_Detect(sensor_result_t *r);
void QMC5883P_Detect(sensor_result_t *r);
void DPS310_Detect(sensor_result_t *r);

/* ---- 初始化(识别成功后调用一次) ---- */
void ICM40608_Init(void);
void QMC5883P_Init(void);
void DPS310_Init(void);

/* ---- 读实时数据: 返回 1=成功 ---- */
uint8_t ICM40608_Read(imu_data_t *d);
uint8_t QMC5883P_Read(int16_t mag[3]);
uint8_t DPS310_Read(int32_t *pressure_pa, int32_t *temp_centi);

#ifdef __cplusplus
}
#endif

#endif /* __SENSORS_H */
