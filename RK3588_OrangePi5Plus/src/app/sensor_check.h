/********************************************************************************
 * File  : sensor_check.h
 * Brief : 识别三传感器 -> 校准 -> 实时刷新 AHRS 姿态/高度仪表盘。
 ********************************************************************************/
#ifndef __SENSOR_CHECK_H
#define __SENSOR_CHECK_H

#ifdef __cplusplus
extern "C" {
#endif

/* 阻塞运行实时仪表盘, Ctrl-C 退出。返回 0 正常。 */
int SensorCheck_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_CHECK_H */
