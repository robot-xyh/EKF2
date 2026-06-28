/********************************************************************************
 * File  : altitude.h
 * Brief : 气压 -> 海拔/相对高度换算 (国际气压高度公式)。
 ********************************************************************************/
#ifndef __ALTITUDE_H
#define __ALTITUDE_H

#ifdef __cplusplus
extern "C" {
#endif

/* 由气压(Pa)算绝对海拔(m), 基于海平面标准气压 101325 Pa */
float Altitude_FromPressure(float pressure_pa);

/* 设定地面基准气压(开机静止时调用), 之后 Altitude_Relative 给出相对起飞高度 */
void  Altitude_SetGround(float pressure_pa);
float Altitude_Relative(float pressure_pa);

#ifdef __cplusplus
}
#endif

#endif /* __ALTITUDE_H */
