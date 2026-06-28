/********************************************************************************
 * File    : bsp.h
 * Brief   : 板级支持包 —— 本工程用到的【所有设备节点 / 引脚 / 总线参数】
 *           都集中定义在这里。改接线 / 换总线只看这一个文件。
 *
 * 板子    : Orange Pi 5 Plus (RK3588)  +  传感器小板(IMU/磁力计/气压计)
 * 系统    : Ubuntu 22.04 (linux 6.1)
 *
 * ============================ 总线分配 (实测可用) ============================
 *   IMU   ICM40608   -> SPI0_M2  /dev/spidev0.0   硬件片选 SPI0_CS0(Pin24)
 *   磁力计 QMC5883P   -> I2C5_M3  /dev/i2c-5  @0x2C
 *   气压计 LQTP001HTA -> I2C5_M3  /dev/i2c-5  @0x76 (DPS310/SPL07 协议族)
 *
 * ===================== 40pin 物理接线 (实测 30/30 读到 0x39) =================
 *   IMU  SDI/MOSI  <- Pin19  (SPI0_MOSI_M2 / GPIO1_B2)
 *   IMU  SDO/MISO  -> Pin21  (SPI0_MISO_M2 / GPIO1_B1)
 *   IMU  SCLK      <- Pin23  (SPI0_CLK_M2  / GPIO1_B3)
 *   IMU  CS        <- Pin24  (SPI0_CS0_M2  / GPIO1_B4)   <== 关键: 接硬件 CS0
 *   磁/压 I2C_SDA  <-> Pin27 (I2C5_SDA_M3  / GPIO1_B7)
 *   磁/压 I2C_SCL  <-> Pin28 (I2C5_SCL_M3  / GPIO1_B6)
 *   3.3V Pin1/Pin17    GND Pin6/9/14/20/25/30/34/39
 *
 *   重要经验:
 *     1) IMU 的片选必须接到【硬件 SPI0_CS0 = Pin24】, 由内核 SPI 控制器自动
 *        在每次传输时拉低/拉高, 这样帧时序正确, 才能稳定读到 WHO_AM_I=0x39。
 *     2) IMUCLK(外部时钟输入)不要使用、也不要接到会跳变的脚上, 否则会扰乱
 *        IMU 内部时钟导致读不出数据。本工程 IMU 走内部时钟, IMUCLK 悬空。
 *     3) (历史教训) 本板 GPIO1_A4/Pin11 实测无法作为 GPIO 输出, 故放弃软件
 *        片选, 改用硬件片选; 软件片选代码保留但默认不启用(见下方宏)。
 ********************************************************************************/
#ifndef __BSP_H
#define __BSP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 1. 设备节点
 * ==========================================================================*/
#define IMU_SPI_DEV         "/dev/spidev0.0"
#define SENSOR_I2C_DEV      "/dev/i2c-5"
#define GPS_UART_DEV        "/dev/ttyUSB0"
#define GPS_UART_BAUD       115200

/* ---- 片选(CS)方式 ----
 *   1 = 硬件片选(默认, 推荐): 用官方 overlay spi0-m2-cs0-spidev,
 *       内核 SPI 控制器自动驱动 SPI0_CS0(Pin24), 用户态不碰 CS。
 *   0 = 用户态软件片选: 自己 toggle 一个普通 GPIO 当 CS
 *       (仅当你把 IMU_CS 接到某个可用的普通 GPIO 时才用, 见下方 IMU_CS_LINE)。
 *   >>> 本板验证可用的是【硬件片选】, 故默认 1。 */
#define IMU_CS_KERNEL_MANAGED   1

/* 仅 IMU_CS_KERNEL_MANAGED=0 时使用: 软件片选所接的 GPIO。
 *   gpiochip label = "gpio1" (自动探测, 失败回退 fallback)
 *   line offset: A0..A7=0..7, B0..B7=8..15  (例: B0=8 对应 40pin Pin15) */
#define IMU_CS_CHIP_LABEL   "gpio1"
#define IMU_CS_CHIP_FALLBACK "/dev/gpiochip1"
#define IMU_CS_LINE         8

/* ============================================================================
 * 2. SPI 参数 (ICM40608 / ICM426xx 家族)
 * ==========================================================================*/
#define IMU_SPI_MODE        3           /* 与 CH32 验证一致: CPOL=1,CPHA=1 */
#define IMU_SPI_BITS        8
#define IMU_SPI_HZ_SLOW     1000000      /* 识别用 1MHz */
#define IMU_SPI_HZ_FAST     8000000      /* 运行用 8MHz */

/* ============================================================================
 * 3. 传感器地址 / ID 寄存器 / 期望 ID
 *    (取自 Betaflight: accgyro_spi_icm426xx / compass_qmc5883 / barometer_dps310)
 * ==========================================================================*/

/* ---- IMU: ICM40608 (SPI, ICM426xx 家族) ---- */
#define ICM40608_REG_WHOAMI     0x75
#define ICM40608_WHOAMI_VAL     0x39
#define ICM40608_SPI_READ       0x80    /* SPI 读: 寄存器地址最高位置1 */

/* ---- 磁力计: QMC5883P (I2C) ---- */
#define QMC5883P_I2C_ADDR       0x2C
#define QMC5883P_REG_ID         0x00
#define QMC5883P_ID_VAL         0x80

/* ---- 气压计: LQTP001HTA (I2C, DPS310/SPL07 协议族) ---- */
#define DPS310_I2C_ADDR         0x76
#define DPS310_I2C_ADDR_ALT     0x77
#define DPS310_REG_ID           0x0D
#define DPS310_ID_VAL           0x10    /* Infineon DPS310 */
#define SPL07_ID_VAL            0x11    /* SPL07-003 兼容 */

/* ============================================================================
 * 4. 量程换算常数 (与 CH32 版一致, ICM42xx ±16g / ±2000dps)
 * ==========================================================================*/
#define ACC_LSB_PER_G       2048.0f     /* ±16g  -> 2048 LSB/g  */
#define GYRO_LSB_PER_DPS    16.4f       /* ±2000 -> 16.4 LSB/dps */
#define MAG_LSB_PER_GAUSS   2500.0f     /* QMC5883P ±8G scale, tune after mag calibration */
#define GRAVITY_MSS         9.80665f
#define DEG2RAD             0.017453292519943295f
#define RAD2DEG             57.29577951308232f

/* ============================================================================
 * 5. EKF frame mapping
 *    PX4 EKF expects body FRD axes. Defaults pass sensor axes through.
 * ==========================================================================*/
#define EKF_IMU_X_SRC       0
#define EKF_IMU_Y_SRC       1
#define EKF_IMU_Z_SRC       2
#define EKF_IMU_X_SIGN      1.0f
#define EKF_IMU_Y_SIGN      1.0f
#define EKF_IMU_Z_SIGN      1.0f

#define EKF_MAG_X_SRC       0
#define EKF_MAG_Y_SRC       1
#define EKF_MAG_Z_SRC       2
#define EKF_MAG_X_SIGN      1.0f
#define EKF_MAG_Y_SIGN      1.0f
#define EKF_MAG_Z_SIGN      1.0f

#define EKF_GPS_POS_X_M     0.0f
#define EKF_GPS_POS_Y_M     0.0f
#define EKF_GPS_POS_Z_M     0.0f

/* ============================================================================
 * 6. 板级 API: 打开/关闭所有总线
 * ==========================================================================*/
int  BSP_Init(void);    /* 打开 spidev + i2c (+软件CS gpio), 返回 0 成功 */
void BSP_Deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_H */
