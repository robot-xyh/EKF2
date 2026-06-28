# Orange Pi 5 Plus 传感器 / AHRS / EKF2 / BFNav

本工程把 CH32V203 验证小板的传感器代码移植到 **Orange Pi 5 Plus (RK3588 / Ubuntu)**。程序在 Linux 用户态读取 IMU、磁力计、气压计和 GPS，并提供三个运行目标：

- `sensor`：稳定版 Mahony 9 轴 AHRS 仪表盘，输出姿态、航向和气压高度。
- `sensor_ekf2`：实验版 PX4 ECL/EKF2 组合导航，融合 IMU、磁力计、气压计和 u-blox UBX GPS，输出姿态、NED 位置/速度和 EKF 状态。
- `sensor_bfnav`：实验版 Betaflight 风格组合导航状态估计，使用 East/North/Up 三个 1D Kalman filter 融合 IMU、GPS、气压计，并用磁力计辅助 Mahony 姿态。

> 已验证：原始 `sensor` 路径可稳定识别三传感器，IMU `WHO_AM_I=0x39`。`sensor_ekf2` 和 `sensor_bfnav` 已完成 clean build 与基础解析/估计器测试，实际导航效果需要在带 GPS 3D fix 的硬件上验证。

---

## 硬件连接

| 设备 | 型号 | 总线 | 节点 / 地址 |
|---|---|---|---|
| IMU | ICM40608 | SPI0_M2 | `/dev/spidev0.0`，硬件 CS0 = Pin24 |
| 磁力计 | QMC5883P | I2C5_M3 | `/dev/i2c-5` @ `0x2C` |
| 气压计 | LQTP001HTA / DPS310 族 | I2C5_M3 | `/dev/i2c-5` @ `0x76` |
| GPS | u-blox UBX NAV-PVT | UART | 默认 `/dev/ttyUSB0` @ `115200` |

40pin 接线：

```text
IMU  SDI/MOSI  <- Pin19  (SPI0_MOSI_M2)
IMU  SDO/MISO  -> Pin21  (SPI0_MISO_M2)
IMU  SCLK      <- Pin23  (SPI0_CLK_M2)
IMU  CS        <- Pin24  (SPI0_CS0_M2)
磁/压 I2C_SDA  <-> Pin27 (I2C5_SDA_M3)
磁/压 I2C_SCL  <-> Pin28 (I2C5_SCL_M3)
3.3V: Pin1/17       GND: Pin6/9/14/20/25/30/34/39
```

关键约束：

1. IMU 片选必须接 **Pin24 硬件 CS0**。
2. IMUCLK 悬空，不要接到会跳变的信号。
3. 当前板子实测 GPIO1_A4/Pin11 不适合做软件片选，默认使用内核硬件片选。

---

## 系统配置

编辑 `/boot/orangepiEnv.txt`：

```text
overlays=spi0-m2-cs0-spidev i2c5-m3
```

重启后检查：

```bash
ls /dev/spidev0.0 /dev/i2c-5
sudo i2cdetect -y 5
```

I2C 扫描应看到 `0x2c` 和 `0x76`。overlay 说明见 `overlay/README.md`。

---

## 构建与运行

所有命令在 `RK3588_OrangePi5Plus/` 目录下执行。

### Mahony AHRS 稳定版

```bash
make
sudo ./sensor
# 或:
./run.sh
```

启动流程：识别传感器、静止校准陀螺零偏、校准地面气压，然后进入 10 Hz 终端仪表盘。`Ctrl-C` 退出。

### PX4 ECL/EKF2 实验版

```bash
make ekf2
sudo ./sensor_ekf2
# 或:
make run-ekf2
```

`sensor_ekf2` 会显示：

- IMU 原始加速度、角速度和温度
- GPS fix、卫星数、经纬高、精度
- 磁力计、气压高度
- EKF2 roll/pitch/yaw、NED 位置、NED 速度
- EKF flags、fault、innovation、GPS check 状态
- Mahony 姿态对照

GPS 没有 3D fix 时，EKF2 仍会运行姿态/高度估计，但位置状态会显示 invalid。

### Betaflight 风格组合导航实验版

```bash
make bfnav
sudo ./sensor_bfnav
# 或:
make run-bfnav
```

`sensor_bfnav` 使用 Betaflight 当前主线位置估计器的核心结构：East、North、Up 三个独立 1D Kalman filter，内部单位为 ENU cm / cm/s。第一版融合：

- IMU 线加速度预测位置/速度
- GPS 经纬度、ENU 速度、MSL 高度修正 XY/Z
- 气压高度修正 Z
- 磁力计参与 Mahony 姿态/航向，帮助把机体系加速度转到地理系

GPS 未 3D fix 时，XY 位置无效；气压计可用时 Z 高度仍会更新。

---

## 配置入口

集中配置文件：

```text
src/bsp/bsp.h
```

常改项：

```c
#define GPS_UART_DEV        "/dev/ttyUSB0"
#define GPS_UART_BAUD       115200

#define EKF_IMU_X_SRC       0
#define EKF_IMU_Y_SRC       1
#define EKF_IMU_Z_SRC       2
#define EKF_IMU_X_SIGN      1.0f
#define EKF_IMU_Y_SIGN      1.0f
#define EKF_IMU_Z_SIGN      1.0f
```

PX4 EKF 使用机体系 **FRD**。如果板子安装方向不一致，调整 `EKF_IMU_*` 和 `EKF_MAG_*` 轴映射，不要先改算法。

---

## 自检与测试

```bash
make spitest
sudo ./spitest
```

`spitest` 连续读取 IMU `WHO_AM_I`，应稳定为 `0x39`。

```bash
make ubx-test
```

`ubx-test` 验证 UBX `NAV-PVT` 解析器，包括有效帧和 checksum 错误帧。

```bash
make bfnav-test
```

`bfnav-test` 验证 Betaflight 风格估计器的 Z 收敛、GPS XY 更新和测量超时。

完整构建检查：

```bash
make clean
make
make ubx-test
make bfnav-test
make bfnav
make ekf2
make spitest
```

---

## 目录结构

```text
RK3588_OrangePi5Plus/
├── Makefile             make / make ekf2 / make spitest
├── CMakeLists.txt       EKF2/C++ 构建入口
├── run.sh               构建并运行 Mahony 仪表盘
├── external/            PX4-ECL 与 Matrix 依赖
├── overlay/             Orange Pi overlay 说明
├── tools/               spitest 与 UBX parser 测试
└── src/
    ├── main.c           Mahony 仪表盘入口
    ├── app/             AHRS、高度、传感器检测
    ├── bfnav/           Betaflight 风格位置估计与仪表盘
    ├── bsp/             SPI/I2C/GPIO/UART 与硬件配置
    ├── drivers/         IMU、磁力计、气压计、UBX GPS
    └── ekf2/            PX4 ECL wrapper 与 EKF2 仪表盘
```

---

## Betaflight 来源与许可

`src/bfnav/` 中的状态估计代码直接参考/移植自 Betaflight：

- 源仓库：`https://github.com/betaflight/betaflight`
- 基线 commit：`25a7f7417164d88c0db833db00af2066857e1258`
- 参考文件：`src/main/flight/position_filter.c`、`position_estimator.c`
- 许可：GPLv3 or later

这部分仅用于科研教学实验。若分发包含 `src/bfnav/` 的版本，请按 GPLv3 要求保留来源、许可和修改说明。

---

## 移植检查清单

1. 接线确认：IMU CS 接 Pin24，IMUCLK 悬空，I2C 使用 Pin27/Pin28。
2. overlay 生效：`/dev/spidev0.0` 和 `/dev/i2c-5` 存在。
3. I2C 扫描：看到 `0x2c` 和 `0x76`。
4. SPI 自检：`make spitest && sudo ./spitest` 稳定输出 `0x39`。
5. Mahony 验证：`sudo ./sensor` 姿态输出稳定。
6. GPS 验证：确认 u-blox 输出 UBX `NAV-PVT`，串口与波特率匹配。
7. EKF2 验证：`sudo ./sensor_ekf2`，GPS 3D fix 后观察 `gps`、`local`、`global` flags。
8. BFNav 验证：`sudo ./sensor_bfnav`，GPS 3D fix 后观察 `validXY`、`validZ`、`trustXY`、`trustZ`。
9. 坐标修正：姿态或航向方向不对时，先改 `bsp.h` 轴映射。

---

## 常见问题

- **权限不足**：直接用 `sudo`，或给用户配置 `spi`、`i2c`、`dialout` 等设备权限。
- **`spitest` 为 0x00 / 0xFF**：重点查 CS 是否接 Pin24、MOSI/MISO/SCLK 是否接反、IMUCLK 是否误接。
- **找不到 `/dev/spidev0.0`**：overlay 未启用或未重启。
- **I2C 扫不到传感器**：查 SDA/SCL、3.3V、GND、上拉。
- **GPS 一直 waiting**：检查 `GPS_UART_DEV`、波特率、GPS 是否输出 UBX `NAV-PVT`，以及用户是否有串口权限。
- **EKF2 位置 invalid**：需要 GPS 3D fix、合理精度字段和持续数据输入；仅 IMU+磁力计+气压计不能提供可靠水平位置。
- **BFNav XY invalid**：需要 GPS 3D fix；无 GPS 时只验证 Z 高度链路。
- **BFNav 位置方向反了**：先确认 `EKF_IMU_*` / `EKF_MAG_*` 轴映射和 GPS ENU 方向，再判断滤波器参数。
