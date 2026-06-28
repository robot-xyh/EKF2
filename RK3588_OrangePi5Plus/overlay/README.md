# 设备树 Overlay 说明

本工程**不需要任何自定义 overlay**，直接用 Orange Pi 官方自带的 overlay 即可。

编辑 `/boot/orangepiEnv.txt`，确保有这一行：

```
overlays=spi0-m2-cs0-spidev i2c5-m3
```

含义：
- `spi0-m2-cs0-spidev`：开启 SPI0(M2 引脚组)，并把 **CS0 交给硬件 SPI 控制器**自动管理，
  生成 `/dev/spidev0.0`。IMU 的片选必须接到 **SPI0_CS0 = 40pin 的 Pin24**。
- `i2c5-m3`：开启 I2C5(M3 引脚组)，生成 `/dev/i2c-5`，用于磁力计/气压计。

改完 `sudo reboot`，重启后应有 `/dev/spidev0.0` 和 `/dev/i2c-5`。

> 这两个 overlay 是 Orange Pi BSP 内置的（位于 `/boot/dtb/rockchip/overlay/`），
> 不用自己编译。`sudo orangepi-config` 的 System → Hardware 里也能勾选。
