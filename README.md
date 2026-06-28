# EKF2 Orange Pi Sensor Project

This repository contains one active project:

```text
RK3588_OrangePi5Plus/
```

It targets Orange Pi 5 Plus / RK3588 and reads an ICM40608 IMU, QMC5883P magnetometer, DPS310-compatible barometer, and optional u-blox UBX GPS. It provides:

- `sensor`: Mahony 9-axis AHRS dashboard.
- `sensor_ekf2`: experimental PX4 ECL/EKF2 navigation dashboard.
- `sensor_bfnav`: experimental Betaflight-style GPS/baro/IMU position estimator.

Start here:

```bash
cd RK3588_OrangePi5Plus
make
sudo ./sensor

make ekf2
sudo ./sensor_ekf2

make bfnav
sudo ./sensor_bfnav
```

See [RK3588_OrangePi5Plus/README.md](RK3588_OrangePi5Plus/README.md) for wiring, overlay setup, GPS configuration, tests, and troubleshooting.
