# Repository Guidelines

## Project Structure & Module Organization

This repository contains one hardware-focused C project under `RK3588_OrangePi5Plus/`. Edit the directory, not the root zip.

- `src/main.c` is the application entry point.
- `src/bsp/` contains board support and hardware configuration. Update `src/bsp/bsp.h` for device nodes, GPS UART, chip select, or axis mapping.
- `src/drivers/` contains ICM40608, QMC5883P, and DPS310-compatible drivers.
- `src/app/` contains AHRS, altitude, and sensor-check logic; `src/ekf2/` and `src/bfnav/` contain experimental navigation dashboards.
- `tools/` contains SPI and UBX parser diagnostics.
- `external/` contains vendored PX4-ECL and Matrix dependencies.
- `overlay/README.md` documents Orange Pi overlay setup.

## Build, Test, and Development Commands

Run commands from `RK3588_OrangePi5Plus/`:

```bash
make              # build ./sensor
make run          # build and run ./sensor with sudo
./run.sh          # scripted build + sudo run
make ekf2         # CMake build for ./sensor_ekf2
make run-ekf2     # build and run EKF2 dashboard with sudo
make bfnav        # build ./sensor_bfnav
make run-bfnav    # build and run BFNav dashboard with sudo
make spitest      # build ./spitest diagnostic
sudo ./spitest    # verify stable IMU WHO_AM_I reads
make ubx-test     # run UBX NAV-PVT parser self-test
make bfnav-test   # run BFNav estimator self-test
make clean        # remove objects and local binaries
```

Runtime expects `/dev/spidev0.0` and `/dev/i2c-5`; hardware runs need `sudo`.

## Coding Style & Naming Conventions

The project is C/C++ built with `-Wall -Wextra`. Match existing style: 4-space indentation, K&R braces, short module comments, and hardware constants in headers. Public APIs use `Prefix_Action` names such as `BSP_Init`; local helpers use lower snake case. Keep board configuration in `src/bsp/bsp.h`.

## Testing Guidelines

There is no full unit-test framework. At minimum, run `make` after edits. For navigation changes, run `make ubx-test && make bfnav-test && make bfnav && make ekf2`. For hardware or driver changes, run `make spitest && sudo ./spitest`, confirm I2C with `sudo i2cdetect -y 5`, then run the relevant dashboard. Do not commit binaries, `build/`, or `*.o`.

## Commit & Pull Request Guidelines

No local Git history is present. Use concise, imperative subjects such as `Fix DPS310 pressure conversion`. PRs should list hardware tested, commands run, observed sensor IDs, and wiring or overlay changes.

## Configuration & Safety Notes

Before hardware tests, enable `spi0-m2-cs0-spidev i2c5-m3`, keep IMU CS on Pin 24, and leave IMUCLK floating. Document pinout, bus, GPS, and axis-map changes.
