# ============================================================================
#  Orange Pi 5 Plus 传感器 + AHRS  本地编译 Makefile
#  在香橙派上直接: make    (产物 ./sensor)
# ============================================================================
TARGET  := sensor

CC      := gcc
CFLAGS  := -O2 -Wall -Wextra -std=gnu11 \
           -Isrc/bsp -Isrc/drivers -Isrc/app -Isrc/bfnav
LDFLAGS := -lm

# 递归收集 src 下所有 .c
SRCS := $(wildcard src/*.c) \
        $(wildcard src/bsp/*.c) \
        $(wildcard src/drivers/*.c) \
        $(wildcard src/app/*.c)
OBJS := $(SRCS:.c=.o)

BSPOBJS := src/bsp/bsp.o src/bsp/spi.o src/bsp/gpio.o src/bsp/i2c.o
BFNAVOBJS := src/bfnav/sensor_bfnav.o \
             src/bfnav/bf_position_filter.o \
             src/bfnav/bf_position_estimator.o \
             src/bsp/bsp.o src/bsp/spi.o src/bsp/gpio.o src/bsp/i2c.o src/bsp/uart.o \
             src/drivers/icm40608.o src/drivers/qmc5883p.o src/drivers/dps310.o src/drivers/ubx_gps.o \
             src/app/ahrs.o src/app/altitude.o

.PHONY: all clean run ekf2 run-ekf2 bfnav run-bfnav ubx-test bfnav-test

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)
	@echo "==> 编译完成: ./$(TARGET)"

spitest: tools/spitest.o $(BSPOBJS)
	$(CC) $^ -o $@ $(LDFLAGS)
	@echo "==> 编译完成: ./spitest"

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

run: all
	sudo ./$(TARGET)

ekf2:
	cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
	cmake --build build --target sensor_ekf2 -j
	cp build/sensor_ekf2 ./sensor_ekf2

run-ekf2: ekf2
	sudo ./sensor_ekf2

bfnav: $(BFNAVOBJS)
	$(CC) $(BFNAVOBJS) -o sensor_bfnav $(LDFLAGS)
	@echo "==> 编译完成: ./sensor_bfnav"

run-bfnav: bfnav
	sudo ./sensor_bfnav

ubx-test: tools/ubx_test.c src/drivers/ubx_gps.c src/drivers/ubx_gps.h
	$(CC) $(CFLAGS) -Isrc/drivers tools/ubx_test.c src/drivers/ubx_gps.c -o ubx_test
	./ubx_test

bfnav-test: tools/bfnav_test.c src/bfnav/bf_position_filter.c src/bfnav/bf_position_estimator.c
	$(CC) $(CFLAGS) tools/bfnav_test.c src/bfnav/bf_position_filter.c src/bfnav/bf_position_estimator.c -o bfnav_test $(LDFLAGS)
	./bfnav_test

clean:
	rm -f $(OBJS) src/bfnav/*.o tools/spitest.o $(TARGET) spitest sensor_ekf2 sensor_bfnav ubx_test bfnav_test
	rm -rf build
