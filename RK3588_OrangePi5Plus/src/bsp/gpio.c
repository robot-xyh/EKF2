/********************************************************************************
 * File  : gpio.c
 * Brief : /dev/gpiochip ioctl (GPIO v1 line handle) 控制 IMU 软件片选。
 ********************************************************************************/
#include "gpio.h"

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>

static int s_line_fd = -1;      /* GPIOHANDLE 行句柄 */

/* 遍历 /dev/gpiochip0..9, 找 label 匹配的那个; 返回打开的 chip fd, 否则 -1 */
static int find_chip_by_label(const char *label)
{
    char path[32];
    int i;

    for (i = 0; i < 10; i++) {
        snprintf(path, sizeof(path), "/dev/gpiochip%d", i);
        int fd = open(path, O_RDWR | O_CLOEXEC);
        if (fd < 0) {
            continue;
        }
        struct gpiochip_info info;
        memset(&info, 0, sizeof(info));
        if (ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &info) == 0) {
            if (strcmp(info.label, label) == 0) {
                return fd;   /* 命中 */
            }
        }
        close(fd);
    }
    return -1;
}

int GPIO_OpenOutput(const char *chip_label, const char *fallback_path,
                    unsigned int line, int init_val)
{
    int chip_fd = find_chip_by_label(chip_label);

    if (chip_fd < 0 && fallback_path) {
        chip_fd = open(fallback_path, O_RDWR | O_CLOEXEC);
        if (chip_fd >= 0) {
            fprintf(stderr, "[gpio] label '%s' 未找到, 回退使用 %s\n",
                    chip_label, fallback_path);
        }
    }
    if (chip_fd < 0) {
        fprintf(stderr, "[gpio] 打不开 gpiochip (label=%s)\n", chip_label);
        return -1;
    }

    struct gpiohandle_request req;
    memset(&req, 0, sizeof(req));
    req.lineoffsets[0]   = line;
    req.lines            = 1;
    req.flags            = GPIOHANDLE_REQUEST_OUTPUT;
    req.default_values[0]= (init_val ? 1 : 0);
    strncpy(req.consumer_label, "imu_cs", sizeof(req.consumer_label) - 1);

    if (ioctl(chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
        perror("[gpio] GPIO_GET_LINEHANDLE_IOCTL");
        close(chip_fd);
        return -1;
    }
    close(chip_fd);             /* 行句柄持有后, chip fd 可关 */

    s_line_fd = req.fd;
    return 0;
}

void GPIO_Write(int value)
{
    if (s_line_fd < 0) {
        return;
    }
    struct gpiohandle_data data;
    data.values[0] = (value ? 1 : 0);
    ioctl(s_line_fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
}

void GPIO_Close(void)
{
    if (s_line_fd >= 0) {
        close(s_line_fd);
        s_line_fd = -1;
    }
}
