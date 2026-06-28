/********************************************************************************
 * File  : i2c.c
 * Brief : 用 I2C_RDWR ioctl 做 重复起始(write reg + read) 的可靠读写。
 ********************************************************************************/
#include "i2c.h"

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

static int s_fd = -1;

int I2C_Open(const char *dev)
{
    s_fd = open(dev, O_RDWR);
    if (s_fd < 0) {
        perror("[i2c] open");
        return -1;
    }
    return 0;
}

void I2C_Close(void)
{
    if (s_fd >= 0) {
        close(s_fd);
        s_fd = -1;
    }
}

int I2C_ReadRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (s_fd < 0) {
        return 0;
    }
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data xfer;

    msgs[0].addr  = addr;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;

    msgs[1].addr  = addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = len;
    msgs[1].buf   = buf;

    xfer.msgs  = msgs;
    xfer.nmsgs = 2;

    return (ioctl(s_fd, I2C_RDWR, &xfer) >= 0) ? 1 : 0;
}

int I2C_ReadReg(uint8_t addr, uint8_t reg, uint8_t *val)
{
    return I2C_ReadRegs(addr, reg, val, 1);
}

int I2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t val)
{
    if (s_fd < 0) {
        return 0;
    }
    uint8_t out[2] = { reg, val };
    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data xfer;

    msg.addr  = addr;
    msg.flags = 0;
    msg.len   = 2;
    msg.buf   = out;

    xfer.msgs  = &msg;
    xfer.nmsgs = 1;

    return (ioctl(s_fd, I2C_RDWR, &xfer) >= 0) ? 1 : 0;
}

int I2C_Probe(uint8_t addr)
{
    if (s_fd < 0) {
        return 0;
    }
    /* 0 字节写探测; 部分控制器不支持 0 长度, 退而读 1 字节 */
    uint8_t dummy = 0;
    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data xfer;

    msg.addr  = addr;
    msg.flags = I2C_M_RD;
    msg.len   = 1;
    msg.buf   = &dummy;

    xfer.msgs  = &msg;
    xfer.nmsgs = 1;

    return (ioctl(s_fd, I2C_RDWR, &xfer) >= 0) ? 1 : 0;
}
