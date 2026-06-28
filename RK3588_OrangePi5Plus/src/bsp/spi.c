/********************************************************************************
 * File  : spi.c
 * Brief : spidev 封装。片选用软件 GPIO(见 gpio.c), 故 mode 加 SPI_NO_CS,
 *         让内核不去 toggle 硬件 CS(那两脚被小板用作 IMUCLK/DRDY)。
 ********************************************************************************/
#include "spi.h"
#include "gpio.h"
#include "bsp.h"

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

static int      s_fd  = -1;
static uint32_t s_hz  = IMU_SPI_HZ_SLOW;
static uint8_t  s_bits = IMU_SPI_BITS;

int SPI_Open(const char *dev, uint8_t mode, uint8_t bits, uint32_t hz)
{
    s_fd = open(dev, O_RDWR);
    if (s_fd < 0) {
        perror("[spi] open");
        return -1;
    }

    /* 注意: RK3588 rockchip-spi 不支持 SPI_NO_CS, 故不加该标志。
     * 片选仍由软件 GPIO1_A4 负责(见 SPI_Transfer); 硬件 CS0(Pin24)虽会被
     * spidev toggle, 但它接的是 IMUCLK 网络(IMU 时钟输入, 默认内部时钟忽略),
     * 不影响通信。 */
    uint32_t m = mode;
    if (ioctl(s_fd, SPI_IOC_WR_MODE32, &m) < 0) {
        uint8_t m8 = (uint8_t)mode;
        if (ioctl(s_fd, SPI_IOC_WR_MODE, &m8) < 0) {
            perror("[spi] set mode");
            return -1;
        }
    }
    if (ioctl(s_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        perror("[spi] set bits");
        return -1;
    }
    if (ioctl(s_fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) {
        perror("[spi] set speed");
        return -1;
    }
    s_bits = bits;
    s_hz   = hz;
    return 0;
}

void SPI_SetSpeed(uint32_t hz)
{
    s_hz = hz;
    if (s_fd >= 0) {
        ioctl(s_fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz);
    }
}

void SPI_Close(void)
{
    if (s_fd >= 0) {
        close(s_fd);
        s_fd = -1;
    }
}

int SPI_Transfer(const uint8_t *tx, uint8_t *rx, int len)
{
    if (s_fd < 0) {
        return -1;
    }

    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf        = (unsigned long)tx;
    tr.rx_buf        = (unsigned long)rx;
    tr.len           = (uint32_t)len;
    tr.speed_hz      = s_hz;
    tr.bits_per_word = s_bits;

#if !IMU_CS_KERNEL_MANAGED
    GPIO_Write(0);                                  /* CS 低 (方案A 软件片选) */
    int ret = ioctl(s_fd, SPI_IOC_MESSAGE(1), &tr);
    GPIO_Write(1);                                  /* CS 高 */
#else
    int ret = ioctl(s_fd, SPI_IOC_MESSAGE(1), &tr); /* 方案B: 内核自动片选 */
#endif

    return (ret < 1) ? -1 : 0;
}

uint8_t SPI_ReadReg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg | ICM40608_SPI_READ), 0x00 };
    uint8_t rx[2] = { 0, 0 };
    SPI_Transfer(tx, rx, 2);
    return rx[1];
}

void SPI_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
    uint8_t rx[2] = { 0, 0 };
    SPI_Transfer(tx, rx, 2);
}

void SPI_ReadBuf(uint8_t reg, uint8_t *buf, int len)
{
    uint8_t tx[32] = { 0 };
    uint8_t rx[32] = { 0 };
    if (len > 31) {
        len = 31;
    }
    tx[0] = (uint8_t)(reg | ICM40608_SPI_READ);
    SPI_Transfer(tx, rx, len + 1);
    memcpy(buf, rx + 1, (size_t)len);
}
