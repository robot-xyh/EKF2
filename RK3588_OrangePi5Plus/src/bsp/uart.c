/********************************************************************************
 * File  : uart.c
 * Brief : Nonblocking raw UART setup for GPS receivers.
 ********************************************************************************/
#include "uart.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B115200;
    }
}

int UART_Open(const char *dev, int baud)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("[uart] open");
        return -1;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) {
        perror("[uart] tcgetattr");
        close(fd);
        return -1;
    }

    cfmakeraw(&tio);
    speed_t speed = baud_to_speed(baud);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        perror("[uart] tcsetattr");
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

void UART_Close(int fd)
{
    if (fd >= 0) {
        close(fd);
    }
}

ssize_t UART_Read(int fd, uint8_t *buf, size_t len)
{
    ssize_t ret = read(fd, buf, len);
    if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return 0;
    }
    return ret;
}

ssize_t UART_Write(int fd, const uint8_t *buf, size_t len)
{
    return write(fd, buf, len);
}
