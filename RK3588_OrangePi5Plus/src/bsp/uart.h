/********************************************************************************
 * File  : uart.h
 * Brief : Small POSIX UART wrapper for GPS input.
 ********************************************************************************/
#ifndef __UART_H
#define __UART_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

int     UART_Open(const char *dev, int baud);
void    UART_Close(int fd);
ssize_t UART_Read(int fd, uint8_t *buf, size_t len);
ssize_t UART_Write(int fd, const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* __UART_H */
