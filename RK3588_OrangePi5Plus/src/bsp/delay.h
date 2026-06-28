/********************************************************************************
 * File  : delay.h
 * Brief : 延时 / 单调时间戳 (替代 CH32 的 Delay_Ms)。
 ********************************************************************************/
#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>
#include <time.h>

static inline void Delay_Ms(uint32_t ms)
{
    struct timespec ts;
    ts.tv_sec  = ms / 1000u;
    ts.tv_nsec = (long)(ms % 1000u) * 1000000L;
    nanosleep(&ts, NULL);
}

static inline void Delay_Us(uint32_t us)
{
    struct timespec ts;
    ts.tv_sec  = us / 1000000u;
    ts.tv_nsec = (long)(us % 1000000u) * 1000L;
    nanosleep(&ts, NULL);
}

/* 单调时钟，单位秒(double)，用于 AHRS 的 dt */
static inline double Now_Sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

#endif /* __DELAY_H */
