/********************************************************************************
 * File  : ubx_gps.h
 * Brief : UBX NAV-PVT stream parser for u-blox GPS receivers.
 ********************************************************************************/
#ifndef __UBX_GPS_H
#define __UBX_GPS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t i_tow_ms;
    uint8_t fix_type;
    uint8_t flags;
    uint8_t num_sv;
    int32_t lon_e7;
    int32_t lat_e7;
    int32_t height_mm;
    int32_t h_msl_mm;
    uint32_t h_acc_mm;
    uint32_t v_acc_mm;
    int32_t vel_n_mm_s;
    int32_t vel_e_mm_s;
    int32_t vel_d_mm_s;
    uint32_t g_speed_mm_s;
    uint32_t s_acc_mm_s;
    uint16_t p_dop;
    uint8_t valid_3d;
} ubx_nav_pvt_t;

typedef struct {
    uint8_t state;
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    uint16_t idx;
    uint8_t ck_a;
    uint8_t ck_b;
    uint8_t payload[100];
} ubx_parser_t;

void UBX_Init(ubx_parser_t *p);
int  UBX_ParseByte(ubx_parser_t *p, uint8_t b, ubx_nav_pvt_t *out);
int  UBX_ParseBuffer(ubx_parser_t *p, const uint8_t *buf, size_t len, ubx_nav_pvt_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __UBX_GPS_H */
