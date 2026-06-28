/********************************************************************************
 * File  : ubx_test.c
 * Brief : Small UBX NAV-PVT parser self-test.
 ********************************************************************************/
#include "ubx_gps.h"

#include <stdio.h>
#include <string.h>

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static size_t make_nav_pvt(uint8_t *frame, uint8_t corrupt)
{
    uint8_t payload[92];
    memset(payload, 0, sizeof(payload));
    put_u32(payload + 0, 123456U);
    payload[20] = 3;
    payload[21] = 1;
    payload[23] = 12;
    put_u32(payload + 24, (uint32_t)(int32_t)-1224194000);
    put_u32(payload + 28, (uint32_t)(int32_t)377749000);
    put_u32(payload + 36, 15200U);
    put_u32(payload + 40, 900U);
    put_u32(payload + 44, 1300U);
    put_u32(payload + 48, 1200U);
    put_u32(payload + 52, (uint32_t)(int32_t)-340U);
    put_u32(payload + 56, 12U);
    put_u32(payload + 68, 80U);
    put_u16(payload + 76, 145U);

    frame[0] = 0xB5;
    frame[1] = 0x62;
    frame[2] = 0x01;
    frame[3] = 0x07;
    put_u16(frame + 4, sizeof(payload));
    memcpy(frame + 6, payload, sizeof(payload));

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (size_t i = 2; i < 6 + sizeof(payload); i++) {
        ck_a = (uint8_t)(ck_a + frame[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
    }
    frame[6 + sizeof(payload)] = ck_a;
    frame[7 + sizeof(payload)] = corrupt ? (uint8_t)(ck_b + 1U) : ck_b;
    return 8 + sizeof(payload);
}

int main(void)
{
    uint8_t frame[128];
    ubx_parser_t parser;
    ubx_nav_pvt_t fix;

    UBX_Init(&parser);
    size_t len = make_nav_pvt(frame, 0);
    int decoded = UBX_ParseBuffer(&parser, frame, len, &fix);
    if (decoded != 1 || !fix.valid_3d || fix.fix_type != 3 || fix.num_sv != 12) {
        fprintf(stderr, "valid NAV-PVT decode failed\n");
        return 1;
    }

    UBX_Init(&parser);
    len = make_nav_pvt(frame, 1);
    decoded = UBX_ParseBuffer(&parser, frame, len, &fix);
    if (decoded != 0) {
        fprintf(stderr, "checksum rejection failed\n");
        return 1;
    }

    printf("ubx_test: OK\n");
    return 0;
}
