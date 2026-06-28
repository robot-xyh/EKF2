/********************************************************************************
 * File  : ubx_gps.c
 * Brief : UBX NAV-PVT parser. Returns one decoded fix when a valid frame lands.
 ********************************************************************************/
#include "ubx_gps.h"

#include <string.h>

#define UBX_NAV_CLASS   0x01
#define UBX_NAV_PVT     0x07
#define UBX_NAV_PVT_LEN 92

enum {
    S_SYNC1 = 0,
    S_SYNC2,
    S_CLASS,
    S_ID,
    S_LEN1,
    S_LEN2,
    S_PAYLOAD,
    S_CKA,
    S_CKB
};

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t get_i32(const uint8_t *p)
{
    return (int32_t)get_u32(p);
}

static void ck_add(ubx_parser_t *p, uint8_t b)
{
    p->ck_a = (uint8_t)(p->ck_a + b);
    p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
}

static int decode_nav_pvt(const uint8_t *d, uint16_t len, ubx_nav_pvt_t *out)
{
    if (len != UBX_NAV_PVT_LEN || out == 0) {
        return 0;
    }

    memset(out, 0, sizeof(*out));
    out->i_tow_ms     = get_u32(d + 0);
    out->fix_type     = d[20];
    out->flags        = d[21];
    out->num_sv       = d[23];
    out->lon_e7       = get_i32(d + 24);
    out->lat_e7       = get_i32(d + 28);
    out->height_mm    = get_i32(d + 32);
    out->h_msl_mm     = get_i32(d + 36);
    out->h_acc_mm     = get_u32(d + 40);
    out->v_acc_mm     = get_u32(d + 44);
    out->vel_n_mm_s   = get_i32(d + 48);
    out->vel_e_mm_s   = get_i32(d + 52);
    out->vel_d_mm_s   = get_i32(d + 56);
    out->g_speed_mm_s = get_u32(d + 60);
    out->s_acc_mm_s   = get_u32(d + 68);
    out->p_dop        = get_u16(d + 76);
    out->valid_3d     = ((out->flags & 0x01) && out->fix_type >= 3) ? 1 : 0;
    return 1;
}

void UBX_Init(ubx_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

int UBX_ParseByte(ubx_parser_t *p, uint8_t b, ubx_nav_pvt_t *out)
{
    switch (p->state) {
    case S_SYNC1:
        if (b == 0xB5) {
            p->state = S_SYNC2;
        }
        break;

    case S_SYNC2:
        p->state = (b == 0x62) ? S_CLASS : S_SYNC1;
        break;

    case S_CLASS:
        p->cls = b;
        p->ck_a = 0;
        p->ck_b = 0;
        ck_add(p, b);
        p->state = S_ID;
        break;

    case S_ID:
        p->id = b;
        ck_add(p, b);
        p->state = S_LEN1;
        break;

    case S_LEN1:
        p->len = b;
        ck_add(p, b);
        p->state = S_LEN2;
        break;

    case S_LEN2:
        p->len |= (uint16_t)b << 8;
        ck_add(p, b);
        p->idx = 0;
        if (p->len > sizeof(p->payload)) {
            p->state = S_SYNC1;
        } else {
            p->state = (p->len == 0) ? S_CKA : S_PAYLOAD;
        }
        break;

    case S_PAYLOAD:
        p->payload[p->idx++] = b;
        ck_add(p, b);
        if (p->idx >= p->len) {
            p->state = S_CKA;
        }
        break;

    case S_CKA:
        if (b == p->ck_a) {
            p->state = S_CKB;
        } else {
            p->state = S_SYNC1;
        }
        break;

    case S_CKB:
        p->state = S_SYNC1;
        if (b == p->ck_b && p->cls == UBX_NAV_CLASS && p->id == UBX_NAV_PVT) {
            return decode_nav_pvt(p->payload, p->len, out);
        }
        break;

    default:
        p->state = S_SYNC1;
        break;
    }

    return 0;
}

int UBX_ParseBuffer(ubx_parser_t *p, const uint8_t *buf, size_t len, ubx_nav_pvt_t *out)
{
    int decoded = 0;
    for (size_t i = 0; i < len; i++) {
        if (UBX_ParseByte(p, buf[i], out)) {
            decoded++;
        }
    }
    return decoded;
}
