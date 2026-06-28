/********************************************************************************
 * File  : altitude.c
 * Brief : h = 44330 * (1 - (p/p0)^(1/5.255))
 ********************************************************************************/
#include "altitude.h"
#include <math.h>

#define SEA_LEVEL_PA    101325.0f

static float ground_pa = SEA_LEVEL_PA;

float Altitude_FromPressure(float pressure_pa)
{
    if (pressure_pa <= 0.0f) {
        return 0.0f;
    }
    return 44330.0f * (1.0f - powf(pressure_pa / SEA_LEVEL_PA, 1.0f / 5.255f));
}

void Altitude_SetGround(float pressure_pa)
{
    if (pressure_pa > 0.0f) {
        ground_pa = pressure_pa;
    }
}

float Altitude_Relative(float pressure_pa)
{
    if (pressure_pa <= 0.0f) {
        return 0.0f;
    }
    return 44330.0f * (1.0f - powf(pressure_pa / ground_pa, 1.0f / 5.255f));
}
