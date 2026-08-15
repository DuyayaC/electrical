#ifndef WHEEL_LEG_LQR_SCHEDULE_H
#define WHEEL_LEG_LQR_SCHEDULE_H

#include <stdint.h>

#include "wheel_leg_lqr.h"

/* Runtime interpolation table for K(L0). */
typedef struct
{
    uint32_t count;
    const float *l0_m;
    const float (*gain)[WHEEL_LEG_LQR_OUTPUT_COUNT]
                      [WHEEL_LEG_LQR_STATE_COUNT];
} WheelLegLQRSchedule;

uint8_t WheelLegLQR_ScheduleInterpolate(
    const WheelLegLQRSchedule *schedule,
    float l0_m,
    float gain[WHEEL_LEG_LQR_OUTPUT_COUNT]
                [WHEEL_LEG_LQR_STATE_COUNT]);

#endif
