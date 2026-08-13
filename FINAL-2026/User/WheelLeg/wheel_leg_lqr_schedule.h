#ifndef WHEEL_LEG_LQR_SCHEDULE_H
#define WHEEL_LEG_LQR_SCHEDULE_H

#include <stdint.h>

#include "wheel_leg_lqr.h"

/*
 * Runtime interpolation for an offline-generated K(L0) table.  The table is
 * supplied by the application so this core module remains hardware-agnostic.
 */
typedef struct
{
    uint32_t count;
    const float *l0_m;
    const float (*gain)[WHEEL_LEG_LQR_OUTPUT_COUNT]
                      [WHEEL_LEG_LQR_STATE_COUNT];
} WheelLegLqrSchedule;

uint8_t WheelLeg_LQR_ScheduleInterpolate(
    const WheelLegLqrSchedule *schedule,
    float l0_m,
    float gain[WHEEL_LEG_LQR_OUTPUT_COUNT][WHEEL_LEG_LQR_STATE_COUNT]);

#endif
