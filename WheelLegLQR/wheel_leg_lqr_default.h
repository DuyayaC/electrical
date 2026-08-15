#ifndef WHEEL_LEG_LQR_DEFAULT_H
#define WHEEL_LEG_LQR_DEFAULT_H

#include "wheel_leg_lqr_schedule.h"

/*
 * Return the explicitly selected 2026 wheel-leg schedule.
 * The returned table has static storage and must not be modified.
 */
const WheelLegLQRSchedule *WheelLegLQR_GetDefault2026Schedule(void);

#endif
