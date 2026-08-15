#ifndef WHEEL_LEG_LQR_H
#define WHEEL_LEG_LQR_H

#include <stdint.h>

#define WHEEL_LEG_LQR_STATE_COUNT 6u
#define WHEEL_LEG_LQR_OUTPUT_COUNT 2u

/*
 * Calculate [T, Tp] = K (x_ref - x).
 *
 * State order:
 *   [theta, dtheta, xb, dxb, phi, dphi]
 *
 * The gain matrix and both state vectors are supplied by the caller.  This
 * module has no hardware, sensor, or dynamics dependency.
 */
uint8_t WheelLegLQR_Calculate(
    const float gain[WHEEL_LEG_LQR_OUTPUT_COUNT]
                [WHEEL_LEG_LQR_STATE_COUNT],
    const float x[WHEEL_LEG_LQR_STATE_COUNT],
    const float x_ref[WHEEL_LEG_LQR_STATE_COUNT],
    float output[WHEEL_LEG_LQR_OUTPUT_COUNT]);

#endif
