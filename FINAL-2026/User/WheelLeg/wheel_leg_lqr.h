#ifndef WHEEL_LEG_LQR_H
#define WHEEL_LEG_LQR_H

#include <stdint.h>

#define WHEEL_LEG_LQR_STATE_COUNT 6u
#define WHEEL_LEG_LQR_OUTPUT_COUNT 2u

/*
 * Calculate [T, Tp] = K (x_ref - x).  K and both states are supplied by the
 * caller; this module contains no nominal gain or dynamics assumptions.
 */
uint8_t WheelLeg_LQR_Calculate(
    const float gain[WHEEL_LEG_LQR_OUTPUT_COUNT][WHEEL_LEG_LQR_STATE_COUNT],
    const float x[WHEEL_LEG_LQR_STATE_COUNT],
    const float x_ref[WHEEL_LEG_LQR_STATE_COUNT],
    float output[WHEEL_LEG_LQR_OUTPUT_COUNT]);

#endif
