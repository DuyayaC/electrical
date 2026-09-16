#include "wheel_leg_lqr.h"

#include <float.h>

static uint8_t wheel_leg_lqr_is_finite(float value)
{
    return (value == value && value <= FLT_MAX && value >= -FLT_MAX) ?
           1u : 0u;
}

uint8_t WheelLegLQR_Calculate(
    const float gain[WHEEL_LEG_LQR_OUTPUT_COUNT]
                [WHEEL_LEG_LQR_STATE_COUNT],
    const float x[WHEEL_LEG_LQR_STATE_COUNT],
    const float x_ref[WHEEL_LEG_LQR_STATE_COUNT],
    float output[WHEEL_LEG_LQR_OUTPUT_COUNT])
{
    float error[WHEEL_LEG_LQR_STATE_COUNT];
    uint8_t row;
    uint8_t column;

    if (output == 0)
    {
        return 0u;
    }
    output[0] = 0.0f;
    output[1] = 0.0f;
    if (gain == 0 || x == 0 || x_ref == 0)
    {
        return 0u;
    }
    for (column = 0u; column < WHEEL_LEG_LQR_STATE_COUNT; column++)
    {
        if (wheel_leg_lqr_is_finite(x[column]) == 0u ||
            wheel_leg_lqr_is_finite(x_ref[column]) == 0u)
        {
            return 0u;
        }
        error[column] = x_ref[column] - x[column];
    }
    for (row = 0u; row < WHEEL_LEG_LQR_OUTPUT_COUNT; row++)
    {
        for (column = 0u; column < WHEEL_LEG_LQR_STATE_COUNT; column++)
        {
            if (wheel_leg_lqr_is_finite(gain[row][column]) == 0u)
            {
                output[0] = 0.0f;
                output[1] = 0.0f;
                return 0u;
            }
            output[row] += gain[row][column] * error[column];
        }
    }
    return 1u;
}
