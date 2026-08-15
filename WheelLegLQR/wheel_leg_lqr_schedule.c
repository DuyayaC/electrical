#include "wheel_leg_lqr_schedule.h"

#include <float.h>

static uint8_t wheel_leg_lqr_schedule_is_finite(float value)
{
    return (value == value && value <= FLT_MAX && value >= -FLT_MAX) ?
           1u : 0u;
}

uint8_t WheelLegLQR_ScheduleInterpolate(
    const WheelLegLQRSchedule *schedule,
    float l0_m,
    float gain[WHEEL_LEG_LQR_OUTPUT_COUNT]
                [WHEEL_LEG_LQR_STATE_COUNT])
{
    uint32_t lower;
    uint32_t upper;
    float alpha;
    uint32_t row;
    uint32_t column;

    if (schedule == 0 || gain == 0 || schedule->count == 0u ||
        schedule->l0_m == 0 || schedule->gain == 0 ||
        wheel_leg_lqr_schedule_is_finite(l0_m) == 0u)
    {
        return 0u;
    }
    if (schedule->count == 1u)
    {
        lower = 0u;
        upper = 0u;
    }
    else if (l0_m <= schedule->l0_m[0])
    {
        lower = 0u;
        upper = 0u;
    }
    else if (l0_m >= schedule->l0_m[schedule->count - 1u])
    {
        lower = schedule->count - 1u;
        upper = lower;
    }
    else
    {
        lower = 0u;
        while (lower + 1u < schedule->count &&
               l0_m > schedule->l0_m[lower + 1u])
        {
            lower++;
        }
        upper = lower + 1u;
        if (schedule->l0_m[upper] <= schedule->l0_m[lower])
        {
            return 0u;
        }
    }
    alpha = (upper == lower) ? 0.0f :
            (l0_m - schedule->l0_m[lower]) /
            (schedule->l0_m[upper] - schedule->l0_m[lower]);
    for (row = 0u; row < WHEEL_LEG_LQR_OUTPUT_COUNT; row++)
    {
        for (column = 0u; column < WHEEL_LEG_LQR_STATE_COUNT; column++)
        {
            float low = schedule->gain[lower][row][column];
            float high = schedule->gain[upper][row][column];
            if (wheel_leg_lqr_schedule_is_finite(low) == 0u ||
                wheel_leg_lqr_schedule_is_finite(high) == 0u)
            {
                return 0u;
            }
            gain[row][column] = low + alpha * (high - low);
        }
    }
    return 1u;
}
