#include "wheel_leg_state_estimator.h"

#include <float.h>
#include <math.h>
#include <string.h>

#ifndef WL_PI_F
#define WL_PI_F 3.14159265358979323846f
#endif

static uint8_t wheel_leg_is_finite(float value)
{
    return (value == value && value <= FLT_MAX && value >= -FLT_MAX) ?
           1u : 0u;
}

static float wheel_leg_wrap_angle(float angle_rad)
{
    while (angle_rad > WL_PI_F)
    {
        angle_rad -= 2.0f * WL_PI_F;
    }
    while (angle_rad < -WL_PI_F)
    {
        angle_rad += 2.0f * WL_PI_F;
    }
    return angle_rad;
}

static uint8_t wheel_leg_kinematics_valid(
    const LegKinematicState *leg)
{
    uint8_t row;
    uint8_t column;

    if (leg == 0 || leg->valid == 0u || leg->jacobian_valid == 0u ||
        wheel_leg_is_finite(leg->L0_m) == 0u ||
        wheel_leg_is_finite(leg->leg_axis_body_rad) == 0u ||
        wheel_leg_is_finite(leg->dL0_m_s) == 0u ||
        wheel_leg_is_finite(leg->dleg_axis_body_rad_s) == 0u ||
        wheel_leg_is_finite(leg->detJ) == 0u)
    {
        return 0u;
    }
    for (row = 0u; row < 2u; row++)
    {
        for (column = 0u; column < 2u; column++)
        {
            if (wheel_leg_is_finite(leg->J[row][column]) == 0u)
            {
                return 0u;
            }
        }
    }
    return 1u;
}

void WheelLeg_StateEstimator_Reset(WheelLegStateEstimator *estimator)
{
    if (estimator != 0)
    {
        memset(estimator, 0, sizeof(*estimator));
    }
}

uint8_t WheelLeg_StateEstimator_Update(
    WheelLegStateEstimator *estimator,
    const WheelLegEstimatorInput *input,
    const WheelLegEstimatorParams *params,
    const LegKinematicState leg_state[WL_WHEEL_COUNT],
    WheelLegEstimate *estimate)
{
    float theta_body;
    float dtheta_body;
    float theta_side;
    float dtheta_side;
    float dxb_side[WL_WHEEL_COUNT];
    uint8_t i;

    if (estimate == 0)
    {
        return 0u;
    }
    memset(estimate, 0, sizeof(*estimate));
    if (estimator == 0 || input == 0 || params == 0 || leg_state == 0 ||
        params->wheel_radius_m <= 0.0f || params->dt_s <= 0.0f ||
        params->dt_s >= 0.1f ||
        wheel_leg_is_finite(params->wheel_radius_m) == 0u ||
        wheel_leg_is_finite(params->dt_s) == 0u ||
        input->body_attitude_valid == 0u ||
        wheel_leg_is_finite(input->body_pitch_rad) == 0u ||
        wheel_leg_is_finite(input->body_pitch_rate_rad_s) == 0u)
    {
        return 0u;
    }

    for (i = 0u; i < WL_WHEEL_COUNT; i++)
    {
        if (input->wheel_velocity_valid[i] == 0u ||
            input->phi_bc_valid[i] == 0u ||
            wheel_leg_kinematics_valid(&leg_state[i]) == 0u ||
            wheel_leg_is_finite(input->wheel_output_velocity_rad_s[i]) == 0u ||
            wheel_leg_is_finite(input->phi_bc_rad[i]) == 0u ||
            wheel_leg_is_finite(input->dphi_bc_rad_s[i]) == 0u)
        {
            return 0u;
        }
    }

    theta_body = 0.5f * (leg_state[WL_WHEEL_LEFT].leg_axis_body_rad +
                         leg_state[WL_WHEEL_RIGHT].leg_axis_body_rad);
    dtheta_body = 0.5f * (leg_state[WL_WHEEL_LEFT].dleg_axis_body_rad_s +
                          leg_state[WL_WHEEL_RIGHT].dleg_axis_body_rad_s);
    estimate->theta_rad =
        wheel_leg_wrap_angle(theta_body + input->body_pitch_rad);
    estimate->dtheta_rad_s = dtheta_body + input->body_pitch_rate_rad_s;
    estimate->phi_rad = input->body_pitch_rad;
    estimate->dphi_rad_s = input->body_pitch_rate_rad_s;

    for (i = 0u; i < WL_WHEEL_COUNT; i++)
    {
        theta_side = leg_state[i].leg_axis_body_rad + input->body_pitch_rad;
        dtheta_side = leg_state[i].dleg_axis_body_rad_s +
                      input->body_pitch_rate_rad_s;
        /*
         * This is the paper's wheel-ground velocity relation.  phi_bc is
         * supplied explicitly because its physical c-link definition still
         * needs CAD confirmation; no leg-angle proxy is used here.
         */
        dxb_side[i] =
            (input->wheel_output_velocity_rad_s[i] +
             input->dphi_bc_rad_s[i] +
             input->body_pitch_rate_rad_s) * params->wheel_radius_m +
            leg_state[i].L0_m * dtheta_side * cosf(theta_side) +
            leg_state[i].dL0_m_s * sinf(theta_side);
    }

    estimate->xb_m = estimator->xb_m;
    estimate->dxb_m_s = 0.5f * (dxb_side[WL_WHEEL_LEFT] +
                                dxb_side[WL_WHEEL_RIGHT]);
    if (estimator->initialized == 0u)
    {
        estimator->xb_m = 0.0f;
        estimator->initialized = 1u;
    }
    else
    {
        estimator->xb_m += estimate->dxb_m_s * params->dt_s;
    }
    estimate->xb_m = estimator->xb_m;
    estimate->x[0] = estimate->theta_rad;
    estimate->x[1] = estimate->dtheta_rad_s;
    estimate->x[2] = estimate->xb_m;
    estimate->x[3] = estimate->dxb_m_s;
    estimate->x[4] = estimate->phi_rad;
    estimate->x[5] = estimate->dphi_rad_s;
    estimate->valid = 1u;
    return 1u;
}
