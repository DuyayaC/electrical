#include "wheel_leg_vmc.h"

#include <float.h>

static uint8_t wheel_leg_vmc_is_finite(float value)
{
    return (value == value && value <= FLT_MAX && value >= -FLT_MAX) ?
           1u : 0u;
}

static uint8_t vmc_leg(const LegKinematicState *leg,
                       float force_N,
                       float pitch_torque_Nm,
                       float torque_Nm[2])
{
    if (torque_Nm == 0)
    {
        return 0u;
    }
    torque_Nm[0] = 0.0f;
    torque_Nm[1] = 0.0f;
    if (leg == 0 || leg->valid == 0u || leg->jacobian_valid == 0u ||
        wheel_leg_vmc_is_finite(leg->detJ) == 0u ||
        wheel_leg_vmc_is_finite(force_N) == 0u ||
        wheel_leg_vmc_is_finite(pitch_torque_Nm) == 0u ||
        wheel_leg_vmc_is_finite(leg->J[0][0]) == 0u ||
        wheel_leg_vmc_is_finite(leg->J[0][1]) == 0u ||
        wheel_leg_vmc_is_finite(leg->J[1][0]) == 0u ||
        wheel_leg_vmc_is_finite(leg->J[1][1]) == 0u)
    {
        return 0u;
    }

    /* Virtual work: tau = J^T [F, Tp]. */
    torque_Nm[0] = leg->J[0][0] * force_N +
                   leg->J[1][0] * pitch_torque_Nm;
    torque_Nm[1] = leg->J[0][1] * force_N +
                   leg->J[1][1] * pitch_torque_Nm;
    return 1u;
}

uint8_t WheelLeg_VMC_Calculate(
    const LegKinematicState leg_state[WL_WHEEL_COUNT],
    const float force_N[WL_WHEEL_COUNT],
    const float pitch_torque_Nm[WL_WHEEL_COUNT],
    float joint_torque_Nm[WL_JOINT_COUNT])
{
    uint8_t left_ok;
    uint8_t right_ok;

    if (joint_torque_Nm == 0)
    {
        return 0u;
    }
    joint_torque_Nm[0] = 0.0f;
    joint_torque_Nm[1] = 0.0f;
    joint_torque_Nm[2] = 0.0f;
    joint_torque_Nm[3] = 0.0f;
    if (leg_state == 0 || force_N == 0 || pitch_torque_Nm == 0)
    {
        return 0u;
    }

    left_ok = vmc_leg(&leg_state[WL_WHEEL_LEFT],
                      force_N[WL_WHEEL_LEFT],
                      pitch_torque_Nm[WL_WHEEL_LEFT],
                      &joint_torque_Nm[WL_JOINT_LEFT_Q1]);
    right_ok = vmc_leg(&leg_state[WL_WHEEL_RIGHT],
                       force_N[WL_WHEEL_RIGHT],
                       pitch_torque_Nm[WL_WHEEL_RIGHT],
                       &joint_torque_Nm[WL_JOINT_RIGHT_Q1]);
    if (left_ok == 0u || right_ok == 0u)
    {
        joint_torque_Nm[0] = 0.0f;
        joint_torque_Nm[1] = 0.0f;
        joint_torque_Nm[2] = 0.0f;
        joint_torque_Nm[3] = 0.0f;
        return 0u;
    }
    return 1u;
}
