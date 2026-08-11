#ifndef WHEEL_LEG_VMC_H
#define WHEEL_LEG_VMC_H

#include <stdint.h>

#include "wheel_leg_control_types.h"

/*
 * Map caller-supplied virtual forces [F_l, F_r] and pitch torques
 * [Tp_l, Tp_r] to [left_q1, left_q4, right_q1, right_q4].
 */
uint8_t WheelLeg_VMC_Calculate(
    const LegKinematicState leg_state[WL_WHEEL_COUNT],
    const float force_N[WL_WHEEL_COUNT],
    const float pitch_torque_Nm[WL_WHEEL_COUNT],
    float joint_torque_Nm[WL_JOINT_COUNT]);

#endif
