#include "vmc.h"

#include <float.h>

static uint8_t vmc_is_finite(float value)
{
    return (value == value && value <= FLT_MAX && value >= -FLT_MAX) ?
           1u : 0u;
}

uint8_t VMC_Calculate2D(
    const float jacobian[2][2],
    float force_N,
    float pitch_torque_Nm,
    float joint_torque_Nm[2])
{
    if (joint_torque_Nm == 0)
    {
        return 0u;
    }
    joint_torque_Nm[0] = 0.0f;
    joint_torque_Nm[1] = 0.0f;
    if (jacobian == 0 ||
        vmc_is_finite(force_N) == 0u ||
        vmc_is_finite(pitch_torque_Nm) == 0u ||
        vmc_is_finite(jacobian[0][0]) == 0u ||
        vmc_is_finite(jacobian[0][1]) == 0u ||
        vmc_is_finite(jacobian[1][0]) == 0u ||
        vmc_is_finite(jacobian[1][1]) == 0u)
    {
        return 0u;
    }

    joint_torque_Nm[0] = jacobian[0][0] * force_N +
                         jacobian[1][0] * pitch_torque_Nm;
    joint_torque_Nm[1] = jacobian[0][1] * force_N +
                         jacobian[1][1] * pitch_torque_Nm;
    return 1u;
}
