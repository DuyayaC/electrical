#ifndef WHEEL_LEG_CONTROL_TYPES_H
#define WHEEL_LEG_CONTROL_TYPES_H

#include <stdint.h>

#include "wheel_leg_types.h"

/*
 * Five-bar output shared by kinematics, the state estimator, and VMC.
 * leg_axis_body_rad is measured from body +Y (upright), so upright is zero.
 */
typedef struct
{
    float q1_rad;
    float q4_rad;
    float dq1_rad_s;
    float dq4_rad_s;
    float L0_m;
    float leg_axis_body_rad;
    float dL0_m_s;
    float dleg_axis_body_rad_s;
    float J[2][2];
    float detJ;
    uint8_t valid;
    uint8_t jacobian_valid;
} LegKinematicState;

/* LQR state order: [theta, dtheta, xb, dxb, phi, dphi]. */
typedef struct
{
    float theta_rad;
    float dtheta_rad_s;
    float xb_m;
    float dxb_m_s;
    float phi_rad;
    float dphi_rad_s;
    float x[6];
    uint8_t valid;
} WheelLegEstimate;

#endif
