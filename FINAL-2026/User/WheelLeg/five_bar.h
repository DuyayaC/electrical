#ifndef WHEEL_LEG_FIVE_BAR_H
#define WHEEL_LEG_FIVE_BAR_H

#include <stdint.h>

#include "wheel_leg_control_types.h"

typedef struct
{
    float l1_m;
    float l2_m;
    float l3_m;
    float l4_m;
    float base_width_m;
    float branch_sign;
    float finite_difference_step_rad;
    float triangle_tolerance_m;
    float min_length_m;
    float max_length_m;
    float min_abs_detJ;
} FiveBarGeometry;

/*
 * Solve position, velocity, Jacobian, and validity using caller-supplied
 * geometry.  The physical phi_bc relationship is deliberately not invented
 * by this solver; the estimator receives it explicitly from its caller.
 */
uint8_t FiveBar_Solve(const FiveBarGeometry *geometry,
                      float q1_rad,
                      float q4_rad,
                      float dq1_rad_s,
                      float dq4_rad_s,
                      LegKinematicState *state);

/* Position-only helper for offline validation and host tests. */
uint8_t FiveBar_Position(const FiveBarGeometry *geometry,
                         float q1_rad,
                         float q4_rad,
                         float *length_m,
                         float *leg_axis_body_rad);

#endif
