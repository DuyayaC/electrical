#ifndef FIVE_BAR_H
#define FIVE_BAR_H

#include <stdint.h>

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

/* Kinematic result for one five-bar leg. */
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
} FiveBarState;

/* Explicit 2026 CAD defaults; the solver never selects them implicitly. */
uint8_t FiveBar_GetDefault2026Geometry(FiveBarGeometry *geometry);

/* Solve position, velocity, Jacobian, and validity from caller geometry. */
uint8_t FiveBar_Solve(const FiveBarGeometry *geometry,
                      float q1_rad,
                      float q4_rad,
                      float dq1_rad_s,
                      float dq4_rad_s,
                      FiveBarState *state);

/* Position-only helper for offline validation and host tests. */
uint8_t FiveBar_Position(const FiveBarGeometry *geometry,
                         float q1_rad,
                         float q4_rad,
                         float *length_m,
                         float *leg_axis_body_rad);

#endif
