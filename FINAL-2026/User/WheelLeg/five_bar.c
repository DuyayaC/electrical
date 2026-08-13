#include "five_bar.h"

#include <float.h>
#include <math.h>
#include <string.h>

#ifndef WL_PI_F
#define WL_PI_F 3.14159265358979323846f
#endif

typedef struct
{
    float x;
    float y;
} FiveBarPoint;

static uint8_t five_bar_is_finite(float value)
{
    return (value == value && value <= FLT_MAX && value >= -FLT_MAX) ?
           1u : 0u;
}

static float five_bar_wrap_angle(float angle_rad)
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

static uint8_t five_bar_geometry_valid(const FiveBarGeometry *geometry)
{
    if (geometry == 0 ||
        five_bar_is_finite(geometry->l1_m) == 0u ||
        five_bar_is_finite(geometry->l2_m) == 0u ||
        five_bar_is_finite(geometry->l3_m) == 0u ||
        five_bar_is_finite(geometry->l4_m) == 0u ||
        five_bar_is_finite(geometry->base_width_m) == 0u ||
        five_bar_is_finite(geometry->branch_sign) == 0u ||
        five_bar_is_finite(geometry->finite_difference_step_rad) == 0u ||
        five_bar_is_finite(geometry->triangle_tolerance_m) == 0u ||
        five_bar_is_finite(geometry->min_length_m) == 0u ||
        five_bar_is_finite(geometry->max_length_m) == 0u ||
        five_bar_is_finite(geometry->min_abs_detJ) == 0u ||
        geometry->l1_m <= 0.0f || geometry->l2_m <= 0.0f ||
        geometry->l3_m <= 0.0f || geometry->l4_m <= 0.0f ||
        geometry->base_width_m <= 0.0f ||
        geometry->branch_sign == 0.0f ||
        geometry->finite_difference_step_rad <= 0.0f ||
        geometry->triangle_tolerance_m <= 0.0f ||
        geometry->min_length_m < 0.0f ||
        geometry->max_length_m < geometry->min_length_m ||
        geometry->min_abs_detJ <= 0.0f)
    {
        return 0u;
    }
    return 1u;
}

uint8_t FiveBar_CadGeometry(FiveBarGeometry *geometry)
{
    if (geometry == 0)
    {
        return 0u;
    }
    geometry->l1_m = 0.145f;
    geometry->l2_m = 0.270f;
    geometry->l3_m = 0.270f;
    geometry->l4_m = 0.145f;
    geometry->base_width_m = 0.150f;
    geometry->branch_sign = -1.0f;
    geometry->finite_difference_step_rad = 0.0001f;
    geometry->triangle_tolerance_m = 0.000001f;
    geometry->min_length_m = 0.1042f;
    geometry->max_length_m = 0.3672f;
    geometry->min_abs_detJ = 0.00001f;
    return 1u;
}

static uint8_t five_bar_position_internal(const FiveBarGeometry *geometry,
                                          float q1_rad,
                                          float q4_rad,
                                          float *length_m,
                                          float *leg_axis_body_rad)
{
    FiveBarPoint A;
    FiveBarPoint E;
    FiveBarPoint B;
    FiveBarPoint D;
    FiveBarPoint C;
    FiveBarPoint O;
    FiveBarPoint BD;
    FiveBarPoint unit;
    FiveBarPoint perpendicular;
    FiveBarPoint base_point;
    float distance;
    float projection;
    float height_squared;
    float height;
    float dx;
    float dy;
    float radius;

    if (length_m == 0 || leg_axis_body_rad == 0 ||
        five_bar_geometry_valid(geometry) == 0u ||
        five_bar_is_finite(q1_rad) == 0u ||
        five_bar_is_finite(q4_rad) == 0u)
    {
        return 0u;
    }

    A.x = 0.0f;
    A.y = 0.0f;
    E.x = geometry->base_width_m;
    E.y = 0.0f;

    B.x = A.x + geometry->l1_m * cosf(q1_rad);
    B.y = A.y + geometry->l1_m * sinf(q1_rad);
    D.x = E.x + geometry->l4_m * cosf(q4_rad);
    D.y = E.y + geometry->l4_m * sinf(q4_rad);

    BD.x = D.x - B.x;
    BD.y = D.y - B.y;
    distance = sqrtf(BD.x * BD.x + BD.y * BD.y);
    if (distance <= geometry->triangle_tolerance_m)
    {
        return 0u;
    }

    /* Intersection of circles (B,l2) and (D,l3). */
    projection = (geometry->l2_m * geometry->l2_m -
                  geometry->l3_m * geometry->l3_m + distance * distance) /
                 (2.0f * distance);
    height_squared = geometry->l2_m * geometry->l2_m - projection * projection;
    if (height_squared < -geometry->triangle_tolerance_m)
    {
        return 0u;
    }
    if (height_squared < 0.0f)
    {
        height_squared = 0.0f;
    }

    unit.x = BD.x / distance;
    unit.y = BD.y / distance;
    base_point.x = B.x + projection * unit.x;
    base_point.y = B.y + projection * unit.y;
    perpendicular.x = -unit.y;
    perpendicular.y = unit.x;
    height = sqrtf(height_squared);

    C.x = base_point.x + geometry->branch_sign * height * perpendicular.x;
    C.y = base_point.y + geometry->branch_sign * height * perpendicular.y;
    O.x = geometry->base_width_m * 0.5f;
    O.y = 0.0f;

    dx = C.x - O.x;
    dy = C.y - O.y;
    radius = sqrtf(dx * dx + dy * dy);
    if (radius < geometry->min_length_m || radius > geometry->max_length_m)
    {
        return 0u;
    }

    *length_m = radius;
    *leg_axis_body_rad = atan2f(dx, dy);
    return 1u;
}

uint8_t FiveBar_Position(const FiveBarGeometry *geometry,
                         float q1_rad,
                         float q4_rad,
                         float *length_m,
                         float *leg_axis_body_rad)
{
    return five_bar_position_internal(geometry,
                                      q1_rad,
                                      q4_rad,
                                      length_m,
                                      leg_axis_body_rad);
}

uint8_t FiveBar_Solve(const FiveBarGeometry *geometry,
                      float q1_rad,
                      float q4_rad,
                      float dq1_rad_s,
                      float dq4_rad_s,
                      LegKinematicState *state)
{
    float length_plus_q1;
    float length_minus_q1;
    float angle_plus_q1;
    float angle_minus_q1;
    float length_plus_q4;
    float length_minus_q4;
    float angle_plus_q4;
    float angle_minus_q4;
    float step;

    if (state == 0)
    {
        return 0u;
    }
    memset(state, 0, sizeof(*state));
    if (five_bar_geometry_valid(geometry) == 0u ||
        five_bar_is_finite(q1_rad) == 0u ||
        five_bar_is_finite(q4_rad) == 0u ||
        five_bar_is_finite(dq1_rad_s) == 0u ||
        five_bar_is_finite(dq4_rad_s) == 0u)
    {
        return 0u;
    }

    state->q1_rad = q1_rad;
    state->q4_rad = q4_rad;
    state->dq1_rad_s = dq1_rad_s;
    state->dq4_rad_s = dq4_rad_s;

    if (FiveBar_Position(geometry, q1_rad, q4_rad,
                         &state->L0_m, &state->leg_axis_body_rad) == 0u)
    {
        return 0u;
    }

    step = geometry->finite_difference_step_rad;
    if (FiveBar_Position(geometry, q1_rad + step, q4_rad,
                         &length_plus_q1, &angle_plus_q1) == 0u ||
        FiveBar_Position(geometry, q1_rad - step, q4_rad,
                         &length_minus_q1, &angle_minus_q1) == 0u ||
        FiveBar_Position(geometry, q1_rad, q4_rad + step,
                         &length_plus_q4, &angle_plus_q4) == 0u ||
        FiveBar_Position(geometry, q1_rad, q4_rad - step,
                         &length_minus_q4, &angle_minus_q4) == 0u)
    {
        memset(state, 0, sizeof(*state));
        return 0u;
    }

    state->J[0][0] = (length_plus_q1 - length_minus_q1) / (2.0f * step);
    state->J[0][1] = (length_plus_q4 - length_minus_q4) / (2.0f * step);
    state->J[1][0] = five_bar_wrap_angle(angle_plus_q1 - angle_minus_q1) /
                     (2.0f * step);
    state->J[1][1] = five_bar_wrap_angle(angle_plus_q4 - angle_minus_q4) /
                     (2.0f * step);
    state->detJ = state->J[0][0] * state->J[1][1] -
                  state->J[0][1] * state->J[1][0];
    state->dL0_m_s = state->J[0][0] * dq1_rad_s +
                     state->J[0][1] * dq4_rad_s;
    state->dleg_axis_body_rad_s = state->J[1][0] * dq1_rad_s +
                                  state->J[1][1] * dq4_rad_s;
    if (five_bar_is_finite(state->detJ) == 0u ||
        five_bar_is_finite(state->dL0_m_s) == 0u ||
        five_bar_is_finite(state->dleg_axis_body_rad_s) == 0u)
    {
        memset(state, 0, sizeof(*state));
        return 0u;
    }

    state->valid = 1u;
    state->jacobian_valid =
        (fabsf(state->detJ) >= geometry->min_abs_detJ) ? 1u : 0u;
    return state->jacobian_valid;
}
