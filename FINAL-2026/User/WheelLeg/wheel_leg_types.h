#ifndef WHEEL_LEG_TYPES_H
#define WHEEL_LEG_TYPES_H

#include <stdint.h>

typedef enum
{
    WL_WHEEL_LEFT = 0,
    WL_WHEEL_RIGHT = 1,
    WL_WHEEL_COUNT = 2
} WheelLegWheelIndex;

typedef enum
{
    WL_JOINT_LEFT_Q1 = 0,
    WL_JOINT_LEFT_Q4 = 1,
    WL_JOINT_RIGHT_Q1 = 2,
    WL_JOINT_RIGHT_Q4 = 3,
    WL_JOINT_COUNT = 4
} WheelLegJointIndex;

/*
 * Hardware-independent estimator input.
 *
 * The adapter above this layer must convert encoder and IMU data into these
 * SI quantities.  No CAN identifiers, raw counts, RPM, sensor axis mapping,
 * or motor command fields belong here.
 */
typedef struct
{
    float wheel_output_velocity_rad_s[WL_WHEEL_COUNT];
    float body_pitch_rad;
    float body_pitch_rate_rad_s;
    float phi_bc_rad[WL_WHEEL_COUNT];
    float dphi_bc_rad_s[WL_WHEEL_COUNT];
    uint8_t wheel_velocity_valid[WL_WHEEL_COUNT];
    uint8_t body_attitude_valid;
    uint8_t phi_bc_valid[WL_WHEEL_COUNT];
} WheelLegEstimatorInput;

/*
 * Explicit physical parameters for one estimator update.  These values are
 * intentionally not defaulted in firmware because they require calibration.
 */
typedef struct
{
    float wheel_radius_m;
    float dt_s;
} WheelLegEstimatorParams;

#endif
