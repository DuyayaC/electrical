#include <assert.h>
#include <math.h>
#include <string.h>

#include "five_bar.h"
#include "wheel_leg_lqr.h"
#include "wheel_leg_state_estimator.h"
#include "wheel_leg_vmc.h"

static FiveBarGeometry test_geometry(void)
{
    FiveBarGeometry geometry;

    memset(&geometry, 0, sizeof(geometry));
    geometry.l1_m = 0.140f;
    geometry.l2_m = 0.200f;
    geometry.l3_m = 0.200f;
    geometry.l4_m = 0.140f;
    geometry.base_width_m = 0.100f;
    geometry.branch_sign = -1.0f;
    geometry.finite_difference_step_rad = 0.0001f;
    geometry.triangle_tolerance_m = 0.000001f;
    geometry.min_length_m = 0.05f;
    geometry.max_length_m = 0.40f;
    geometry.min_abs_detJ = 0.00001f;
    return geometry;
}

static void test_fivebar_and_estimator(void)
{
    FiveBarGeometry geometry = test_geometry();
    LegKinematicState legs[WL_WHEEL_COUNT];
    WheelLegEstimatorInput input;
    WheelLegEstimatorParams params;
    WheelLegStateEstimator estimator;
    WheelLegEstimate estimate;
    uint8_t i;

    assert(FiveBar_Solve(&geometry, 1.0471976f, 2.0943951f,
                         0.0f, 0.0f, &legs[WL_WHEEL_LEFT]) == 1u);
    assert(FiveBar_Solve(&geometry, 1.0471976f, 2.0943951f,
                         0.0f, 0.0f, &legs[WL_WHEEL_RIGHT]) == 1u);
    assert(fabsf(legs[0].leg_axis_body_rad) < 0.001f);
    assert(fabsf(legs[0].detJ) > geometry.min_abs_detJ);
    assert(fabsf(legs[0].dL0_m_s) < 0.001f);

    memset(&input, 0, sizeof(input));
    input.body_attitude_valid = 1u;
    params.wheel_radius_m = 0.075f;
    params.dt_s = 0.001f;
    for (i = 0u; i < WL_WHEEL_COUNT; i++)
    {
        input.wheel_velocity_valid[i] = 1u;
        input.phi_bc_valid[i] = 1u;
    }

    WheelLeg_StateEstimator_Reset(&estimator);
    assert(WheelLeg_StateEstimator_Update(&estimator, &input, &params,
                                          legs, &estimate) == 1u);
    assert(estimate.valid == 1u);
    assert(fabsf(estimate.theta_rad) < 0.001f);
    assert(fabsf(estimate.dxb_m_s) < 0.001f);

    input.phi_bc_valid[WL_WHEEL_LEFT] = 0u;
    assert(WheelLeg_StateEstimator_Update(&estimator, &input, &params,
                                          legs, &estimate) == 0u);
    assert(estimate.valid == 0u);
    input.phi_bc_valid[WL_WHEEL_LEFT] = 1u;
    input.body_pitch_rad = 0.1f;
    input.body_pitch_rate_rad_s = 0.2f;
    assert(WheelLeg_StateEstimator_Update(&estimator, &input, &params,
                                          legs, &estimate) == 1u);
    assert(fabsf(estimate.theta_rad - 0.1f) < 0.001f);
    assert(fabsf(estimate.dtheta_rad_s - 0.2f) < 0.001f);
}

static void test_lqr_and_vmc(void)
{
    const float gain[WHEEL_LEG_LQR_OUTPUT_COUNT]
                    [WHEEL_LEG_LQR_STATE_COUNT] =
    {
        {-44.3788f, -6.8496f, -22.2828f, -21.5569f, 28.7706f, 4.3751f},
        { 11.2006f,  0.7339f,   3.7300f,   3.2058f,151.7300f, 4.6387f}
    };
    const float x[WHEEL_LEG_LQR_STATE_COUNT] =
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    const float x_ref[WHEEL_LEG_LQR_STATE_COUNT] =
        {0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.2f};
    LegKinematicState legs[WL_WHEEL_COUNT];
    float force_N[WL_WHEEL_COUNT] = {5.0f, 5.0f};
    float pitch_torque_Nm[WL_WHEEL_COUNT] = {6.0f, 6.0f};
    float lqr_output[WHEEL_LEG_LQR_OUTPUT_COUNT];
    float joint_torque_Nm[WL_JOINT_COUNT];
    uint8_t i;

    memset(legs, 0, sizeof(legs));
    for (i = 0u; i < WL_WHEEL_COUNT; i++)
    {
        legs[i].valid = 1u;
        legs[i].jacobian_valid = 1u;
        legs[i].detJ = 1.0f;
        legs[i].J[0][0] = 1.0f;
        legs[i].J[0][1] = 2.0f;
        legs[i].J[1][0] = 3.0f;
        legs[i].J[1][1] = 4.0f;
    }

    assert(WheelLeg_LQR_Calculate(gain, x, x_ref, lqr_output) == 1u);
    assert(fabsf(lqr_output[0] + 1.35355f) < 0.01f);
    assert(fabsf(lqr_output[1] - 1.30074f) < 0.01f);

    assert(WheelLeg_VMC_Calculate(legs, force_N, pitch_torque_Nm,
                                  joint_torque_Nm) == 1u);
    assert(fabsf(joint_torque_Nm[WL_JOINT_LEFT_Q1] - 23.0f) < 0.001f);
    assert(fabsf(joint_torque_Nm[WL_JOINT_LEFT_Q4] - 34.0f) < 0.001f);
    assert(fabsf(joint_torque_Nm[WL_JOINT_RIGHT_Q1] - 23.0f) < 0.001f);
    assert(fabsf(joint_torque_Nm[WL_JOINT_RIGHT_Q4] - 34.0f) < 0.001f);
}

int main(void)
{
    test_fivebar_and_estimator();
    test_lqr_and_vmc();
    return 0;
}
