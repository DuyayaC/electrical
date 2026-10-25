/*
 * selftest.c -- equivalence / determinism test driver.
 *
 * The SAME source file is compiled twice:
 *
 *   ORIGINAL : -DINFANTRY_CHASSIS_BUILD_ORIGINAL
 *              linking FiveBar/five_bar.c, VMC/vmc.c and the three
 *              WheelLegLQR/wheel_leg_lqr*.c files (kept for reference).
 *   MERGED   : linking the integrated library in Chassis/:
 *              Chassis/five_bar_merged.c, Chassis/vmc_merged.c and
 *              Chassis/wheel_leg_lqr_merged.c.
 *
 * Both builds call every public function over a deterministic input set
 * (valid cases, null pointers, NaN, +Inf, degenerate geometry, decreasing and
 * constant gain tables) and fold every status byte and every output byte into
 * one FNV-1a fingerprint.  Identical fingerprints prove that merging the three
 * folders into one .h/.c pair each preserved behaviour bit for bit.
 *
 * Build:
 *   merged  : -IChassis                                   (headers are siblings here)
 *   original: -IChassis -IFiveBar -IVMC -IWheelLegLQR     (original headers)
 *   gcc -std=c99 -O0 -ffp-contract=off <includes> selftest.c <sources> -lm
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef INFANTRY_CHASSIS_BUILD_ORIGINAL
#include "five_bar.h"
#include "vmc.h"
#include "wheel_leg_lqr.h"
#include "wheel_leg_lqr_schedule.h"
#include "wheel_leg_lqr_default.h"

/*
 * Compatibility bridge for the ORIGINAL baseline build only.
 *
 * The original headers still declare the pre-rename API, while this test file
 * uses the final names.  These aliases map old -> new so one test source can
 * drive both builds; nothing in the delivered library depends on them.
 */
#define FiveBar_Get2026DefaultGeometry    FiveBar_GetDefault2026Geometry
#define FiveBar_GetLengthAndAxis          FiveBar_Position
#define FiveBar_SolveLegKinematics        FiveBar_Solve
#define VMC_ConvertWrenchToJointTorque    VMC_Calculate2D
#define WheelLegLQR_CalculateFeedbackOutput WheelLegLQR_Calculate
#define WheelLegLQR_GetGainsForLegLength  WheelLegLQR_ScheduleInterpolate
#define WheelLegLQR_Get2026DefaultSchedule WheelLegLQR_GetDefault2026Schedule
#else
#include "five_bar_merged.h"
#include "vmc_merged.h"
#include "wheel_leg_lqr_merged.h"
#endif

#define HASH_OFFSET 2166136261u
#define HASH_PRIME 16777619u

static uint32_t g_hash = HASH_OFFSET;
static int g_failures = 0;

#ifdef INFANTRY_CHASSIS_DIAG
#define DIAG_MARK(tag) \
    printf("  mark %-22s = 0x%08X\n", tag, (unsigned)g_hash)
#else
#define DIAG_MARK(tag) ((void)0)
#endif

static void hash_bytes(const void *data, size_t length)
{
    const unsigned char *bytes = (const unsigned char *)data;
    size_t index;

    for (index = 0u; index < length; ++index)
    {
        g_hash ^= (uint32_t)bytes[index];
        g_hash *= HASH_PRIME;
    }
}

static void hash_u8(uint8_t value)
{
    hash_bytes(&value, sizeof(value));
}

static void hash_f32(float value)
{
    hash_bytes(&value, sizeof(value));
}

/* Hash a raw float array bit for bit, with no value filtering on purpose. */
static void hash_f32_array(const float *values, size_t count)
{
    hash_bytes(values, count * sizeof(float));
}

/* Bit-pattern finiteness check, independent of libm and of rounding. */
static int is_finite_bits(float value)
{
    uint32_t bits;
    uint32_t exponent;

    memcpy(&bits, &value, sizeof(bits));
    exponent = bits & 0x7F800000u;
    return (exponent != 0x7F800000u) ? 1 : 0;
}

static float nan_value(void)
{
    uint32_t bits = 0x7FC00000u;
    float value;

    memcpy(&value, &bits, sizeof(value));
    return value;
}

static float inf_value(void)
{
    uint32_t bits = 0x7F800000u;
    float value;

    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void check(int condition, const char *message)
{
    if (!condition)
    {
        printf("FAIL: %s\n", message);
        g_failures++;
    }
}

/* ---------------------------------------------------------------- */
/* FiveBar                                                          */
/* ---------------------------------------------------------------- */

static FiveBarGeometry g_geometry;

static const float g_q_grid[] =
{
    -3.0f, -2.4f, -1.8f, -1.2f, -0.6f, -0.15f, 0.0f,
     0.15f,  0.6f,  1.2f,  1.8f,  2.4f,  3.0f
};

static const float g_dq_grid[] =
{
    -20.0f, -7.5f, -1.0f, -0.001f, 0.0f, 0.001f, 1.0f, 7.5f, 20.0f
};

static FiveBarState g_state;

static void five_bar_case(const FiveBarGeometry *geometry,
                          float q1,
                          float q4,
                          float dq1,
                          float dq4)
{
    float length_m = 0.0f;
    float leg_axis_body_rad = 0.0f;
    float guard_length_m = 123.0f;
    float guard_angle = 456.0f;
    uint8_t position_ok;
    uint8_t solve_ok;

    position_ok = FiveBar_GetLengthAndAxis(geometry, q1, q4,
                                   &length_m, &leg_axis_body_rad);
    hash_u8(position_ok);
    hash_f32(length_m);
    hash_f32(leg_axis_body_rad);

    solve_ok = FiveBar_SolveLegKinematics(geometry, q1, q4, dq1, dq4, &g_state);
    hash_u8(solve_ok);
    hash_f32_array(&g_state.q1_rad, 8u);        /* q1..dleg_axis_body_rad */
    hash_f32_array(&g_state.J[0][0], 4u);
    hash_f32(g_state.detJ);
    hash_u8(g_state.valid);
    hash_u8(g_state.jacobian_valid);

    /* Independently recheck the documented invariants. */
    check(solve_ok <= 1u, "FiveBar_SolveLegKinematics status must be 0 or 1");
    check(g_state.valid <= 1u, "FiveBarState.valid must be 0 or 1");
    check(g_state.jacobian_valid <= 1u,
          "FiveBarState.jacobian_valid must be 0 or 1");
    if (position_ok != 0u)
    {
        check(is_finite_bits(length_m), "valid length must be finite");
        check(is_finite_bits(leg_axis_body_rad),
              "valid leg axis must be finite");
        check(length_m >= geometry->min_length_m &&
              length_m <= geometry->max_length_m,
              "reported length must respect min/max limits");
    }
    if (solve_ok != 0u)
    {
        check(g_state.valid == 1u, "solve==1 requires valid==1");
        check(g_state.jacobian_valid == 1u,
              "solve==1 requires jacobian_valid==1");
    }

    /* Null position outputs must be rejected without writing memory. */
    check(FiveBar_GetLengthAndAxis(geometry, q1, q4, 0, &guard_angle) == 0u,
          "FiveBar_GetLengthAndAxis must reject a null length pointer");
    check(guard_angle == 456.0f,
          "FiveBar_GetLengthAndAxis must not write a null-length companion");
    check(FiveBar_GetLengthAndAxis(geometry, q1, q4, &guard_length_m, 0) == 0u,
          "FiveBar_GetLengthAndAxis must reject a null angle pointer");
    check(guard_length_m == 123.0f,
          "FiveBar_GetLengthAndAxis must not write a null-angle companion");
    hash_f32(guard_length_m);
    hash_f32(guard_angle);
}

static void test_five_bar(void)
{
    size_t i;
    size_t j;
    size_t k;
    size_t m;
    uint8_t ok;

    memset(&g_geometry, 0, sizeof(g_geometry));
    check(FiveBar_Get2026DefaultGeometry(0) == 0u,
          "FiveBar_Get2026DefaultGeometry must reject a null pointer");
    ok = FiveBar_Get2026DefaultGeometry(&g_geometry);
    hash_u8(ok);
    check(ok == 1u, "default geometry must be produced");
    hash_bytes(&g_geometry, sizeof(g_geometry));

    for (i = 0u; i < sizeof(g_q_grid) / sizeof(g_q_grid[0]); ++i)
    {
        for (j = 0u; j < sizeof(g_q_grid) / sizeof(g_q_grid[0]); ++j)
        {
            for (k = 0u; k < sizeof(g_dq_grid) / sizeof(g_dq_grid[0]); ++k)
            {
                for (m = 0u; m < sizeof(g_dq_grid) / sizeof(g_dq_grid[0]); ++m)
                {
                    five_bar_case(&g_geometry,
                                  g_q_grid[i], g_q_grid[j],
                                  g_dq_grid[k], g_dq_grid[m]);
                }
            }
        }
    }

    /* Non-finite joint inputs and null pointers. */
    five_bar_case(&g_geometry, nan_value(), 0.3f, 1.0f, -1.0f);
    five_bar_case(&g_geometry, 0.3f, inf_value(), 1.0f, -1.0f);
    five_bar_case(&g_geometry, 0.3f, 0.3f, nan_value(), -1.0f);
    five_bar_case(&g_geometry, 0.3f, 0.3f, 1.0f, inf_value());
    check(FiveBar_SolveLegKinematics(&g_geometry, 0.3f, 0.3f, 1.0f, 1.0f, 0) == 0u,
          "FiveBar_SolveLegKinematics must reject a null state pointer");
    check(FiveBar_SolveLegKinematics(0, 0.3f, 0.3f, 1.0f, 1.0f, &g_state) == 0u,
          "FiveBar_SolveLegKinematics must reject null geometry");
    check(FiveBar_GetLengthAndAxis(0, 0.3f, 0.3f, 0, 0) == 0u,
          "FiveBar_GetLengthAndAxis must reject null geometry");

    /* Every geometry field must be able to invalidate the solve. */
    {
        static const float bad[] = { 0.0f, -0.01f };
        size_t b;
        size_t field;

        for (field = 0u; field < 11u; ++field)
        {
            for (b = 0u; b < sizeof(bad) / sizeof(bad[0]); ++b)
            {
                FiveBarGeometry geometry;
                float *fields[11];
                int expect_valid;
                uint8_t result;

                memcpy(&geometry, &g_geometry, sizeof(geometry));
                fields[0] = &geometry.l1_m;
                fields[1] = &geometry.l2_m;
                fields[2] = &geometry.l3_m;
                fields[3] = &geometry.l4_m;
                fields[4] = &geometry.base_width_m;
                fields[5] = &geometry.branch_sign;
                fields[6] = &geometry.finite_difference_step_rad;
                fields[7] = &geometry.triangle_tolerance_m;
                fields[8] = &geometry.min_length_m;
                fields[9] = &geometry.max_length_m;
                fields[10] = &geometry.min_abs_detJ;
                *fields[field] = bad[b];

                /* Recompute the documented geometry rule independently, so the
                 * assertion states what the module promises rather than what
                 * this particular mutation happens to do. */
                expect_valid = (geometry.l1_m > 0.0f &&
                                geometry.l2_m > 0.0f &&
                                geometry.l3_m > 0.0f &&
                                geometry.l4_m > 0.0f &&
                                geometry.base_width_m > 0.0f &&
                                geometry.branch_sign != 0.0f &&
                                geometry.finite_difference_step_rad > 0.0f &&
                                geometry.triangle_tolerance_m > 0.0f &&
                                geometry.min_length_m >= 0.0f &&
                                geometry.max_length_m >=
                                    geometry.min_length_m &&
                                geometry.min_abs_detJ > 0.0f);

                memset(&g_state, 0xAB, sizeof(g_state));
                result = FiveBar_SolveLegKinematics(&geometry, 0.3f, 0.3f, 1.0f, 1.0f,
                                       &g_state);
                hash_u8(result);
                hash_f32_array(&g_state.q1_rad, 8u);
                hash_f32_array(&g_state.J[0][0], 4u);
                hash_f32(g_state.detJ);
                hash_u8(g_state.valid);
                hash_u8(g_state.jacobian_valid);
                if (expect_valid == 0)
                {
                    check(result == 0u,
                          "invalid geometry must fail the solve");
                    check(g_state.valid == 0u,
                          "failed solve must clear the valid flag");
                }
                else
                {
                    check(g_state.valid == 1u || result == 0u,
                          "a valid geometry must not report valid==0 "
                          "on a successful solve");
                }
            }
        }

        /* Non-finite geometry fields, hashed separately: NaN payloads are
         * implementation sensitive, so only the status is compared here. */
        for (field = 0u; field < 11u; ++field)
        {
            FiveBarGeometry geometry;
            float *fields[11];

            memcpy(&geometry, &g_geometry, sizeof(geometry));
            fields[0] = &geometry.l1_m;
            fields[1] = &geometry.l2_m;
            fields[2] = &geometry.l3_m;
            fields[3] = &geometry.l4_m;
            fields[4] = &geometry.base_width_m;
            fields[5] = &geometry.branch_sign;
            fields[6] = &geometry.finite_difference_step_rad;
            fields[7] = &geometry.triangle_tolerance_m;
            fields[8] = &geometry.min_length_m;
            fields[9] = &geometry.max_length_m;
            fields[10] = &geometry.min_abs_detJ;
            *fields[field] = inf_value();
            hash_u8(FiveBar_SolveLegKinematics(&geometry, 0.3f, 0.3f, 1.0f, 1.0f,
                                  &g_state));
            *fields[field] = nan_value();
            hash_u8(FiveBar_SolveLegKinematics(&geometry, 0.3f, 0.3f, 1.0f, 1.0f,
                                  &g_state));
        }
    }

    /* Branch selection flips with branch_sign. */
    {
        FiveBarGeometry left;
        FiveBarGeometry right;
        float left_length = 0.0f;
        float left_angle = 0.0f;
        float right_length = 0.0f;
        float right_angle = 0.0f;

        memcpy(&left, &g_geometry, sizeof(left));
        memcpy(&right, &g_geometry, sizeof(right));
        left.branch_sign = 1.0f;
        right.branch_sign = -1.0f;
        hash_u8(FiveBar_GetLengthAndAxis(&left, 0.6f, 0.6f,
                                 &left_length, &left_angle));
        hash_u8(FiveBar_GetLengthAndAxis(&right, 0.6f, 0.6f,
                                 &right_length, &right_angle));
        hash_f32(left_length);
        hash_f32(left_angle);
        hash_f32(right_length);
        hash_f32(right_angle);
        check(left_angle != right_angle,
              "the two branch solutions must differ");
    }
}

/* ---------------------------------------------------------------- */
/* VMC                                                              */
/* ---------------------------------------------------------------- */

static void test_vmc(void)
{
    static const float force_grid[] = { -120.0f, -1.5f, 0.0f, 1.5f, 250.0f };
    static const float torque_grid[] = { -9.0f, -0.25f, 0.0f, 0.25f, 9.0f };
    static const float jacobian_grid[] =
    {
        -1.5f, -0.75f, 0.0f, 0.25f, 1.0f
    };
    size_t a;
    size_t b;
    size_t c;
    size_t d;
    size_t e;
    size_t f;
    float jacobian[2][2];
    float torque[2];
    float null_torque[2];
    float zero_jacobian[2][2] = { { 0.0f, 0.0f }, { 0.0f, 0.0f } };

    for (a = 0u; a < sizeof(jacobian_grid) / sizeof(jacobian_grid[0]); ++a)
    {
        for (b = 0u; b < sizeof(jacobian_grid) / sizeof(jacobian_grid[0]); ++b)
        {
            for (c = 0u; c < sizeof(jacobian_grid) / sizeof(jacobian_grid[0]);
                 ++c)
            {
                for (d = 0u;
                     d < sizeof(jacobian_grid) / sizeof(jacobian_grid[0]);
                     ++d)
                {
                    for (e = 0u;
                         e < sizeof(force_grid) / sizeof(force_grid[0]);
                         ++e)
                    {
                        for (f = 0u;
                             f < sizeof(torque_grid) / sizeof(torque_grid[0]);
                             ++f)
                        {
                            float expected0;
                            float expected1;
                            uint8_t ok;

                            jacobian[0][0] = jacobian_grid[a];
                            jacobian[0][1] = jacobian_grid[b];
                            jacobian[1][0] = jacobian_grid[c];
                            jacobian[1][1] = jacobian_grid[d];
                            torque[0] = 7.0f;
                            torque[1] = -7.0f;
                            ok = VMC_ConvertWrenchToJointTorque(jacobian,
                                                 force_grid[e],
                                                 torque_grid[f],
                                                 torque);
                            hash_u8(ok);
                            hash_f32_array(torque, 2u);
                            check(ok == 1u, "finite VMC input must succeed");

                            expected0 = jacobian[0][0] * force_grid[e] +
                                        jacobian[1][0] * torque_grid[f];
                            expected1 = jacobian[0][1] * force_grid[e] +
                                        jacobian[1][1] * torque_grid[f];
                            check(torque[0] == expected0 &&
                                  torque[1] == expected1,
                                  "VMC must compute tau = J^T [F, Tp]");
                        }
                    }
                }
            }
        }
    }

    null_torque[0] = 11.0f;
    null_torque[1] = -11.0f;
    check(VMC_ConvertWrenchToJointTorque(jacobian, 1.0f, 1.0f, 0) == 0u,
          "VMC must reject a null torque pointer");

    null_torque[0] = 11.0f;
    null_torque[1] = -11.0f;
    hash_u8(VMC_ConvertWrenchToJointTorque(0, 1.0f, 1.0f, null_torque));
    check(null_torque[0] == 0.0f && null_torque[1] == 0.0f,
          "VMC must zero outputs for a null Jacobian");
    hash_f32_array(null_torque, 2u);

    null_torque[0] = 11.0f;
    null_torque[1] = -11.0f;
    hash_u8(VMC_ConvertWrenchToJointTorque(jacobian, nan_value(), 1.0f, null_torque));
    check(null_torque[0] == 0.0f && null_torque[1] == 0.0f,
          "VMC must zero outputs for a NaN force");
    hash_f32_array(null_torque, 2u);

    null_torque[0] = 11.0f;
    null_torque[1] = -11.0f;
    hash_u8(VMC_ConvertWrenchToJointTorque(jacobian, 1.0f, inf_value(), null_torque));
    check(null_torque[0] == 0.0f && null_torque[1] == 0.0f,
          "VMC must zero outputs for an infinite torque");
    hash_f32_array(null_torque, 2u);

    {
        float bad_jacobian[2][2] = { { 1.0f, 2.0f }, { 3.0f, nan_value() } };

        null_torque[0] = 11.0f;
        null_torque[1] = -11.0f;
        hash_u8(VMC_ConvertWrenchToJointTorque(bad_jacobian, 1.0f, 1.0f, null_torque));
        check(null_torque[0] == 0.0f && null_torque[1] == 0.0f,
              "VMC must zero outputs for a NaN Jacobian entry");
        hash_f32_array(null_torque, 2u);
    }

    hash_u8(VMC_ConvertWrenchToJointTorque(zero_jacobian, 5.0f, 5.0f, null_torque));
    hash_f32_array(null_torque, 2u);
}

/* ---------------------------------------------------------------- */
/* WheelLegLQR                                                      */
/* ---------------------------------------------------------------- */

#define LQR_CUSTOM_COUNT 3u

static const float g_lqr_custom_l0[LQR_CUSTOM_COUNT] = { 0.10f, 0.25f, 0.40f };

static const float g_lqr_custom_gain[LQR_CUSTOM_COUNT]
                                   [WHEEL_LEG_LQR_OUTPUT_COUNT]
                                   [WHEEL_LEG_LQR_STATE_COUNT] =
{
    {
        { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f },
        { -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f }
    },
    {
        { 2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f },
        { -2.0f, -4.0f, -6.0f, -8.0f, -10.0f, -12.0f }
    },
    {
        { 3.0f, 6.0f, 9.0f, 12.0f, 15.0f, 18.0f },
        { -3.0f, -6.0f, -9.0f, -12.0f, -15.0f, -18.0f }
    }
};

static void lqr_interpolate_case(const WheelLegLQRSchedule *schedule,
                                 float l0_m)
{
    float gain[WHEEL_LEG_LQR_OUTPUT_COUNT][WHEEL_LEG_LQR_STATE_COUNT];
    uint8_t ok;
    uint32_t row;
    uint32_t column;

    memset(gain, 0, sizeof(gain));
    ok = WheelLegLQR_GetGainsForLegLength(schedule, l0_m, gain);
    hash_u8(ok);
    hash_f32_array(&gain[0][0],
                   WHEEL_LEG_LQR_OUTPUT_COUNT * WHEEL_LEG_LQR_STATE_COUNT);
    check(ok == 0u || ok == 1u, "interpolation status must be 0 or 1");
    if (ok == 1u)
    {
        for (row = 0u; row < WHEEL_LEG_LQR_OUTPUT_COUNT; ++row)
        {
            for (column = 0u; column < WHEEL_LEG_LQR_STATE_COUNT; ++column)
            {
                check(is_finite_bits(gain[row][column]),
                      "interpolated gain must be finite");
            }
        }
    }
}

static void test_lqr(void)
{
    const WheelLegLQRSchedule *schedule = WheelLegLQR_Get2026DefaultSchedule();
    float l0;
    float gain[WHEEL_LEG_LQR_OUTPUT_COUNT][WHEEL_LEG_LQR_STATE_COUNT];
    float x[WHEEL_LEG_LQR_STATE_COUNT];
    float x_ref[WHEEL_LEG_LQR_STATE_COUNT];
    float output[WHEEL_LEG_LQR_OUTPUT_COUNT];
    uint32_t index;

    hash_u8(schedule != 0 ? 1u : 0u);
    check(schedule != 0, "the default schedule must not be null");
    /*
     * Hash the schedule's scalar content only.  The struct also holds two
     * pointers, and pointer values differ between two separately linked
     * binaries (layout, ASLR), so hashing sizeof(*schedule) would report a
     * false mismatch even when the data is identical.  The pointed-to tables
     * are hashed right below.
     */
    hash_u8((uint8_t)(schedule->count == 11u ? 1u : 0u));
    hash_f32_array(schedule->l0_m, schedule->count);
    hash_f32_array(&schedule->gain[0][0][0],
                   schedule->count * WHEEL_LEG_LQR_OUTPUT_COUNT *
                   WHEEL_LEG_LQR_STATE_COUNT);
    check(schedule->count == 11u, "the 2026 schedule must hold 11 nodes");
    check(schedule->l0_m[0] == 0.1042f &&
          schedule->l0_m[10] == 0.3672f,
          "the 2026 schedule endpoints must match the documented range");

    /* Fine L0 sweep over the whole table and beyond both ends. */
    for (index = 0u; index <= 600u; ++index)
    {
        l0 = 0.05f + 0.0006f * (float)index;
        lqr_interpolate_case(schedule, l0);
    }
    lqr_interpolate_case(schedule, schedule->l0_m[0]);
    lqr_interpolate_case(schedule, schedule->l0_m[schedule->count - 1u]);
    lqr_interpolate_case(schedule, -1.0f);
    lqr_interpolate_case(schedule, 0.0f);
    lqr_interpolate_case(schedule, nan_value());
    lqr_interpolate_case(schedule, inf_value());
    lqr_interpolate_case(0, 0.2f);

    {
        WheelLegLQRSchedule degenerate;
        const float single_l0 = 0.2f;
        const float single_gain[1][WHEEL_LEG_LQR_OUTPUT_COUNT]
                                [WHEEL_LEG_LQR_STATE_COUNT] =
        {
            {
                { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
                { 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f }
            }
        };

        degenerate.count = 1u;
        degenerate.l0_m = &single_l0;
        degenerate.gain = single_gain;
        lqr_interpolate_case(&degenerate, 0.2f);
        lqr_interpolate_case(&degenerate, -5.0f);

        degenerate.count = 0u;
        lqr_interpolate_case(&degenerate, 0.2f);
        degenerate.count = 1u;
        degenerate.l0_m = 0;
        lqr_interpolate_case(&degenerate, 0.2f);
        degenerate.l0_m = &single_l0;
        degenerate.gain = 0;
        lqr_interpolate_case(&degenerate, 0.2f);

        /* Strictly decreasing L0 table: the non-monotonic guard must fire. */
        {
            static const float decreasing_l0[LQR_CUSTOM_COUNT] =
            {
                0.40f, 0.25f, 0.10f
            };
            WheelLegLQRSchedule decreasing;

            decreasing.count = LQR_CUSTOM_COUNT;
            decreasing.l0_m = decreasing_l0;
            decreasing.gain = g_lqr_custom_gain;
            lqr_interpolate_case(&decreasing, 0.27f);
            lqr_interpolate_case(&decreasing, 0.35f);
        }
    }

    /* Custom table: exactness at nodes and linearity between them. */
    {
        WheelLegLQRSchedule custom;
        uint32_t row;
        uint32_t column;

        custom.count = LQR_CUSTOM_COUNT;
        custom.l0_m = g_lqr_custom_l0;
        custom.gain = g_lqr_custom_gain;

        for (index = 0u; index < LQR_CUSTOM_COUNT; ++index)
        {
            uint8_t ok;

            memset(gain, 0, sizeof(gain));
            ok = WheelLegLQR_GetGainsForLegLength(&custom, custom.l0_m[index],
                                                 gain);
            hash_u8(ok);
            hash_f32_array(&gain[0][0],
                           WHEEL_LEG_LQR_OUTPUT_COUNT *
                           WHEEL_LEG_LQR_STATE_COUNT);
            check(ok == 1u, "node interpolation must succeed");
            for (row = 0u; row < WHEEL_LEG_LQR_OUTPUT_COUNT; ++row)
            {
                for (column = 0u; column < WHEEL_LEG_LQR_STATE_COUNT;
                     ++column)
                {
                    check(gain[row][column] ==
                          custom.gain[index][row][column],
                          "interpolation must be exact at a node");
                }
            }
        }

        {
            float midpoint_gain[WHEEL_LEG_LQR_OUTPUT_COUNT]
                               [WHEEL_LEG_LQR_STATE_COUNT];
            float middle = 0.5f * (custom.l0_m[0] + custom.l0_m[1]);
            uint8_t ok;

            memset(midpoint_gain, 0, sizeof(midpoint_gain));
            ok = WheelLegLQR_GetGainsForLegLength(&custom, middle,
                                                 midpoint_gain);
            hash_u8(ok);
            hash_f32_array(&midpoint_gain[0][0],
                           WHEEL_LEG_LQR_OUTPUT_COUNT *
                           WHEEL_LEG_LQR_STATE_COUNT);
            check(ok == 1u, "midpoint interpolation must succeed");
            for (row = 0u; row < WHEEL_LEG_LQR_OUTPUT_COUNT; ++row)
            {
                for (column = 0u; column < WHEEL_LEG_LQR_STATE_COUNT;
                     ++column)
                {
                    float expected = 0.5f *
                        (custom.gain[0][row][column] +
                         custom.gain[1][row][column]);
                    check(midpoint_gain[row][column] == expected,
                          "interpolation must be linear between nodes");
                }
            }
        }
    }

    /* LQR feedback sweep: u = K (x_ref - x). */
    {
        static const float state_grid[] =
        {
            -2.0f, -0.5f, 0.0f, 0.5f, 2.0f
        };
        size_t i;

        for (i = 0u; i < WHEEL_LEG_LQR_STATE_COUNT; ++i)
        {
            x[i] = state_grid[i % 5u];
            x_ref[i] = state_grid[(i + 2u) % 5u];
        }
        for (index = 0u; index < WHEEL_LEG_LQR_STATE_COUNT; ++index)
        {
            uint8_t ok;
            uint32_t row;
            uint32_t column;
            float expected[WHEEL_LEG_LQR_OUTPUT_COUNT];

            x_ref[index] += 0.317f;
            memset(gain, 0, sizeof(gain));
            check(WheelLegLQR_GetGainsForLegLength(schedule,
                                                  0.2f + 0.01f * (float)index,
                                                  gain) == 1u,
                  "gain schedule lookup must succeed");
            memset(output, 0, sizeof(output));
            ok = WheelLegLQR_CalculateFeedbackOutput(gain, x, x_ref, output);
            hash_u8(ok);
            hash_f32_array(output, WHEEL_LEG_LQR_OUTPUT_COUNT);
            check(ok == 1u, "finite LQR input must succeed");

            for (row = 0u; row < WHEEL_LEG_LQR_OUTPUT_COUNT; ++row)
            {
                expected[row] = 0.0f;
                for (column = 0u; column < WHEEL_LEG_LQR_STATE_COUNT;
                     ++column)
                {
                    expected[row] += gain[row][column] *
                                     (x_ref[column] - x[column]);
                }
                check(output[row] == expected[row],
                      "LQR must compute K (x_ref - x)");
            }
        }
    }

    /* Failure paths: null pointers and non-finite state or gain entries. */
    memset(gain, 0, sizeof(gain));
    check(WheelLegLQR_GetGainsForLegLength(schedule, 0.2f, gain) == 1u,
          "baseline gain lookup must succeed");

    {
        float zero_gain[WHEEL_LEG_LQR_OUTPUT_COUNT]
                       [WHEEL_LEG_LQR_STATE_COUNT];

        memset(zero_gain, 0, sizeof(zero_gain));
        check(WheelLegLQR_CalculateFeedbackOutput(zero_gain, x, x_ref, 0) == 0u,
              "LQR must reject a null output pointer");
        check(WheelLegLQR_CalculateFeedbackOutput(0, x, x_ref, output) == 0u,
              "LQR must reject a null gain matrix");
        check(WheelLegLQR_CalculateFeedbackOutput(zero_gain, 0, x_ref, output) == 0u,
              "LQR must reject a null state vector");
        check(WheelLegLQR_CalculateFeedbackOutput(zero_gain, x, 0, output) == 0u,
              "LQR must reject a null reference vector");

        output[0] = 3.0f;
        output[1] = -3.0f;
        hash_u8(WheelLegLQR_CalculateFeedbackOutput(zero_gain, x, x_ref, output));
        hash_f32_array(output, WHEEL_LEG_LQR_OUTPUT_COUNT);
        check(output[0] == 0.0f && output[1] == 0.0f,
              "LQR must zero the outputs for a null gain matrix");

        x[2] = nan_value();
        output[0] = 3.0f;
        output[1] = -3.0f;
        hash_u8(WheelLegLQR_CalculateFeedbackOutput(zero_gain, x, x_ref, output));
        hash_f32_array(output, WHEEL_LEG_LQR_OUTPUT_COUNT);
        check(output[0] == 0.0f && output[1] == 0.0f,
              "LQR must zero the outputs for a NaN state");
        x[2] = 0.5f;

        x_ref[4] = inf_value();
        output[0] = 3.0f;
        output[1] = -3.0f;
        hash_u8(WheelLegLQR_CalculateFeedbackOutput(zero_gain, x, x_ref, output));
        hash_f32_array(output, WHEEL_LEG_LQR_OUTPUT_COUNT);
        check(output[0] == 0.0f && output[1] == 0.0f,
              "LQR must zero the outputs for an infinite reference");
        x_ref[4] = -0.5f;

        memset(gain, 0, sizeof(gain));
        gain[1][1] = nan_value();
        output[0] = 3.0f;
        output[1] = -3.0f;
        hash_u8(WheelLegLQR_CalculateFeedbackOutput(gain, x, x_ref, output));
        hash_f32_array(output, WHEEL_LEG_LQR_OUTPUT_COUNT);
        check(output[0] == 0.0f && output[1] == 0.0f,
              "LQR must zero the outputs for a NaN gain entry");

        memset(gain, 0, sizeof(gain));
        gain[0][5] = 1.0f;
        output[0] = 3.0f;
        output[1] = -3.0f;
        hash_u8(WheelLegLQR_CalculateFeedbackOutput(gain, x, x_ref, output));
        hash_f32_array(output, WHEEL_LEG_LQR_OUTPUT_COUNT);
    }
}

/* ---------------------------------------------------------------- */
/* Full pipeline: FiveBar -> VMC and FiveBar -> LQR                 */
/* ---------------------------------------------------------------- */

static void test_pipeline(void)
{
    const WheelLegLQRSchedule *schedule = WheelLegLQR_Get2026DefaultSchedule();
    size_t i;
    size_t j;
    size_t k;
    size_t m;

    for (i = 0u; i < sizeof(g_q_grid) / sizeof(g_q_grid[0]); ++i)
    {
        for (j = 0u; j < sizeof(g_q_grid) / sizeof(g_q_grid[0]); ++j)
        {
            for (k = 0u; k < sizeof(g_dq_grid) / sizeof(g_dq_grid[0]); ++k)
            {
                for (m = 0u; m < sizeof(g_dq_grid) / sizeof(g_dq_grid[0]); ++m)
                {
                    float torque[2];
                    float gain[WHEEL_LEG_LQR_OUTPUT_COUNT]
                              [WHEEL_LEG_LQR_STATE_COUNT];
                    float x[WHEEL_LEG_LQR_STATE_COUNT] =
                    {
                        0.02f, -0.3f, 0.01f, 0.4f, 0.05f, -0.2f
                    };
                    float x_ref[WHEEL_LEG_LQR_STATE_COUNT] =
                    {
                        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
                    };
                    float output[WHEEL_LEG_LQR_OUTPUT_COUNT];
                    uint8_t solved;
                    uint8_t vmc_ok;
                    uint8_t schedule_ok;
                    uint8_t lqr_ok;

                    solved = FiveBar_SolveLegKinematics(&g_geometry,
                                           g_q_grid[i], g_q_grid[j],
                                           g_dq_grid[k], g_dq_grid[m],
                                           &g_state);
                    hash_u8(solved);
                    hash_f32_array(&g_state.q1_rad, 8u);
                    hash_f32_array(&g_state.J[0][0], 4u);
                    hash_f32(g_state.detJ);
                    if (solved == 0u)
                    {
                        continue;
                    }

                    /* Feed the leg Jacobian into the torque mapping. */
                    torque[0] = 0.0f;
                    torque[1] = 0.0f;
                    vmc_ok = VMC_ConvertWrenchToJointTorque(g_state.J,
                                             40.0f * g_state.dL0_m_s,
                                             8.0f *
                                             g_state.dleg_axis_body_rad_s,
                                             torque);
                    hash_u8(vmc_ok);
                    hash_f32_array(torque, 2u);

                    /* Feed the leg length into the gain schedule. */
                    memset(gain, 0, sizeof(gain));
                    schedule_ok = WheelLegLQR_GetGainsForLegLength(
                        schedule, g_state.L0_m, gain);
                    hash_u8(schedule_ok);
                    hash_f32_array(&gain[0][0],
                                   WHEEL_LEG_LQR_OUTPUT_COUNT *
                                   WHEEL_LEG_LQR_STATE_COUNT);
                    if (schedule_ok == 0u)
                    {
                        continue;
                    }

                    output[0] = 0.0f;
                    output[1] = 0.0f;
                    lqr_ok = WheelLegLQR_CalculateFeedbackOutput(gain, x, x_ref, output);
                    hash_u8(lqr_ok);
                    hash_f32_array(output, WHEEL_LEG_LQR_OUTPUT_COUNT);

                    check(vmc_ok == 1u && lqr_ok == 1u,
                          "the full pipeline must succeed on a valid state");
                }
            }
        }
    }
}

int main(void)
{
    memset(&g_geometry, 0, sizeof(g_geometry));
    if (FiveBar_Get2026DefaultGeometry(&g_geometry) == 0u)
    {
        printf("cannot build default geometry\n");
        return 2;
    }

#ifdef INFANTRY_CHASSIS_DIAG
    {
        uint32_t before;

        before = g_hash;
        test_five_bar();
        printf("phase five_bar  = 0x%08X\n", (unsigned)(g_hash ^ before));

        before = g_hash;
        test_vmc();
        printf("phase vmc       = 0x%08X\n", (unsigned)(g_hash ^ before));

        before = g_hash;
        test_lqr();
        printf("phase lqr       = 0x%08X\n", (unsigned)(g_hash ^ before));

        before = g_hash;
        test_pipeline();
        printf("phase pipeline  = 0x%08X\n", (unsigned)(g_hash ^ before));
    }
#else
    test_five_bar();
    test_vmc();
    test_lqr();
    test_pipeline();
#endif

    printf("build=%s\n",
#ifdef INFANTRY_CHASSIS_BUILD_ORIGINAL
           "original");
#else
           "merged");
#endif
    printf("failures=%d\n", g_failures);
    printf("fingerprint=0x%08X\n", (unsigned)g_hash);
    return (g_failures == 0) ? 0 : 1;
}
