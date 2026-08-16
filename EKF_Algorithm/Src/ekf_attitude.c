#include "ekf_attitude.h"

#include <math.h>
#include <string.h>

#ifndef EKF_ATTITUDE_EPS
#define EKF_ATTITUDE_EPS 1.0e-9f
#endif

#define EKF_N EKF_ATTITUDE_STATE_DIM
#define EKF_M EKF_ATTITUDE_MEAS_DIM

static float32_t clampf(float32_t value, float32_t low, float32_t high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static void zero(float32_t *data, uint32_t count)
{
    memset(data, 0, sizeof(float32_t) * count);
}

static void set_identity(float32_t *data, uint32_t n)
{
    zero(data, n * n);
    for (uint32_t i = 0U; i < n; ++i) {
        data[i * n + i] = 1.0f;
    }
}

static void normalize_quaternion(float32_t q[4])
{
    const float32_t norm2 = q[0] * q[0] + q[1] * q[1]
                          + q[2] * q[2] + q[3] * q[3];
    if (norm2 <= EKF_ATTITUDE_EPS) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    const float32_t inv_norm = 1.0f / sqrtf(norm2);
    q[0] *= inv_norm;
    q[1] *= inv_norm;
    q[2] *= inv_norm;
    q[3] *= inv_norm;
}

static void symmetrize_covariance(float32_t P[36])
{
    for (uint32_t r = 0U; r < EKF_N; ++r) {
        for (uint32_t c = r + 1U; c < EKF_N; ++c) {
            const float32_t v = 0.5f * (P[r * EKF_N + c] + P[c * EKF_N + r]);
            P[r * EKF_N + c] = v;
            P[c * EKF_N + r] = v;
        }
    }

    for (uint32_t i = 0U; i < EKF_N; ++i) {
        if (P[i * EKF_N + i] < EKF_ATTITUDE_EPS) {
            P[i * EKF_N + i] = EKF_ATTITUDE_EPS;
        }
    }
}

static void predict_quaternion(float32_t q[4],
                               const float32_t gyro_rad_s[3],
                               float32_t bias_x,
                               float32_t bias_y,
                               float32_t dt_s)
{
    const float32_t wx = gyro_rad_s[0] - bias_x;
    const float32_t wy = gyro_rad_s[1] - bias_y;
    const float32_t wz = gyro_rad_s[2];
    const float32_t half_dt = 0.5f * dt_s;
    const float32_t q0 = q[0];
    const float32_t q1 = q[1];
    const float32_t q2 = q[2];
    const float32_t q3 = q[3];

    q[0] = q0 + half_dt * (-wx * q1 - wy * q2 - wz * q3);
    q[1] = q1 + half_dt * ( wx * q0 + wz * q2 - wy * q3);
    q[2] = q2 + half_dt * ( wy * q0 - wz * q1 + wx * q3);
    q[3] = q3 + half_dt * ( wz * q0 + wy * q1 - wx * q2);
    normalize_quaternion(q);
}

static void build_F(ekf_attitude_t *ekf,
                    const float32_t gyro_rad_s[3],
                    float32_t dt_s)
{
    const float32_t *x = ekf->x;
    const float32_t q0 = x[0];
    const float32_t q1 = x[1];
    const float32_t q2 = x[2];
    const float32_t q3 = x[3];
    const float32_t wx = gyro_rad_s[0] - x[4];
    const float32_t wy = gyro_rad_s[1] - x[5];
    const float32_t wz = gyro_rad_s[2];
    const float32_t h = 0.5f * dt_s;
    float32_t *F = ekf->F;

    set_identity(F, EKF_N);

    F[0 * EKF_N + 1] = -wx * h;
    F[0 * EKF_N + 2] = -wy * h;
    F[0 * EKF_N + 3] = -wz * h;

    F[1 * EKF_N + 0] =  wx * h;
    F[1 * EKF_N + 2] =  wz * h;
    F[1 * EKF_N + 3] = -wy * h;

    F[2 * EKF_N + 0] =  wy * h;
    F[2 * EKF_N + 1] = -wz * h;
    F[2 * EKF_N + 3] =  wx * h;

    F[3 * EKF_N + 0] =  wz * h;
    F[3 * EKF_N + 1] =  wy * h;
    F[3 * EKF_N + 2] = -wx * h;

    F[0 * EKF_N + 4] =  q1 * h;
    F[1 * EKF_N + 4] = -q0 * h;
    F[2 * EKF_N + 4] = -q3 * h;
    F[3 * EKF_N + 4] =  q2 * h;

    F[0 * EKF_N + 5] =  q2 * h;
    F[1 * EKF_N + 5] =  q3 * h;
    F[2 * EKF_N + 5] = -q0 * h;
    F[3 * EKF_N + 5] = -q1 * h;
}

static void build_H(const float32_t q[4], float32_t H[18])
{
    zero(H, EKF_M * EKF_N);

    H[0 * EKF_N + 0] = -2.0f * q[2];
    H[0 * EKF_N + 1] =  2.0f * q[3];
    H[0 * EKF_N + 2] = -2.0f * q[0];
    H[0 * EKF_N + 3] =  2.0f * q[1];

    H[1 * EKF_N + 0] =  2.0f * q[1];
    H[1 * EKF_N + 1] =  2.0f * q[0];
    H[1 * EKF_N + 2] =  2.0f * q[3];
    H[1 * EKF_N + 3] =  2.0f * q[2];

    H[2 * EKF_N + 0] =  2.0f * q[0];
    H[2 * EKF_N + 1] = -2.0f * q[1];
    H[2 * EKF_N + 2] = -2.0f * q[2];
    H[2 * EKF_N + 3] =  2.0f * q[3];
}

static void measurement_model(const float32_t q[4], float32_t h[3])
{
    h[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    h[1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
    h[2] = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
}

static arm_status propagate_covariance(ekf_attitude_t *ekf,
                                       const float32_t gyro_rad_s[3],
                                       float32_t dt_s)
{
    arm_matrix_instance_f32 F;
    arm_matrix_instance_f32 Ft;
    arm_matrix_instance_f32 P;
    arm_matrix_instance_f32 tmp_a;
    arm_matrix_instance_f32 tmp_b;

    build_F(ekf, gyro_rad_s, dt_s);

    arm_mat_init_f32(&F, EKF_N, EKF_N, ekf->F);
    arm_mat_init_f32(&Ft, EKF_N, EKF_N, ekf->Ft);
    arm_mat_init_f32(&P, EKF_N, EKF_N, ekf->P);
    arm_mat_init_f32(&tmp_a, EKF_N, EKF_N, ekf->tmp66_a);
    arm_mat_init_f32(&tmp_b, EKF_N, EKF_N, ekf->tmp66_b);

    arm_status st = arm_mat_trans_f32(&F, &Ft);
    if (st != ARM_MATH_SUCCESS) {
        return st;
    }

    memcpy(ekf->tmp66_b, ekf->P, sizeof(ekf->P));

    if ((ekf->cfg.bias_fading_lambda > 0.0f)
        && (ekf->cfg.bias_fading_lambda < 1.0f)) {
        const float32_t s = 1.0f / ekf->cfg.bias_fading_lambda;
        for (uint32_t i = 0U; i < EKF_N; ++i) {
            ekf->tmp66_b[4U * EKF_N + i] *= s;
            ekf->tmp66_b[5U * EKF_N + i] *= s;
            ekf->tmp66_b[i * EKF_N + 4U] *= s;
            ekf->tmp66_b[i * EKF_N + 5U] *= s;
        }
    }

    arm_matrix_instance_f32 faded_P;
    arm_mat_init_f32(&faded_P, EKF_N, EKF_N, ekf->tmp66_b);

    st = arm_mat_mult_f32(&F, &faded_P, &tmp_a);
    if (st != ARM_MATH_SUCCESS) {
        return st;
    }
    st = arm_mat_mult_f32(&tmp_a, &Ft, &tmp_b);
    if (st != ARM_MATH_SUCCESS) {
        return st;
    }
    for (uint32_t i = 0U; i < EKF_N * EKF_N; ++i) {
        ekf->P[i] = ekf->tmp66_b[i] + ekf->Q[i];
    }

    symmetrize_covariance(ekf->P);
    return ARM_MATH_SUCCESS;
}

static void mat_vec_6x3(const float32_t A[18],
                        const float32_t v[3],
                        float32_t out[6])
{
    for (uint32_t r = 0U; r < EKF_N; ++r) {
        out[r] = A[r * EKF_M + 0U] * v[0]
               + A[r * EKF_M + 1U] * v[1]
               + A[r * EKF_M + 2U] * v[2];
    }
}

static void mat_vec_3x3(const float32_t A[9],
                        const float32_t v[3],
                        float32_t out[3])
{
    for (uint32_t r = 0U; r < EKF_M; ++r) {
        out[r] = A[r * EKF_M + 0U] * v[0]
               + A[r * EKF_M + 1U] * v[1]
               + A[r * EKF_M + 2U] * v[2];
    }
}

static arm_status correct_with_accel(ekf_attitude_t *ekf,
                                     const float32_t z[3],
                                     const float32_t gyro_rad_s[3])
{
    float32_t h[3];
    float32_t e[3];
    float32_t tmp_v3[3];
    float32_t dx[6];
    arm_matrix_instance_f32 P;
    arm_matrix_instance_f32 H;
    arm_matrix_instance_f32 Ht;
    arm_matrix_instance_f32 HP;
    arm_matrix_instance_f32 PHt;
    arm_matrix_instance_f32 S;
    arm_matrix_instance_f32 S_inv;
    arm_matrix_instance_f32 K;
    arm_matrix_instance_f32 KH;
    arm_matrix_instance_f32 I_KH;
    arm_matrix_instance_f32 tmp66;

    measurement_model(ekf->x, h);
    e[0] = z[0] - h[0];
    e[1] = z[1] - h[1];
    e[2] = z[2] - h[2];
    build_H(ekf->x, ekf->H);

    arm_mat_init_f32(&P, EKF_N, EKF_N, ekf->P);
    arm_mat_init_f32(&H, EKF_M, EKF_N, ekf->H);
    arm_mat_init_f32(&Ht, EKF_N, EKF_M, ekf->Ht);
    arm_mat_init_f32(&HP, EKF_M, EKF_N, ekf->HP);
    arm_mat_init_f32(&PHt, EKF_N, EKF_M, ekf->PHt);
    arm_mat_init_f32(&S, EKF_M, EKF_M, ekf->S);
    arm_mat_init_f32(&S_inv, EKF_M, EKF_M, ekf->S_inv);
    arm_mat_init_f32(&K, EKF_N, EKF_M, ekf->K);
    arm_mat_init_f32(&KH, EKF_N, EKF_N, ekf->KH);
    arm_mat_init_f32(&I_KH, EKF_N, EKF_N, ekf->I_KH);
    arm_mat_init_f32(&tmp66, EKF_N, EKF_N, ekf->tmp66_a);

    arm_status st = arm_mat_trans_f32(&H, &Ht);
    if (st != ARM_MATH_SUCCESS) {
        return st;
    }
    st = arm_mat_mult_f32(&H, &P, &HP);
    if (st != ARM_MATH_SUCCESS) {
        return st;
    }
    st = arm_mat_mult_f32(&HP, &Ht, &S);
    if (st != ARM_MATH_SUCCESS) {
        return st;
    }
    for (uint32_t i = 0U; i < EKF_M * EKF_M; ++i) {
        ekf->S[i] += ekf->R[i];
    }
    st = arm_mat_inverse_f32(&S, &S_inv);
    if (st != ARM_MATH_SUCCESS) {
        return st;
    }

    mat_vec_3x3(ekf->S_inv, e, tmp_v3);
    ekf->last_chi_square = e[0] * tmp_v3[0] + e[1] * tmp_v3[1] + e[2] * tmp_v3[2];

    const float32_t gyro_norm = sqrtf(gyro_rad_s[0] * gyro_rad_s[0]
                                   + gyro_rad_s[1] * gyro_rad_s[1]
                                   + gyro_rad_s[2] * gyro_rad_s[2]);
    const uint8_t force_update =
        (ekf->cfg.force_update_after_rejects > 0U)
        && (ekf->reject_count >= ekf->cfg.force_update_after_rejects)
        && (gyro_norm < ekf->cfg.gyro_static_limit);

    if ((ekf->cfg.chi_square_gate > 0.0f)
        && (ekf->last_chi_square > ekf->cfg.chi_square_gate)
        && (force_update == 0U)) {
        ekf->reject_count++;
        ekf->last_gain_scale = 0.0f;
        return ARM_MATH_ARGUMENT_ERROR;
    }

    ekf->reject_count = 0U;
    ekf->last_gain_scale = 1.0f;
    if ((ekf->cfg.adaptive_gate > 0.0f)
        && (ekf->last_chi_square > ekf->cfg.adaptive_gate)) {
        ekf->last_gain_scale = ekf->cfg.adaptive_gate / ekf->last_chi_square;
        ekf->last_gain_scale = clampf(ekf->last_gain_scale,
                                      ekf->cfg.adaptive_gain_min,
                                      1.0f);
    }

    st = arm_mat_mult_f32(&P, &Ht, &PHt);
    if (st != ARM_MATH_SUCCESS) {
        return st;
    }
    st = arm_mat_mult_f32(&PHt, &S_inv, &K);
    if (st != ARM_MATH_SUCCESS) {
        return st;
    }

    if (ekf->last_gain_scale < 1.0f) {
        for (uint32_t i = 0U; i < EKF_N * EKF_M; ++i) {
            ekf->K[i] *= ekf->last_gain_scale;
        }
    }

    mat_vec_6x3(ekf->K, e, dx);

    dx[3] = 0.0f;
    dx[4] = clampf(dx[4],
                   -ekf->cfg.max_bias_correction,
                    ekf->cfg.max_bias_correction);
    dx[5] = clampf(dx[5],
                   -ekf->cfg.max_bias_correction,
                    ekf->cfg.max_bias_correction);

    for (uint32_t i = 0U; i < EKF_N; ++i) {
        ekf->x[i] += dx[i];
    }
    normalize_quaternion(ekf->x);

    st = arm_mat_mult_f32(&K, &H, &KH);
    if (st != ARM_MATH_SUCCESS) {
        return st;
    }
    set_identity(ekf->I_KH, EKF_N);
    for (uint32_t i = 0U; i < EKF_N * EKF_N; ++i) {
        ekf->I_KH[i] -= ekf->KH[i];
    }
    st = arm_mat_mult_f32(&I_KH, &P, &tmp66);
    if (st != ARM_MATH_SUCCESS) {
        return st;
    }

    memcpy(ekf->P, ekf->tmp66_a, sizeof(ekf->P));
    symmetrize_covariance(ekf->P);
    return ARM_MATH_SUCCESS;
}

void ekf_attitude_default_config(ekf_attitude_config_t *cfg)
{
    cfg->q_gyro = 1.0e-6f;
    cfg->q_bias = 5.0e-6f;
    cfg->r_accel = 1.0e-2f;
    cfg->initial_attitude_variance = 1.0e-2f;
    cfg->initial_bias_variance = 1.0e-3f;
    cfg->bias_fading_lambda = 0.9995f;
    cfg->chi_square_gate = 50.0f;
    cfg->adaptive_gate = 30.0f;
    cfg->adaptive_gain_min = 0.2f;
    cfg->max_bias_correction = 2.0e-4f;
    cfg->accel_norm_min = 1.0e-3f;
    cfg->accel_norm_max = 1.0e9f;
    cfg->gyro_static_limit = 0.08f;
    cfg->force_update_after_rejects = 50U;
}

void ekf_attitude_init(ekf_attitude_t *ekf,
                       const ekf_attitude_config_t *cfg)
{
    ekf_attitude_config_t local_cfg;
    if (cfg == NULL) {
        ekf_attitude_default_config(&local_cfg);
        cfg = &local_cfg;
    }

    memset(ekf, 0, sizeof(*ekf));
    ekf->cfg = *cfg;
    ekf->x[0] = 1.0f;

    for (uint32_t i = 0U; i < EKF_N; ++i) {
        ekf->P[i * EKF_N + i] =
            (i < 4U) ? cfg->initial_attitude_variance
                     : cfg->initial_bias_variance;
        ekf->Q[i * EKF_N + i] = (i < 4U) ? cfg->q_gyro : cfg->q_bias;
    }
    for (uint32_t i = 0U; i < EKF_M; ++i) {
        ekf->R[i * EKF_M + i] = cfg->r_accel;
    }
    ekf->last_gain_scale = 1.0f;
}

void ekf_attitude_set_quaternion(ekf_attitude_t *ekf,
                                 float32_t q0,
                                 float32_t q1,
                                 float32_t q2,
                                 float32_t q3)
{
    ekf->x[0] = q0;
    ekf->x[1] = q1;
    ekf->x[2] = q2;
    ekf->x[3] = q3;
    normalize_quaternion(ekf->x);
}

void ekf_attitude_set_gyro_bias(ekf_attitude_t *ekf,
                                float32_t bias_x,
                                float32_t bias_y)
{
    ekf->x[4] = bias_x;
    ekf->x[5] = bias_y;
}

ekf_attitude_status_t ekf_attitude_step(ekf_attitude_t *ekf,
                                        const float32_t gyro_rad_s[3],
                                        const float32_t accel[3],
                                        float32_t dt_s)
{
    if ((ekf == NULL) || (gyro_rad_s == NULL) || (accel == NULL)
        || (dt_s <= 0.0f)) {
        return EKF_ATTITUDE_NUMERIC_ERROR;
    }

    if (propagate_covariance(ekf, gyro_rad_s, dt_s) != ARM_MATH_SUCCESS) {
        return EKF_ATTITUDE_NUMERIC_ERROR;
    }
    predict_quaternion(ekf->x, gyro_rad_s, ekf->x[4], ekf->x[5], dt_s);

    const float32_t accel_norm = sqrtf(accel[0] * accel[0]
                                     + accel[1] * accel[1]
                                     + accel[2] * accel[2]);
    ekf->last_accel_norm = accel_norm;
    if (accel_norm <= EKF_ATTITUDE_EPS) {
        return EKF_ATTITUDE_PREDICT_ONLY;
    }

    if ((accel_norm < ekf->cfg.accel_norm_min)
        || (accel_norm > ekf->cfg.accel_norm_max)) {
        ekf->reject_count++;
        return EKF_ATTITUDE_REJECTED;
    }

    const float32_t inv_accel_norm = 1.0f / accel_norm;
    const float32_t z[3] = {
        accel[0] * inv_accel_norm,
        accel[1] * inv_accel_norm,
        accel[2] * inv_accel_norm
    };

    const arm_status st = correct_with_accel(ekf, z, gyro_rad_s);
    if (st == ARM_MATH_ARGUMENT_ERROR) {
        return EKF_ATTITUDE_REJECTED;
    }
    if (st != ARM_MATH_SUCCESS) {
        return EKF_ATTITUDE_NUMERIC_ERROR;
    }
    return EKF_ATTITUDE_OK;
}

void ekf_attitude_get_quaternion(const ekf_attitude_t *ekf,
                                 float32_t q[4])
{
    memcpy(q, ekf->x, sizeof(float32_t) * 4U);
}

void ekf_attitude_get_gyro_bias(const ekf_attitude_t *ekf,
                                float32_t bias_xy[2])
{
    bias_xy[0] = ekf->x[4];
    bias_xy[1] = ekf->x[5];
}

float32_t ekf_attitude_get_roll(const ekf_attitude_t *ekf)
{
    const float32_t *q = ekf->x;
    return atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                  1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
}

float32_t ekf_attitude_get_pitch(const ekf_attitude_t *ekf)
{
    const float32_t *q = ekf->x;
    float32_t s = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    s = clampf(s, -1.0f, 1.0f);
    return asinf(s);
}

float32_t ekf_attitude_get_yaw(const ekf_attitude_t *ekf)
{
    const float32_t *q = ekf->x;
    return atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                  1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}
