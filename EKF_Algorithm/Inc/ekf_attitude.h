#ifndef EKF_ATTITUDE_H
#define EKF_ATTITUDE_H

#include <stdint.h>
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EKF_ATTITUDE_STATE_DIM 6U
#define EKF_ATTITUDE_MEAS_DIM 3U

typedef enum {
    EKF_ATTITUDE_OK = 0,
    EKF_ATTITUDE_PREDICT_ONLY = 1,
    EKF_ATTITUDE_REJECTED = 2,
    EKF_ATTITUDE_NUMERIC_ERROR = 3
} ekf_attitude_status_t;

typedef struct {
    float32_t q_gyro;
    float32_t q_bias;
    float32_t r_accel;
    float32_t initial_attitude_variance;
    float32_t initial_bias_variance;
    float32_t bias_fading_lambda;
    float32_t chi_square_gate;
    float32_t adaptive_gate;
    float32_t adaptive_gain_min;
    float32_t max_bias_correction;
    float32_t accel_norm_min;
    float32_t accel_norm_max;
    float32_t gyro_static_limit;
    uint16_t force_update_after_rejects;
} ekf_attitude_config_t;

typedef struct {
    float32_t x[EKF_ATTITUDE_STATE_DIM];
    float32_t P[EKF_ATTITUDE_STATE_DIM * EKF_ATTITUDE_STATE_DIM];
    float32_t Q[EKF_ATTITUDE_STATE_DIM * EKF_ATTITUDE_STATE_DIM];
    float32_t R[EKF_ATTITUDE_MEAS_DIM * EKF_ATTITUDE_MEAS_DIM];
    ekf_attitude_config_t cfg;
    uint16_t reject_count;
    float32_t last_chi_square;
    float32_t last_accel_norm;
    float32_t last_gain_scale;

    float32_t F[36];
    float32_t Ft[36];
    float32_t H[18];
    float32_t Ht[18];
    float32_t S[9];
    float32_t S_inv[9];
    float32_t HP[18];
    float32_t PHt[18];
    float32_t K[18];
    float32_t KH[36];
    float32_t I_KH[36];
    float32_t tmp66_a[36];
    float32_t tmp66_b[36];
} ekf_attitude_t;

void ekf_attitude_default_config(ekf_attitude_config_t *cfg);
void ekf_attitude_init(ekf_attitude_t *ekf,
                       const ekf_attitude_config_t *cfg);
void ekf_attitude_set_quaternion(ekf_attitude_t *ekf,
                                 float32_t q0,
                                 float32_t q1,
                                 float32_t q2,
                                 float32_t q3);
void ekf_attitude_set_gyro_bias(ekf_attitude_t *ekf,
                                float32_t bias_x,
                                float32_t bias_y);
ekf_attitude_status_t ekf_attitude_step(ekf_attitude_t *ekf,
                                        const float32_t gyro_rad_s[3],
                                        const float32_t accel[3],
                                        float32_t dt_s);
void ekf_attitude_get_quaternion(const ekf_attitude_t *ekf,
                                 float32_t q[4]);
void ekf_attitude_get_gyro_bias(const ekf_attitude_t *ekf,
                                float32_t bias_xy[2]);
float32_t ekf_attitude_get_roll(const ekf_attitude_t *ekf);
float32_t ekf_attitude_get_pitch(const ekf_attitude_t *ekf);
float32_t ekf_attitude_get_yaw(const ekf_attitude_t *ekf);

#ifdef __cplusplus
}
#endif

#endif
