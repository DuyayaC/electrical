#ifndef IMU_EKF_APP_H
#define IMU_EKF_APP_H

#include <stdint.h>

#include "bmi088.h"
#include "ekf_attitude.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IMU_EKF_APP_OK = 0,
    IMU_EKF_APP_PREDICT_ONLY = 1,
    IMU_EKF_APP_REJECTED = 2,
    IMU_EKF_APP_ERROR_NULL = -1,
    IMU_EKF_APP_ERROR_IMU = -2,
    IMU_EKF_APP_ERROR_EKF = -3,
    IMU_EKF_APP_ERROR_DT = -4
} imu_ekf_app_status_t;

typedef struct {
    bmi088_config_t bmi088;
    ekf_attitude_config_t ekf;
    uint16_t gyro_bias_samples;
    uint16_t gyro_bias_sample_interval_ms;
    float32_t min_dt_s;
    float32_t max_dt_s;
} imu_ekf_app_config_t;

typedef struct {
    bmi088_t imu;
    ekf_attitude_t ekf;
    imu_ekf_app_config_t cfg;
    uint32_t last_update_us;
    float32_t gyro_bias_rad_s[3];
    bmi088_raw_t last_raw;
    bmi088_data_t last_data;
    float32_t corrected_gyro_rad_s[3];
    float32_t roll;
    float32_t pitch;
    float32_t yaw;
    uint8_t ready;
} imu_ekf_app_t;

void imu_ekf_app_default_config(imu_ekf_app_config_t *cfg);
imu_ekf_app_status_t imu_ekf_app_init(imu_ekf_app_t *app,
                                      const bmi088_bus_t *bus,
                                      const imu_ekf_app_config_t *cfg,
                                      uint32_t now_us);
imu_ekf_app_status_t imu_ekf_app_update(imu_ekf_app_t *app,
                                        uint32_t now_us);
void imu_ekf_app_get_rpy(const imu_ekf_app_t *app,
                         float32_t *roll,
                         float32_t *pitch,
                         float32_t *yaw);
void imu_ekf_app_get_quaternion(const imu_ekf_app_t *app, float32_t q[4]);
void imu_ekf_app_get_gyro_bias(const imu_ekf_app_t *app,
                               float32_t bias_rad_s[3]);

#ifdef __cplusplus
}
#endif

#endif
