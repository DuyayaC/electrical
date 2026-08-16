#include "imu_ekf_app.h"

#include <string.h>

static void copy_config(imu_ekf_app_config_t *dst,
                        const imu_ekf_app_config_t *src)
{
    if (src != NULL) {
        *dst = *src;
    } else {
        imu_ekf_app_default_config(dst);
    }
}

static void app_delay_ms(const imu_ekf_app_t *app, uint32_t ms)
{
    if (app->imu.bus.delay_ms != NULL) {
        app->imu.bus.delay_ms(app->imu.bus.user, ms);
    }
}

static float32_t elapsed_s(uint32_t now_us, uint32_t prev_us)
{
    const uint32_t diff_us = now_us - prev_us;
    return (float32_t)diff_us * 1.0e-6f;
}

static imu_ekf_app_status_t map_ekf_status(ekf_attitude_status_t st)
{
    switch (st) {
    case EKF_ATTITUDE_OK:
        return IMU_EKF_APP_OK;
    case EKF_ATTITUDE_PREDICT_ONLY:
        return IMU_EKF_APP_PREDICT_ONLY;
    case EKF_ATTITUDE_REJECTED:
        return IMU_EKF_APP_REJECTED;
    case EKF_ATTITUDE_NUMERIC_ERROR:
    default:
        return IMU_EKF_APP_ERROR_EKF;
    }
}

static imu_ekf_app_status_t calibrate_gyro_bias(imu_ekf_app_t *app)
{
    float32_t sum[3] = {0.0f, 0.0f, 0.0f};
    uint16_t good = 0U;

    for (uint16_t i = 0U; i < app->cfg.gyro_bias_samples; ++i) {
        bmi088_data_t data;
        const bmi088_status_t st = bmi088_read_converted(&app->imu, &data);
        if (st != BMI088_OK) {
            return IMU_EKF_APP_ERROR_IMU;
        }

        sum[0] += data.gyro_rad_s[0];
        sum[1] += data.gyro_rad_s[1];
        sum[2] += data.gyro_rad_s[2];
        good++;

        if (app->cfg.gyro_bias_sample_interval_ms > 0U) {
            app_delay_ms(app, app->cfg.gyro_bias_sample_interval_ms);
        }
    }

    if (good == 0U) {
        app->gyro_bias_rad_s[0] = 0.0f;
        app->gyro_bias_rad_s[1] = 0.0f;
        app->gyro_bias_rad_s[2] = 0.0f;
        return IMU_EKF_APP_OK;
    }

    const float32_t inv_good = 1.0f / (float32_t)good;
    app->gyro_bias_rad_s[0] = sum[0] * inv_good;
    app->gyro_bias_rad_s[1] = sum[1] * inv_good;
    app->gyro_bias_rad_s[2] = sum[2] * inv_good;
    ekf_attitude_set_gyro_bias(&app->ekf,
                               app->gyro_bias_rad_s[0],
                               app->gyro_bias_rad_s[1]);
    return IMU_EKF_APP_OK;
}

void imu_ekf_app_default_config(imu_ekf_app_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }

    bmi088_default_config(&cfg->bmi088);
    ekf_attitude_default_config(&cfg->ekf);
    cfg->gyro_bias_samples = 200U;
    cfg->gyro_bias_sample_interval_ms = 2U;
    cfg->min_dt_s = 0.0002f;
    cfg->max_dt_s = 0.02f;
}

imu_ekf_app_status_t imu_ekf_app_init(imu_ekf_app_t *app,
                                      const bmi088_bus_t *bus,
                                      const imu_ekf_app_config_t *cfg,
                                      uint32_t now_us)
{
    bmi088_status_t imu_st;

    if ((app == NULL) || (bus == NULL)) {
        return IMU_EKF_APP_ERROR_NULL;
    }

    memset(app, 0, sizeof(*app));
    copy_config(&app->cfg, cfg);

    if ((app->cfg.min_dt_s <= 0.0f)
        || (app->cfg.max_dt_s <= app->cfg.min_dt_s)) {
        return IMU_EKF_APP_ERROR_DT;
    }

    imu_st = bmi088_init(&app->imu, bus, &app->cfg.bmi088);
    if (imu_st != BMI088_OK) {
        return IMU_EKF_APP_ERROR_IMU;
    }

    ekf_attitude_init(&app->ekf, &app->cfg.ekf);

    const imu_ekf_app_status_t cal_st = calibrate_gyro_bias(app);
    if (cal_st != IMU_EKF_APP_OK) {
        return cal_st;
    }

    app->last_update_us = now_us;
    app->ready = 1U;
    return IMU_EKF_APP_OK;
}

imu_ekf_app_status_t imu_ekf_app_update(imu_ekf_app_t *app,
                                        uint32_t now_us)
{
    bmi088_status_t imu_st;
    ekf_attitude_status_t ekf_st;
    float32_t dt_s;

    if (app == NULL) {
        return IMU_EKF_APP_ERROR_NULL;
    }
    if (app->ready == 0U) {
        return IMU_EKF_APP_ERROR_IMU;
    }

    dt_s = elapsed_s(now_us, app->last_update_us);
    if (dt_s < app->cfg.min_dt_s) {
        return IMU_EKF_APP_ERROR_DT;
    }
    if (dt_s > app->cfg.max_dt_s) {
        dt_s = app->cfg.max_dt_s;
    }
    app->last_update_us = now_us;

    imu_st = bmi088_read_raw(&app->imu, &app->last_raw);
    if (imu_st != BMI088_OK) {
        return IMU_EKF_APP_ERROR_IMU;
    }
    bmi088_convert(&app->imu, &app->last_raw, &app->last_data);

    app->corrected_gyro_rad_s[0] = app->last_data.gyro_rad_s[0];
    app->corrected_gyro_rad_s[1] = app->last_data.gyro_rad_s[1];
    app->corrected_gyro_rad_s[2] =
        app->last_data.gyro_rad_s[2] - app->gyro_bias_rad_s[2];

    ekf_st = ekf_attitude_step(&app->ekf,
                               app->corrected_gyro_rad_s,
                               app->last_data.accel_m_s2,
                               dt_s);

    app->roll = ekf_attitude_get_roll(&app->ekf);
    app->pitch = ekf_attitude_get_pitch(&app->ekf);
    app->yaw = ekf_attitude_get_yaw(&app->ekf);

    return map_ekf_status(ekf_st);
}

void imu_ekf_app_get_rpy(const imu_ekf_app_t *app,
                         float32_t *roll,
                         float32_t *pitch,
                         float32_t *yaw)
{
    if (app == NULL) {
        return;
    }

    if (roll != NULL) {
        *roll = app->roll;
    }
    if (pitch != NULL) {
        *pitch = app->pitch;
    }
    if (yaw != NULL) {
        *yaw = app->yaw;
    }
}

void imu_ekf_app_get_quaternion(const imu_ekf_app_t *app, float32_t q[4])
{
    if ((app == NULL) || (q == NULL)) {
        return;
    }

    ekf_attitude_get_quaternion(&app->ekf, q);
}

void imu_ekf_app_get_gyro_bias(const imu_ekf_app_t *app,
                               float32_t bias_rad_s[3])
{
    if ((app == NULL) || (bias_rad_s == NULL)) {
        return;
    }

    ekf_attitude_get_gyro_bias(&app->ekf, bias_rad_s);
    bias_rad_s[2] = app->gyro_bias_rad_s[2];
}
