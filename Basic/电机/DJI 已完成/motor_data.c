#include "motor_data.h"

void motor_value_updata(void)
{
    m2006_update(&m2006);
    m3508_update(&m3508);
    gm6020_update(&gm6020);
}

static void m3508_update(M3508_t *m3508)
{
    chassis_m3508_update(&m3508);
    friction_m3508_update(&m3508);
}
#if (Robot_ID == 1)
static void chassis_m3508_update(M3508_t *m3508)
{
    for (int i = 0; i < 4; i++)
    {
        volatile motor_measure_t *motor = get_chassis_motor_measure_point(i);

        m3508->motor_current[i] = motor->given_current;
        m3508->motor_ecd[i] = motor->ecd;
        m3508->motor_last_ecd[i] = motor->last_ecd;
        m3508->motor_omega[i] = motor->speed_rpm * 6.0f;
        m3508->motor_temperature[i] = motor->temperate;
    }
}

static void friction_m3508_update(M3508_t *m3508)
{
    for (int i = 0; i < 4; i++)
    {
        volatile motor_measure_t *motor = get_friction_motor_measure_point(i);

        m3508->motor_current[i+4] = motor->given_current;
        m3508->motor_ecd[i+4] = motor->ecd;
        m3508->motor_last_ecd[i+4] = motor->last_ecd;
        m3508->motor_omega[i+4] = motor->speed_rpm * 6.0f;
        m3508->motor_temperature[i+4] = motor->temperate;        
    }
}

#elif (Robot_ID == 3)
static void chassis_m3508_update(M3508_t *m3508)
{
    for (int i = 0; i < 4; i++)
    {
        volatile motor_measure_t *motor = get_chassis_motor_measure_point(i);

        m3508->motor_current[i] = motor->given_current;
        m3508->motor_ecd[i] = motor->ecd;
        m3508->motor_last_ecd[i] = motor->last_ecd;
        m3508->motor_omega[i] = motor->speed_rpm * 6.0f;
        m3508->motor_temperature[i] = motor->temperate;
    }
}

static void friction_m3508_update(M3508_t *m3508)
{
    for (int i = 0; i < 2; i++)
    {
        volatile motor_measure_t *motor = get_friction_motor_measure_point(i);

        m3508->motor_current[i+4] = motor->given_current;
        m3508->motor_ecd[i+4] = motor->ecd;
        m3508->motor_last_ecd[i+4] = motor->last_ecd;
        m3508->motor_omega[i+4] = motor->speed_rpm * 6.0f;
        m3508->motor_temperature[i+4] = motor->temperate;        
    }
}
#endif

static void barrel_m2006_update(M2006_t)
{
    ;
}

static void trigger_m2006_update(M2006_t *m2006)
{
    volatile motor_measure_t *motor = get_trigger_motor_measure_point();

    m2006->motor_current[0] = motor->given_current;
    m2006->motor_ecd[0] = motor->ecd;
    m2006->motor_last_ecd[0] = motor->last_ecd;
    m2006->motor_omega[0] = motor->speed_rpm * 6.0f;
}

static void gm6020_update(GM6020_t *gm6020)
{
    volatile motor_measure_t *pitch = get_pitch_gimbal_motor_measure_point();
    volatile motor_measure_t *yaw = get_yaw_gimbal_motor_measure_point();

    gm6020->motor_current[0] = yaw->given_current;
    gm6020->motor_ecd[0] = yaw->ecd;
    gm6020->motor_last_ecd[0] = yaw->last_ecd;
    gm6020->motor_omega[0] = yaw->speed_rpm * 6.0f;

    gm6020->motor_current[1] = pitch->given_current;
    gm6020->motor_ecd[1] = pitch->ecd;
    gm6020->motor_last_ecd[1] = pitch->last_ecd;
    gm6020->motor_omega[1] = pitch->speed_rpm * 6.0f;

    yaw_ecd_angle();
    pitch_ecd_angle();
}

static void yaw_ecd_angle(void)
{
    float angle = (gm6020.motor_ecd[0] - YAW_ZERO_ECD) *0.0439453125f;
    float angle_temp;
    arm_clip_f32(&angle, &angle_temp, -180.0f, 180.0f, 1);
    gm6020.motor_ecd_angle[0] = angle_temp;
}

void delta_angle_dji(int *now_ecd, int *target_ecd, float delta_angle_deg)
{
    *target_ecd = *now_ecd + (int)(delta_angle_deg * 22.75555556f);
}

void delta_angle_8115(int *now_ecd, int *target_ecd, float _delta_angle)
{
    *target_ecd = (int)now_ecd + delta_angle * 45.5111111f;
}