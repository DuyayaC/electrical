//==============================//
/*
*/
//==============================//
#ifndef MOTOR_DATA
#define MOTOR_DATA

#include "drv_CAN_receive.h"
#include "arm_math.h"
#include <math.h>
#include "Robot_Property.h"

#define YAW_ZERO_ECD
#define PITCH_ZERO_ECD

#if (Robot_ID == 1)
typedef struct 
{
    float motor_omega[8];
    float motor_current[8];
    float motor_ecd[8];
    float motor_last_ecd[8];
    float motor_temperature[8];
    float motor_ecd_angle[8]
}M3508_t __attribute__((aligned(4)));
extern M3508_t m3508;

#elif ((Robot_ID == 3) || (Robot_ID == 4))
typedef struct 
{
    float motor_omega[6];
    float motor_current[6];
    float motor_ecd[6];
    float motor_last_ecd[6];
    float motor_temperature[6];
    float motor_ecd_angle[6]
}M3508_t __attribute__((aligned(4)));
extern M3508_t m3508;
#endif

typedef struct 
{
    float motor_omega[2];
    float motor_current[2];
    float motor_ecd[2];
    float motor_last_ecd[2];
    float motor_ecd_angle[2];
}GM6020_t __attribute__((aligned(4)));
extern GM6020_t gm6020;

typedef struct 
{
    float motor_omega[2];
    float motor_current[2];
    float motor_ecd[2];
    float motor_last_ecd[2];
    float motor_ecd_angle[2];
}M2006_t __attribute__((aligned(4)));
extern M2006_t m2006;

typedef struct 
{
    ;//��֪��дʲô�������ڱ����д ( ?? �� ?? )?
}GIM8115_t;
extern GIM8115_t gim8115;

extern void motor_value_updata(MOTOR_NAME NAME);
extern void delta_angle_dji(float *now_ecd, float delta_angle, float *target_ecd);
extern void delta_angle_8115(float *now_ecd, float delta_angle, float *target_ecd);