#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "drv_CAN_receive.h"
#include "ZeroCheck.h"
#include "alg_pid.h"
#include "control_task.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "calibrate_task.h"

#define WHEEL_TO_CORE_DISTANCE 0.22873f
#define WHEEL_RADIUS 0.075f
#define GEAR_RATIO 19.20320856f
#define init_ecd 2700.0f

typedef struct 
{   
    float gimbal_target_vx;
    float gimbal_target_vy;
    float wheel_target_omega[4]; 
    float chassis_target_omega;
    float chassis_target_vx;
    float chassis_target_vy;
    float motor_output_value[4];
	float wheel_now_omega[4];
	float now_ecd;
	float theta;
}__attribute__((aligned(4))) chassis_motion_value_t ;
extern chassis_motion_value_t chassis_motion;

void CHASSIS_TASK(void const * argument);

#endif