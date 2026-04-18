#ifndef CHASSIS_RESOLUTION_H
#define CHASSIS_RESOLUTION_H

#include "Robot_Property.h"
#include "arm_math.h"

/*
MECANUM 0
OMNI 1
*/

#define CHASSIS_TYPE 1

#if (CHASSIS_TYPE == 0)
#define WHEEL_RADIUS
#define GEAR_RATIO 19.20320856f
#define dx 
#define dy 
#elif (CHASSIS_TYPE == 1)
#define WHEEL_TO_CORE_DISTANCE 0.22873f
#define WHEEL_RADIUS 0.075f
#define GEAR_RATIO 19.20320856f
#endif

typedef struct 
{   
    float chassis_gimbal_deltatheta; 
    float wheel_target_omega[4];
    float chassis_omega; 
    float chassis_target_omega;
    float chassis_target_vx;
    float chassis_target_vy;
    float motor_target_omega[4];
    float motor_output_value[4];
}__attribute__((aligned(4))) chassis_motion_value_t ;
extern chassis_motion_value_t chassis_motion;

extern void chassis_coordinate_resolution(float robot_target_vx, float robot_target_vy, float robot_target_omega, chassis_motion_value_t *chassis_motion);
extern void gimbal_coordinate_resolution(float gimbal_target_vx, float gimbal_target_vy, float chassis_target_omega, float theta, chassis_motion_value_t *chassis_motion);
extern void top_stand_still(float chassis_target_omega, chassis_motion_value_t *chassis_motion);
extern void top_moving(float gimbal_target_vx, float gimbal_target_vy, float chassis_target_omega, float theta);

#endif
