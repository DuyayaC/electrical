#include "chassis_resolution.h"

#if (CHASSIS_TYPE == 'MECANUM')
static void chassis_wheel_inverse_resolution(chassis_motion_value_t *chassis_motion)
{
    chassis_motion->wheel_target_omega[0] = GEAR_RATIO * (+chassis_motion->chassis_target_vx - chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * (dx + dy)) / WHEEL_RADIUS;
    chassis_motion->wheel_target_omega[1] = GEAR_RATIO * (+chassis_motion->chassis_target_vx + chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * (dx + dy)) / WHEEL_RADIUS;
    chassis_motion->wheel_target_omega[2] = GEAR_RATIO * (-chassis_motion->chassis_target_vx + chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * (dx + dy)) / WHEEL_RADIUS;
    chassis_motion->wheel_target_omega[3] = GEAR_RATIO * (-chassis_motion->chassis_target_vx - chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * (dx + dy)) / WHEEL_RADIUS;
}

#elif (CHASSIS_TYPE == 'OMNI')
static void chassis_wheel_inverse_resolution(chassis_motion_value_t *chassis_motion)
{
    chassis_motion->wheel_target_omega[0] = GEAR_RATIO * (0.707f * chassis_motion->chassis_target_vx - 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //1
    chassis_motion->wheel_target_omega[1] = GEAR_RATIO * (0.707f * chassis_motion->chassis_target_vx + 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //2
    chassis_motion->wheel_target_omega[2] = GEAR_RATIO * (-0.707f * chassis_motion->chassis_target_vx + 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //3
    chassis_motion->wheel_target_omega[3] = GEAR_RATIO * (-0.707f * chassis_motion->chassis_target_vx - 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //4
}
#endif

//底盘系解算
void chassis_coordinate_resolution(float robot_target_vx, float robot_target_vy, float robot_target_omega, chassis_motion_value_t *chassis_motion)
{
    chassis_motion->chassis_target_vx = robot_target_vx;
    chassis_motion->chassis_target_vy = robot_target_vy;
    chassis_motion->chassis_target_omega = robot_target_omega;
    chassis_wheel_inverse_resolution(chassis_motion);
}

//云台系解算
void gimbal_coordinate_resolution(float gimbal_target_vx, float gimbal_target_vy, float chassis_target_omega, float theta, chassis_motion_value_t *chassis_motion)
{   
    float32_t sin_theta, cos_theta;
    arm_sin_cos_f32(theta, &sin_theta, &cos_theta); 
    chassis_motion->chassis_gimbal_deltatheta = cos_theta;
    chassis_motion->chassis_target_vx = cos_theta * gimbal_target_vx - sin_theta * gimbal_target_vy;
    chassis_motion->chassis_target_vy = sin_theta * gimbal_target_vx + cos_theta * gimbal_target_vy;
    chassis_motion->chassis_target_omega = chassis_target_omega;
    chassis_wheel_inverse_resolution(chassis_motion);
}

//原地小陀螺
void top_stand_still(float chassis_target_omega, chassis_motion_value_t *chassis_motion)
{
    chassis_motion->chassis_target_vx = 0.0f;
    chassis_motion->chassis_target_vy = 0.0f;
    chassis_motion->chassis_target_omega = chassis_target_omega;
    chassis_wheel_inverse_resolution(chassis_motion);
}

//小陀螺行进
#if (Robot_ID != 5)
void top_moving(float gimbal_target_vx, float gimbal_target_vy, float chassis_target_omega, float theta, chassis_motion_value_t *chassis_motion)
{
    gimbal_coordinate_resolution(gimbal_target_vx, gimbal_target_vy, chassis_target_omega, theta, chassis_motion);
}
#endif
