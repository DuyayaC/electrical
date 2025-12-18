Class_PID chassis[4];

#include "chassis_control_mode.h"


void chassis_control(float target_vx, float target_vy, float target_omega, float theta, CHASSIS_CONTROL_MODE MODE)
{
    switch(MODE)
    {
        case CHASSIS_FOLLOW:
            chassis_follow(float target_vx, float target_vy);
            break;
        case CHASSIS_ZERO_FORCE:
            chassis_zero_force();
            break;
        case CHASSIS_STATIC:
            chassis_static(target_vx, target_vy);
            break;
        case CHASSIS_TOP:
            chassis_top(target_omega);
            break;
        case CHASSIS_TOP_MOVING:
            chassis_top_moving(target_vx, target_vy, target_omega, theta);
            break;
    }
}

//底盘无力
static void chassis_zero_force(void)
{
    //不输入电流
    CAN_cmd_chassis(0, 0, 0, 0);
}

//底盘静止
static void chassis_static(void)
{
    float target_vx = 0.0f;
    float target_vy = 0.0f;
    float target_omega = 0.0f;
    chassis_coordinate_resolution(target_vx, target_vy, target_omega, &chassis_motion);
    chassis_calculation();
    CAN_cmd_chassis(chassis_motion.motor_output_value[0], chassis_motion.motor_output_value[1], chassis_motion.motor_output_value[2], chassis_motion.motor_output_value[3]);
}

//底盘跟随
static void chassis_follow(float target_vx, float target_vy, float target_omega)
{
    chassis_coordinate_resolution(target_vx, target_vy, target_omega, &chassis_motion);
    chassis_calculation();
    CAN_cmd_chassis(chassis_motion.motor_output_value[0], chassis_motion.motor_output_value[1], chassis_motion.motor_output_value[2], chassis_motion.motor_output_value[3]);
}

//小陀螺
static void chassis_top(float target_omega)
{
    top_stand_still(target_omega, &chassis_motion);
    chassis_calculation();
    CAN_cmd_chassis(chassis_motion.motor_output_value[0], chassis_motion.motor_output_value[1], chassis_motion.motor_output_value[2], chassis_motion.motor_output_value[3]);
}

//小陀螺行进
static void chassis_top_moving(float target_vx, float target_vy, float target_omega, float theta)
{

    top_moving(target_vx, target_vy, target_omega, theta, &chassis_motion);
    chassis_calculation();
    CAN_cmd_chassis(chassis_motion.motor_output_value[0], chassis_motion.motor_output_value[1], chassis_motion.motor_output_value[2], chassis_motion.motor_output_value[3]);

}

static void chassis_calculation(void)
{
    chassis[0].Set_Values(chassis_motion.wheel_target_omega[0], m3508.motor_omega[0]);
    chassis[1].Set_Values(chassis_motion.wheel_target_omega[0], m3508.motor_omega[1]);
    chassis[2].Set_Values(chassis_motion.wheel_target_omega[0], m3508.motor_omega[2]);
    chassis[3].Set_Values(chassis_motion.wheel_target_omega[0], m3508.motor_omega[3]);

    chassis[0].Calculate(&chassis_motion.motor_output_value[0]);
    chassis[1].Calculate(&chassis_motion.motor_output_value[1]);
    chassis[2].Calculate(&chassis_motion.motor_output_value[2]);
    chassis[3].Calculate(&chassis_motion.motor_output_value[3]);
}

void chassis_init(void)
{
    chassis[0].Init(35.0f, 25.0f, 0.0f, 0.0f, 6000.0f, 16384.0f);
    chassis[1].Init(35.0f, 25.0f, 0.0f, 0.0f, 6000.0f, 16384.0f);
    chassis[2].Init(35.0f, 25.0f, 0.0f, 0.0f, 6000.0f, 16384.0f);
    chassis[3].Init(35.0f, 25.0f, 0.0f, 0.0f, 6000.0f, 16384.0f);
}