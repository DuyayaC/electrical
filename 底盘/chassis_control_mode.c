//==================================================//
/*****************************************************
 * @brief 底盘运动控制
 * @author Modi CHI（粉色猫猫头）o(〃＾▽＾〃)o
 * @attention 全向轮与麦轮底盘运动控制代码，整合模式选择与
              运动控制参数解算以及电机电流发送
*****************************************************/
//==================================================//
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
    //传输电流值0，电机无力
    CAN_cmd_chassis(0, 0, 0, 0);
}

//底盘静止，电机锁定
static void chassis_static(float target_vx, float target_vy)
{
    //解算底盘驱动电机目标角速度
    chassis_coordinate_resolution(target_vx, target_vy, &chassis_motion);

}

//底盘跟随
static void chassis_follow(float target_vx, float target_vy)
{
    //解算底盘驱动电机目标角速度
    chassis_coordinate_resolution(target_vx, target_vy, &chassis_motion);

}

//原地小陀螺
static void chassis_top(float target_omega)
{
    //解算底盘驱动电机目标角速度
    top_stand_still(target_omega, &chassis_motion);

}

//小陀螺行进
static void chassis_top_moving(float target_vx, float target_vy, float target_omega, float theta)
{
    //解算底盘驱动电机目标角速度
    top_moving(target_vx, target_vy, target_omega, theta, &chassis_motion);

}