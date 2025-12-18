#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

#include "drv_CAN_receive.h"
#include "chassis_resolution.h"
#include "motor_data.h"
#include "alg_pid.h"



typedef enum
{
    CHASSIS_FOLLOW, //底盘跟随
    CHASSIS_COORDINATE, //底盘系
    GIMBAL_COORDINATE, //云台系
    CHASSIS_ZERO_FORCE, //底盘无力
    CHASSIS_STATIC, //底盘静止，电机锁定位置
    CHASSIS_TOP, //原地小陀螺
    CHASSIS_TOP_MOVING, //小陀螺行进
    CHASSIS_RC_MODE, //遥控器控制模式
    CHASSIS_KEY_MODE, //键鼠控制模式
}CHASSIS_CONTROL_MODE;

//底盘模式选择，附带对应模式下的任务解算
extern void chassis_control(float target_vx, float target_vy, float target_omega, float theta, CHASSIS_CONTROL_MODE MODE);
extern void chassis_init(void);