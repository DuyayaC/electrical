#include "chassis_resolution.h"

//麦克纳姆轮底盘，简称麦轮底盘
static void chassis_wheel_inverse_resolution(chassis_motion_value_t *chassis_motion)
{
    chassis_motion->wheel_target_omega[0] = GEAR_RATIO * (+chassis_motion->chassis_target_vx - chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * (dx + dy)) / WHEEL_RADIUS;
    chassis_motion->wheel_target_omega[1] = GEAR_RATIO * (+chassis_motion->chassis_target_vx + chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * (dx + dy)) / WHEEL_RADIUS;
    chassis_motion->wheel_target_omega[2] = GEAR_RATIO * (-chassis_motion->chassis_target_vx + chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * (dx + dy)) / WHEEL_RADIUS;
    chassis_motion->wheel_target_omega[3] = GEAR_RATIO * (-chassis_motion->chassis_target_vx - chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * (dx + dy)) / WHEEL_RADIUS;
}

//全向轮底盘
static void chassis_wheel_inverse_resolution(chassis_motion_value_t *chassis_motion)
{
    chassis_motion->wheel_target_omega[0] = GEAR_RATIO * (0.707f * chassis_motion->chassis_target_vx - 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //1
    chassis_motion->wheel_target_omega[1] = GEAR_RATIO * (0.707f * chassis_motion->chassis_target_vx + 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //2
    chassis_motion->wheel_target_omega[2] = GEAR_RATIO * (-0.707f * chassis_motion->chassis_target_vx + 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //3
    chassis_motion->wheel_target_omega[3] = GEAR_RATIO * (-0.707f * chassis_motion->chassis_target_vx - 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //4
}

/*关于全向轮底盘解算以及麦克纳姆轮底盘解算，请观看 https://www.bilibili.com/video/BV1toH6ekEfJ?spm_id_from=333.788.videopod.sections&vd_source=97ad3a2e4c0b08c1bfe6107adda046bb 
请注意视频中底盘正方向与实际使用中存在180°的角度关系，因此要给视频中公式乘上系数“-1”。视频麦克纳姆轮公式中的a，b为代码中的dx，dy*/