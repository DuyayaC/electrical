#include "chassis_control_mode.h"


static void chassis_calculation(PID_t *chassis)
{
    
}

//��������
static void chassis_zero_force(void)
{
    //���������?
    CAN_cmd_chassis(0, 0, 0, 0);
}

//���̾�ֹ
static void chassis_static(void)
{
    float target_vx = 0.0f;
    float target_vy = 0.0f;
    float target_omega = 0.0f;
    chassis_coordinate_resolution(target_vx, target_vy, target_omega, &chassis_motion);
    chassis_calculation();
    CAN_cmd_chassis((int16_t)chassis_motion.motor_output_value[0], (int16_t)chassis_motion.motor_output_value[1], 
                    (int16_t)chassis_motion.motor_output_value[2], (int16_t)chassis_motion.motor_output_value[3]);
}

//���̸���
static void chassis_follow(float target_vx, float target_vy, float target_omega)
{
    chassis_coordinate_resolution(target_vx, target_vy, target_omega, &chassis_motion);
    chassis_calculation();
    CAN_cmd_chassis((int16_t)chassis_motion.motor_output_value[0], (int16_t)chassis_motion.motor_output_value[1], 
                    (int16_t)chassis_motion.motor_output_value[2], (int16_t)chassis_motion.motor_output_value[3]);
}

//С����
static void chassis_top(float target_omega)
{
    top_stand_still(target_omega, &chassis_motion);
    chassis_calculation();
    CAN_cmd_chassis((int16_t)chassis_motion.motor_output_value[0], (int16_t)chassis_motion.motor_output_value[1], 
                    (int16_t)chassis_motion.motor_output_value[2], (int16_t)chassis_motion.motor_output_value[3]);
}

//С�����н�
static void chassis_top_moving(float target_vx, float target_vy, float target_omega, float theta)
{

    top_moving(target_vx, target_vy, target_omega, theta);
    chassis_calculation();
    CAN_cmd_chassis((int16_t)chassis_motion.motor_output_value[0], (int16_t)chassis_motion.motor_output_value[1], 
                    (int16_t)chassis_motion.motor_output_value[2], (int16_t)
	chassis_motion.motor_output_value[3]);

}

void chassis_control(float target_vx, float target_vy, float target_omega, float theta, CHASSIS_CONTROL_MODE MODE)
{
    switch(MODE)
    {
        case CHASSIS_FOLLOW:
            chassis_follow(target_vx, target_vy, target_omega);
            break;
        case CHASSIS_ZERO_FORCE:
            chassis_zero_force();
            break;
        case CHASSIS_STATIC:
            chassis_static();
            break;
        case CHASSIS_TOP:
            chassis_top(target_omega);
            break;
        case CHASSIS_TOP_MOVING:
            chassis_top_moving(target_vx, target_vy, target_omega, theta);
            break;
    }
}

void chassis_init(void)
{
    ;
}