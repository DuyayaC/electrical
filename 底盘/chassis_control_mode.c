//==================================================//
/*****************************************************
 * @brief �����˶�����
 * @author Modi CHI����ɫèèͷ��o(���ި��ޡ�)o
 * @attention ȫ���������ֵ����˶����ƴ��룬����ģʽѡ����
              �˶����Ʋ��������Լ������������
*****************************************************/
//==================================================//
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

//��������
static void chassis_zero_force(void)
{
    //�������ֵ0���������
    CAN_cmd_chassis(0, 0, 0, 0);
}

//���̾�ֹ���������
static void chassis_static(float target_vx, float target_vy)
{
    //��������������Ŀ����ٶ�
    chassis_coordinate_resolution(target_vx, target_vy, &chassis_motion);

}

//���̸���
static void chassis_follow(float target_vx, float target_vy)
{
    //��������������Ŀ����ٶ�
    chassis_coordinate_resolution(target_vx, target_vy, &chassis_motion);

}

//ԭ��С����
static void chassis_top(float target_omega)
{
    //��������������Ŀ����ٶ�
    top_stand_still(target_omega, &chassis_motion);

}

//С�����н�
static void chassis_top_moving(float target_vx, float target_vy, float target_omega, float theta)
{
    //��������������Ŀ����ٶ�
    top_moving(target_vx, target_vy, target_omega, theta, &chassis_motion);

}

static void chassis_calculation(void)
{
    ;
}

void chassis_init(void)
{
    chassis[0].Init();
}