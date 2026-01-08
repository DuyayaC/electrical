#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <math.h>
#include <stdint.h>

//物理常量
#define L1   0.145f
#define L2   0.27f
#define L3   0.27f
#define L4   0.145f
#define L5   0.15f
#define M    1.81069f           //Mass_Wheel
#define m    18.39266f          //Mass_Body
#define TOTAL_MASS 20.20335f    // M + m
#define h    0.15652f           //pendulum length
#define G    9.81f     
#define PI   3.14159265f

//结构体定义

typedef struct
{
    float q[4]; // 四元数
    float gyro[3]; // gx,gy,xz from IMU
    float alpha_L, beta_L; // 输入：左腿电机角度(rad)
    float alpha_R, beta_R; // 输入：右腿电机角度(rad)
    float x, x_vel;       //轮子位移和速度
} Robot_Sensors_t;

typedef struct
{
    float pitch, pitch_vel;
    float roll, roll_vel;
    float h_L, h_R;      // 左右腿高度
    float theta1_L, theta2_L; // 左腿内部连杆角
    float theta1_R, theta2_R; // 右腿内部连杆角
} Robot_State_t;

typedef struct
{
    float wheel_L, wheel_R; // 2个轮电机力矩
    float leg_LF, leg_RF; // 左腿前后电机力矩
    float leg_LB, leg_RB; // 右腿前后电机力矩
} Motor_Output_t;

//函数
void Update_Euler_from_Quat(Robot_Sensors_t *sensor_data, Robot_State_t *robot_state);
void Calculate_FK(float alpha, float beta, float *height, float *theta1, float *theta2);
void Jacobian_Map(float alpha, float beta, float theta1, float theta2, float Fy, float *tf, float *tb);
void Robot_Control_Loop(Robot_Sensors_t *sensor_data,Robot_State_t *robot_state, Motor_Output_t *motor_output, float target_h, float target_v);



#endif
