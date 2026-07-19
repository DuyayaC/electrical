#ifndef ALG_PID_H
#define ALG_PID_H

#include "stdint.h" 

/* PID 结构体定义 */
typedef struct
{
    // PID 参数
    float K_P;
    float K_I;
    float K_D;
    float K_F;          // 前馈系数
    float I_Out_Max;    // 积分限幅
    int16_t Out_Max;      // 输出限幅
    float D_T;          // 时间片长度 (控制周期)
    float Dead_Zone;    // 死区

    // 状态变量
    float Target;       // 目标值
    float Now;          // 当前测量值
    float Pre_Now;      // 上次测量值
    float Pre_Target;   // 上次目标值
    float Pre_Out;      // 上次输出
    float Pre_Error;    // 上次误差
    float Integral_Error; // 累计积分值
} PID_t;

/* 函数声明 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float kf, float i_max, float out_max, float dead_zone);
void PID_Set_Values(PID_t *pid, float target, float now);
void PID_Clear_Error(PID_t *pid);
void PID_Calculate(PID_t *pid, float *output);

#endif