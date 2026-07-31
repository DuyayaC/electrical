#include "alg_pid.h"

/* --- PID 核心功能实现 --- */

/**
 * @brief PID 初始化
 */

 void PID_Init(PID_t *pid, float kp, float ki, float kd, float kf, float i_max, float out_max, float dead_zone, bool_e derivative_first)
{
    pid->K_P = kp;
    pid->K_I = ki;
    pid->K_D = kd;
    pid->K_F = kf;
    pid->I_Out_Max = i_max;
    pid->Out_Max = out_max;
    pid->Dead_Zone = dead_zone;
    pid->Derivative_First = derivative_first;   // 微分先行：TRUE=对测量值微分，FALSE=对误差微分
    
    PID_Clear_Error(pid); 
}

/**
 * @brief 设置目标值和当前值
 */
void PID_Set_Values(PID_t *pid, float target, float now)
{
    pid->Target = target;
    pid->Now = now;
}

/**
 * @brief 清除误差和状态
 */
void PID_Clear_Error(PID_t *pid)
{
    pid->Pre_Now = 0.0f;
    pid->Pre_Target = 0.0f;
    pid->Pre_Out = 0.0f;
    pid->Pre_Error = 0.0f;
    pid->Integral_Error = 0.0f;
}

/**
 * @brief PID 计算函数
 */
void PID_Calculate(PID_t *pid, float *output)
{
    float p_out = 0.0f;
    float i_out = 0.0f;
    float d_out = 0.0f;
    float f_out = 0.0f;
    float error;
    float abs_error;

    // 计算当前误差
    error = pid->Target - pid->Now;
    abs_error = ABS(error); 

    // 判断死区
    if (abs_error < pid->Dead_Zone)
    {
        error = 0.0f;
        abs_error = 0.0f;
    }

    // P 项计算
    p_out = pid->K_P * error;

    // I 项计算（离散累加）
    float new_integral = pid->Integral_Error + error;
    
    if (pid->I_Out_Max != 0.0f && pid->K_I != 0.0f)
    {
        float integral_min = -pid->I_Out_Max / pid->K_I;
        float integral_max = pid->I_Out_Max / pid->K_I;
        new_integral = CLAMP(new_integral, integral_min, integral_max);
    }
    pid->Integral_Error = new_integral;
    i_out = pid->K_I * pid->Integral_Error;

    // D 项计算（离散差分；微分先行时对测量值微分，避免目标突变引起微分冲击）
    if (pid->Derivative_First)
        d_out = pid->K_D * (pid->Pre_Now - pid->Now);   // 微分先行：对测量值微分
    else
        d_out = pid->K_D * (error - pid->Pre_Error);    // 标准：对误差微分

    // F前馈计算
    f_out = (pid->Target - pid->Pre_Target) * pid->K_F;

    // 总输出
    *output = p_out + i_out + d_out + f_out;
    
    // 输出限幅
    if (pid->Out_Max != 0.0f)
    {
        *output = CLAMP(*output, -pid->Out_Max, pid->Out_Max);
    }

    // 更新历史状态
    pid->Pre_Now = pid->Now;
    pid->Pre_Target = pid->Target;
    pid->Pre_Out = *output;
    pid->Pre_Error = error;
}