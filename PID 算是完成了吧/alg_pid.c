#include "alg_pid.h"

/* --- 内部集成数学函数 (Static) --- */

/**
 * @brief 求绝对值
 */
static float PID_Math_Abs(float x)
{
    return (x > 0) ? x : -x;
}

/**
 * @brief 限幅函数
 */
static void PID_Math_Constrain(float *x, float Min, float Max)
{
    if (*x < Min)
    {
        *x = Min;
    }
    else if (*x > Max)
    {
        *x = Max;
    }
}

/* --- PID 核心功能实现 --- */

/**
 * @brief PID 初始化
 */

 void PID_Init(PID_t *pid, float kp, float ki, float kd, float kf, float i_max, float out_max, float dead_zone)
{
    pid->K_P = kp;
    pid->K_I = ki;
    pid->K_D = kd;
    pid->K_F = kf;
    pid->I_Out_Max = i_max;
    pid->Out_Max = out_max;
    pid->D_T = 0.001f;
    pid->Dead_Zone = dead_zone;
    
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
    abs_error = PID_Math_Abs(error); 

    // 判断死区
    if (abs_error < pid->Dead_Zone)
    {
        error = 0.0f;
        abs_error = 0.0f;
    }

    // P 项计算
    p_out = pid->K_P * error;

    // I 项计算 
    float new_integral = pid->Integral_Error + pid->D_T * error;
    
    if (pid->I_Out_Max != 0.0f && pid->K_I != 0.0f)
    {
        float integral_min = -pid->I_Out_Max / pid->K_I;
        float integral_max = pid->I_Out_Max / pid->K_I;
        PID_Math_Constrain(&new_integral, integral_min, integral_max);
    }
    pid->Integral_Error = new_integral;
    i_out = pid->K_I * pid->Integral_Error;

    // D 项计算 (标准微分)
    d_out = pid->K_D * (error - pid->Pre_Error) / pid->D_T;

    // F前馈计算
    f_out = (pid->Target - pid->Pre_Target) * pid->K_F;

    // 总输出
    *output = p_out + i_out + d_out + f_out;
    
    // 输出限幅
    if (pid->Out_Max != 0.0f)
    {
        PID_Math_Constrain(output, -pid->Out_Max, pid->Out_Max);
    }

    // 更新历史状态
    pid->Pre_Now = pid->Now;
    pid->Pre_Target = pid->Target;
    pid->Pre_Out = *output;
    pid->Pre_Error = error;
}