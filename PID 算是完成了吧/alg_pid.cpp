#include "alg_pid.h"

/**
 * @brief PID初�?�化
 *
 * @param __K_P P�?
 * @param __K_I I�?
 * @param __K_D D�?
 * @param __K_F 前�??
 * @param __I_Out_Max �?分限�?
 * @param __Out_Max 输出限幅
 * @param __D_T 时间片长�?
 */
void Class_PID::Init(float __K_P, float __K_I, float __K_D, float __K_F, float __I_Out_Max, float __Out_Max, float __D_T, float __Dead_Zone, float __I_Variable_Speed_A, float __I_Variable_Speed_B, float __I_Separate_Threshold, Enum_PID_D_First __D_First)
{
    K_P = __K_P;
    K_I = __K_I;
    K_D = __K_D;
    K_F = __K_F;
    I_Out_Max = __I_Out_Max;
    Out_Max = __Out_Max;
    D_T = __D_T;
    Dead_Zone = __Dead_Zone;
    I_Variable_Speed_A = __I_Variable_Speed_A;
    I_Variable_Speed_B = __I_Variable_Speed_B;
    I_Separate_Threshold = __I_Separate_Threshold;
    D_First = __D_First;
}

/**
 * @brief 设置�?标值和当前�?
 *
 * @param __Target �?标�?
 * @param __Now 当前�?
 */
void Class_PID::Set_Values(float __Target, float __Now)
{
    Target = __Target;
    Now = __Now;
}

/**
 * @brief 清除�?�?和状�?
 *
 */
void Class_PID::Clear_Error()
{
    Pre_Now = 0.0f;
    Pre_Target = 0.0f;
    Pre_Out = 0.0f;
    Pre_Error = 0.0f;
    Integral_Error = 0.0f;
}

/**
 * @brief PID计算函数
 *
 * @param output 输出指针
 */
void Class_PID::Calculate(float *output)
{
    // P输出
    float p_out = 0.0f;
    // I输出
    float i_out = 0.0f;
    // D输出
    float d_out = 0.0f;
    // F输出
    float f_out = 0.0f;
    //�?�?
    float error;
    //绝�?�值�??�?
    float abs_error;
    //线性变速积�?
    float speed_ratio;
    
    
    error = Target - Now;
    // 使用CMSIS-DSP的绝对值函�?
    abs_error = Math_Abs(error);

    //判断死区
    if (abs_error < Dead_Zone)
    {
        error = 0.0f;
        abs_error = 0.0f;
    }

    //计算p�?
    p_out = K_P * error;

    //计算i�?
    if (I_Variable_Speed_A == 0.0f && I_Variable_Speed_B == 0.0f)
    {
        //非变速积�?
        speed_ratio = 1.0f;
    }
    else
    {
        //变速积�?
        if (abs_error <= I_Variable_Speed_B)
        {
            speed_ratio = 1.0f;
        }
        else if (abs_error < (I_Variable_Speed_A + I_Variable_Speed_B))
        {
            speed_ratio = (I_Variable_Speed_A + I_Variable_Speed_B - abs_error) / I_Variable_Speed_A;
        }
        else
        {
            speed_ratio = 0.0f;
        }
    }

    
    if (I_Separate_Threshold == 0.0f)
    {
        //没有�?分分�?
        Integral_Error += speed_ratio * D_T * error;
        i_out = K_I * Integral_Error;
    }
    else
    {
        //�?分分离使�?
        if (abs_error < I_Separate_Threshold)
        {
            Integral_Error += speed_ratio * D_T * error;
            i_out = K_I * Integral_Error;
        }
        else
        {
            Integral_Error = 0.0f;
            i_out = 0.0f;
        }
    }
    if (I_Out_Max != 0.0f)
    {
        float integral_min = -I_Out_Max / K_I;
        float integral_max = I_Out_Max / K_I;
        Math_Constrain(&Integral_Error, &Integral_Error, integral_min, integral_max);
    }

    //计算d�?
    if (D_First == PID_D_First_DISABLE)
    {
        //没有�?分先�?
        d_out = K_D * (error - Pre_Error) / D_T;
    }
    else
    {
        //�?分先行使�?
        d_out = K_D * (*output - Pre_Out) / D_T;
    }

    //计算前�??
    f_out = (Target - Pre_Target) * K_F;

    //计算总共的输�?
    *output = p_out + i_out + d_out + f_out;
    
    //输出限幅 - 使用CMSIS-DSP的限幅函�?
    if (Out_Max != 0.0f)
    {
        *output = Math_Constrain(*output, -Out_Max, Out_Max);
    }

    //善后工作
    Pre_Now = Now;
    Pre_Target = Target;
    Pre_Out = *output;
    Pre_Error = error;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/