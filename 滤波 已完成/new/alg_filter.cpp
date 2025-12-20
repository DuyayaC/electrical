#include "alg_filter.h"

void Class_IIR::Init(float __alpha)
{
    alpha == __alpha;
    last_data = 0.0f;
}

void Class_IIR::In_Calculate_Out(float *data)
{
    *data = alpha * (*data) + (1 - alpha) * last_data;
    last_data = *data;
}

//单变量卡尔曼滤波
void Class_Filter_Kalman::Init(float a, float b, float h, float q, float r)
{
    A = a;
    B = b;
    H = h;
    Q = q;
    R = r;
}

/**
 * @brief 滤波器调整值, 周期与采样周期相同
 *
 */
void Class_Filter_Kalman::TIM_Calculate()
{   
    //predict update
    Out = A * Out;
    _P = A * P * A +Q;

    //actual update
    G = _P * H / (H * _P * H + R);
    Out = Out + G * (Now - H * Out);
    P = (1.0f - G * H) * _P;
}

//三轴卡尔曼滤波器
void Class_Filter_XYZ_Kalman::Init()
{
    A = 1.0f;
    B = 0.0f;
    H = 1.0f;
    
    // 初始化矩阵实例
    arm_mat_init_f32(&mat_P, 1, 1, P);   // 标量视为1x1矩阵
    arm_mat_init_f32(&mat_Q, 1, 1, Q);
    arm_mat_init_f32(&mat_R, 1, 1, R);
}

void Class_Filter_XYZ_Kalman_Optimized::TIM_Calculate_DSP_Batch()
{
    // 批量处理三个轴的数据
    float32_t A_squared = A * A;
    
    // 使用DSP库的批量操作
    // 1. 批量预测更新
    arm_scale_f32(Out, A, Out, 3);
    
    // 2. 批量协方差预测
    for(int i = 0; i < 3; i++) {
        _P[i] = A_squared * P[i] + Q[i];
    }
    
    // 3. 批量卡尔曼增益计算
    for(int i = 0; i < 3; i++) {
        float32_t denominator = H * H * _P[i] + R[i];
        G[i] = (H * _P[i]) / denominator;
    }
    
    // 4. 批量状态更新
    float32_t temp[3];
    arm_scale_f32(Out, H, temp, 3);          // temp = H * Out
    arm_sub_f32(Now, temp, temp, 3);         // temp = Now - temp (innovation)
    
    for(int i = 0; i < 3; i++) {
        Out[i] = Out[i] + G[i] * temp[i];    // 状态更新
        P[i] = (1.0f - G[i] * H) * _P[i];    // 协方差更新
    }
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
