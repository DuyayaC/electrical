#include "alg_filter.h"

/*IIR 滤波器*/
void IIR_Init(IIR_Filter_t *filter, float alpha)
{
    filter->alpha = alpha;
    filter->last_data = 0.0f;
}

void IIR_Calculate(IIR_Filter_t *filter, float *data)
{
    *data = filter->alpha * (*data) + (1.0f - filter->alpha) * filter->last_data;
    filter->last_data = *data;
}

void IIR_Clear(IIR_Filter_t *filter)
{
    filter->last_data = 0.0f;
}

/*单维卡尔曼滤波器*/
void Kalman_Init(Kalman_Filter_t *filter, float a, float b, float h, float q, float r)
{
    filter->A = a;
    filter->B = b;
    filter->H = h;
    filter->Q = q;
    filter->R = r;
    filter->P = 1.0f; // 初始估计误差协方差
    filter->Out = 0.0f; // 初始滤波输出
}

void Kalman_Calculate(Kalman_Filter_t *filter)
{   
    // 预测更新
    filter->Out = filter->A * filter->Out;
    filter->P_predict = filter->A * filter->P * filter->A + filter->Q;

    // 实际更新
    float denom = filter->H * filter->P_predict * filter->H + filter->R;
    filter->G = filter->P_predict * filter->H / denom;
    filter->Out = filter->Out + filter->G * (filter->Now - filter->H * filter->Out);
    filter->P = (1.0f - filter->G * filter->H) * filter->P_predict;
}

/*XYZ三轴卡尔曼滤波器*/
void XYZ_Kalman_Init(XYZ_Kalman_Filter_t *filter, float a, float b, float h)
{
    filter->A = a;
    filter->B = b;
    filter->H = h;

    //初始化数组和矩阵实例
    for (int i = 0; i < 3; i++) {
        filter->P[i] = 1.0f; // 初始估计误差协方差
        filter->Out[i] = 0.0f; // 初始滤波输出
    }

    // 初始化矩阵实例
    arm_mat_init_f32(&filter->mat_P, 1, 1, filter->P);   
    arm_mat_init_f32(&filter->mat_Q, 1, 1, filter->Q);
    arm_mat_init_f32(&filter->mat_R, 1, 1, filter->R);
}

void XYZ_Kalman_Calculate(XYZ_Kalman_Filter_t *filter)
{
    float32_t A_squared = filter->A * filter->A;
    float32_t temp_innovation[3];

    // 1. 状态预测
    arm_scale_f32(filter->Out, filter->A, filter->Out, 3);

    // 2. 协方差预测与增益计算
    for(int i = 0; i < 3; i++) {
        filter->_P[i] = A_squared * filter->P[i] + filter->Q[i];
    }
    // 3. 批量计算增益和更新输出
    for(int i = 0; i < 3; i++) {
        float32_t denominator = filter->H * filter->H * filter->_P[i] + filter->R[i];
        filter->G[i] = (filter->H * filter->_P[i]) / denominator;
    }
    // 4. 批量更新输出和协方差
    
    arm_scale_f32(filter->Out, filter->H, temp_innovation, 3); // H * Out
    arm_sub_f32(filter->Now, temp_innovation, temp_innovation, 3); // Now - H * Out

    for (int i = 0; i < 3; i++) {
        filter->Out[i] = filter->Out[i] + filter->G[i] * temp_innovation[i]; // Out + G * (Now - H * Out)
        filter->P[i] = (1.0f - filter->G[i] * filter->H) * filter->_P[i]; // P = (1 - G * H) * P_predict
    }
}