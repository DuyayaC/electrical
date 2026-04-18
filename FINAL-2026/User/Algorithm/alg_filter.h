#ifndef ALG_FILTER_H
#define ALG_FILTER_H

#include "arm_math.h"

/*IIR 滤波器*/
typedef struct
{
    float alpha;      // 滤波系数
    float last_data;  // 上一次的滤波结果
} IIR_Filter_t;

void IIR_Init(IIR_Filter_t *filter, float alpha);
void IIR_Calculate(IIR_Filter_t *filter, float *data);
void IIR_Clear(IIR_Filter_t *filter);

/*单维卡尔曼滤波器*/
typedef struct
{
    float A;  // 状态转移矩阵
    float B;  // 控制输入矩阵
    float H;  // 观测矩阵
    float Q;  // 过程噪声协方差
    float R;  // 测量噪声协方差
    float P;  // 估计误差协方差
    float P_predict; // 临时变量，用于计算
    float G;   // 卡尔曼增益
    float Out; // 滤波输出
    float Now; // 当前测量值
} Kalman_Filter_t;

void Kalman_Init(Kalman_Filter_t *filter, float a, float b, float h, float q, float r);
void Kalman_Calculate(Kalman_Filter_t *filter);

/*XYZ三轴卡尔曼滤波器*/ 
typedef struct
{
    float A;  // 状态转移矩阵
    float B;  // 控制输入矩阵
    float H;  // 观测矩阵
    
    float P[3];  // 估计误差协方差
    float _P[3];  // 临时变量，用于计算
    float Q[3];  // 过程噪声协方差
    float R[3];  // 测量噪声协方差
    float G[3];  // 卡尔曼增益

    arm_matrix_instance_f32 mat_P; // 估计误差协方差矩阵
    arm_matrix_instance_f32 mat_Q; // 过程噪声协方差矩阵
    arm_matrix_instance_f32 mat_R; // 测量噪声协方差矩阵

    float Out[3]; // 滤波输出
    float Now[3]; // 当前测量值
} __attribute__((aligned(32))) XYZ_Kalman_Filter_t;

extern XYZ_Kalman_Filter_t GYRO;
extern XYZ_Kalman_Filter_t ACCEL;

void XYZ_Kalman_Init(XYZ_Kalman_Filter_t *filter, float a, float b, float h);
void XYZ_Kalman_Calculate(XYZ_Kalman_Filter_t *filter);

#endif
