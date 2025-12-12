/**
 * @file alg_filter.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 滤波器
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-09-25 1.1 可自定义滤波器阶数
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_filter.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化Kalman滤波器
 * 
 * @param __Error_Measure 测量误差
 * @param __Value 当前值
 * @param __Error_Estimate 估计误差
 */
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
}

void Class_Filter_XYZ_Kalman::TIM_Calculate()
{
    for (int i = 0; i < 3; i++)
    {
        //predict update
        Out[i] = A * Out[i];
        _P[i] = A * P[i] * A +Q[i];

        //actual update
        G[i] = _P[i] * H / (H * _P[i] * H + R[i]);
        Out[i] = Out[i] + G[i] * (Now[i] - H * Out[i]);
        P[i] = (1.0f - G[i] * H) * _P[i];
    }

}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
