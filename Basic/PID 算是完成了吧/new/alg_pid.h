/**
 * @file alg_pid.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief PID算法
 * @version 0.1
 * @date 2022-05-03
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef ALG_PID_H
#define ALG_PID_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief �?分先�?
 * 注意：�?�枚举已不再使用，为保持向后兼�?�性而保�?
 */
enum Enum_PID_D_First
{
    PID_D_First_DISABLE = 0,
    PID_D_First_ENABLE,
};

/**
 * @brief Reusable, PID算法
 *
 */
class Class_PID
{
public:
    void Init(float __K_P, float __K_I, float __K_D, float __K_F = 0.0f, float __I_Out_Max = 0.0f, float __Out_Max = 0.0f, float __D_T = 0.001f, float __Dead_Zone = 0.0f);

    // 设置�?标值和当前�?
    void Set_Values(float __Target, float __Now);

    // 清除�?�?和状�?
    void Clear_Error();

    // PID计算函数
    void Calculate(float *output);

protected:
    //初�?�化相关常量

    // PID计时器周�?, s
    float D_T;
    //死区, Error在其绝�?�值内不输�?
    float Dead_Zone;

    //内部变量

    //之前的当前�?
    float Pre_Now = 0.0f;
    //之前的目标�?
    float Pre_Target = 0.0f;
    //之前的输出�?
    float Pre_Out = 0.0f;
    //前向�?�?
    float Pre_Error = 0.0f;

    // PID参数
    // PID的P
    float K_P = 0.0f;
    // PID的I
    float K_I = 0.0f;
    // PID的D
    float K_D = 0.0f;
    //前�??
    float K_F = 0.0f;

    //�?分限�?, 0为不限制
    float I_Out_Max = 0;
    //输出限幅, 0为不限制
    float Out_Max = 0;

    //�?标�?
    float Target = 0.0f;
    //当前�?
    float Now = 0.0f;

    //�?分�?
    float Integral_Error = 0.0f;

    //内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/