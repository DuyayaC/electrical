#include "main.h"
#include "arm_math.h"
#include <math.h>

#ifndef Filter_ON
#define Filter_ON
#define ALIGN_32_BYTES __attribute__((aligned(32)))


class Class_IIR
{
public:
    void Init(float __alpha);

    void In_Calculate_Out(float *data);

protected:
    float alpha;
    float last_data;
    
}

//一维卡尔曼
class Class_Filter_Kalman
{
public:
    void Init(float a, float b, float h, float q, float r);

    float Get_Out();

    void Set_Now(float __Now);

    void TIM_Calculate();

protected:
    // 初�?�化相关常量
    float A, B, H, Q, R;
    /*
    *A: state transfer scale
    *B: control scale
    *H: observer scale
    *P: estimate covariance
    *Q: process covariance
    *R: observe covariance
    */
		//float z = 0.0f;
    //预测输出

    // 增益
    float G = 0.0f;

    //初�?�估计协方差
    float P = 0.0f;
	float _P = 10.0f;

    // 输出�?
    float Out = 0.0f;

    // 当前�?
    float Now = 0.0f;

};

inline float Class_Filter_Kalman::Get_Out()
{
    return (Out);
}


inline void Class_Filter_Kalman::Set_Now(float __Now)
{
    Now = __Now;
}

//一维三轴卡尔曼
class Class_Filter_XYZ_Kalman
{
public:
    void Init();

    void Get_Out(float *__Out);

    void Set_Now(float *__Now);

    void TIM_Calculate();

protected:
    //初�?�化相关变量
    float A, B, H;
    
    float ALIGN_32_BYTES P[3] = {0.0f, 0.0f, 0.0f};
    float ALIGN_32_BYTES _P[3] = {0.0f, 0.0f, 0.0f};
		
	float ALIGN_32_BYTES Q[3] = {0.1f, 0.1f, 0.1f};
	float ALIGN_32_BYTES R[3] = {1.0f, 1.0f, 1.0f}; // x y z
    
    //三轴增益
    float ALIGN_32_BYTES G[3] = {0.0f, 0.0f, 0.0f};

    //输出
    float ALIGN_32_BYTES Out[3] = {0.0f, 0.0f, 0.0f};

    //输入
    float ALIGN_32_BYTES Now[3] = {0.0f, 0.0f, 0.0f};

    arm_matrix_instance_f32 mat_P, mat_Q, mat_R;
};

inline void Class_Filter_XYZ_Kalman::Get_Out(float *__Out)
{
    arm_copy_f32(Out, __Out, 3)
}

inline void Class_Filter_XYZ_Kalman::Set_Now(float *__Now)
{
    arm_copy_f32(__Now, Now, 3);

}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
