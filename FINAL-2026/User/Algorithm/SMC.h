#ifndef SLIDING_H
#define SLIDING_H

#include <math.h>
#include <stdlib.h>
#include "string.h"

// #define SAMPLE_PERIOD 0.002f
#define SAMPLE_PERIOD 0.001f

typedef enum {
    EXPONENT = 0,
    EISMC = 1
} Rmode;

typedef struct {
    float tar_now;
    float tar_last;
    float tar_differential;
    float tar_differential_last;
    float tar_differential_second;

    float p_error;
    float v_error;
    float p_error_integral;     // EISMC 模式使用的积分项
    float pos_error_eps;
} RError;

typedef struct {
    float J;
    float K;
    float c;                    // EXPONENT 模式参数
    float c1;                   // EISMC 模式参数 1
    float c2;                   // EISMC 模式参数 2
    float epsilon;
} SlidingParam;

typedef struct {
    float u;
    float s;
    SlidingParam param;
    RError error;
    float u_max;
    float limit;
    Rmode flag;
} Sliding;

extern Sliding yaw;
extern Sliding pitch;

// 函数声明
void SMC_Init(Sliding *smc);
void SMC_SetParam_Exponent(Sliding *smc, float J, float K, float c, float epsilon, float limit, float u_max, float pos_esp);
void SMC_SetParam_EISMC(Sliding *smc, float J, float K, float c1, float c2, float epsilon, float limit, float u_max, float pos_esp);
void SMC_ErrorUpdate(Sliding *smc, float target, float pos_now, float vol_now);
float SMC_Calculate(Sliding *smc);
void SMC_Clear(Sliding *smc);
void SMC_IntegralClear(Sliding *smc);

#endif