#ifndef DRV_MATH_H
#define DRV_MATH_H

#include "stdint.h"
#include "limits.h"
#include "math.h"
#include "float.h"

#ifndef PI
#define PI (3.14159265f)
#endif

// 求和函数
uint8_t Math_Sum_8(uint8_t *Address, uint32_t Length);
uint16_t Math_Sum_16(uint16_t *Address, uint32_t Length);
uint32_t Math_Sum_32(uint32_t *Address, uint32_t Length);

// 特殊数学函数
float Math_Sinc(float x);
float Math_Modulus_Normalization(float x, float modulus);

/**
 * @brief 限幅函数 (float版本)
 */
static inline void Math_Constrain_Float(float *x, float Min, float Max) {
    if (*x < Min) *x = Min;
    else if (*x > Max) *x = Max;
}

/**
 * @brief 限幅函数 (int32版本)
 */
static inline void Math_Constrain_Int(int32_t *x, int32_t Min, int32_t Max) {
    if (*x < Min) *x = Min;
    else if (*x > Max) *x = Max;
}

/**
 * @brief 求绝对值宏 (通用)
 */
#define MATH_ABS(x) ((x) > 0 ? (x) : -(x))

#endif