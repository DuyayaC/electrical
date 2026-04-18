#include "drv_math.h"

/**
 * @brief 求和 (8位)
 */
uint8_t Math_Sum_8(uint8_t *Address, uint32_t Length)
{
    uint8_t sum = 0;
    for (uint32_t i = 0; i < Length; i++) {
        sum += Address[i];
    }
    return sum;
}

/**
 * @brief 求和 (16位)
 */
uint16_t Math_Sum_16(uint16_t *Address, uint32_t Length)
{
    uint16_t sum = 0;
    for (uint32_t i = 0; i < Length; i++) {
        sum += Address[i];
    }
    return sum;
}

/**
 * @brief 求和 (32位)
 */
uint32_t Math_Sum_32(uint32_t *Address, uint32_t Length)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < Length; i++) {
        sum += Address[i];
    }
    return sum;
}

/**
 * @brief sinc函数的实现
 */
float Math_Sinc(float x)
{
    if (MATH_ABS(x) <= 2.0f * FLT_EPSILON) {
        return 1.0f;
    }
    return (sinf(x) / x);
}

/**
 * @brief 求取模归化 (例如角度归化到 -180 ~ 180)
 */
float Math_Modulus_Normalization(float x, float modulus)
{
    float tmp = fmodf(x + modulus / 2.0f, modulus);
    if (tmp < 0.0f) {
        tmp += modulus;
    }
    return (tmp - modulus / 2.0f);
}