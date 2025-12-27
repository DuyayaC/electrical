/**
 * @file drv_math.h
 * @author yssickjgd 1345578933@qq.com
 * @brief 一些数学
 * @version 0.1
 * @date 2023-05-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef DRV_MATH_H
#define DRV_MATH_H

/* Includes ------------------------------------------------------------------*/

#include "stdint.h"
#include "limits.h"
#include "math.h"
#include "stdint.h"
#include "float.h"

/* Exported macros -----------------------------------------------------------*/

//圆周率PI
#define PI (3.14159265f)

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

void Math_Endian_Reverse_16(void *Address);
void Math_Endian_Reverse_16(void *Source, void *Destination);
void Math_Endian_Reverse_32(void *Address);
void Math_Endian_Reverse_32(void *Source, void *Destination);

uint8_t Math_Sum_8(uint8_t *Address, uint32_t Length);
uint16_t Math_Sum_16(uint16_t *Address, uint32_t Length);
uint32_t Math_Sum_32(uint32_t *Address, uint32_t Length);

float Math_Sinc(float x);

/**
 * @brief 限幅函数
 *
 * @tparam Type
 * @param x 传入数据
 * @param Min 最小值
 * @param Max 最大值
 */
template <typename Type>
void Math_Constrain(Type *x, Type Min, Type Max)
{
    if (*x < Min)
    {
        *x = Min;
    }
    else if (*x > Max)
    {
        *x = Max;
    }
}

/**
 * @brief 求绝对值
 *
 * @tparam Type
 * @param x 传入数据
 * @return Type x的绝对值
 */
template <typename Type>
Type Math_Abs(Type x)
{
    return ((x > 0) ? x : -x);
}

/**
 * @brief 求取模归化
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param modulus 模数
 * @return Type 返回的归化数, 介于 ±modulus / 2 之间
 */
template<typename Type>
Type Math_Modulus_Normalization(Type x, Type modulus)
{
    float tmp;

    tmp = fmod(x + modulus / 2.0f, modulus);

    if (tmp < 0.0f)
    {
        tmp += modulus;
    }

    return (tmp - modulus / 2.0f);
}


#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
