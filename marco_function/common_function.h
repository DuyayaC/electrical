#ifndef FUNCTION_H
#define FUNCTION_H

#include <stdint.h>
#include <math.h>

//最大值函数
#define MAX(a,b) ((a) > (b) ? (a) : (b))

//最小值函数
#define MIN(a,b) ((a) < (b) ? (a) : (b))

//绝对值函数
#define ABS(x) ((x) < 0 ? -(x) : (x))

//角度转弧度函数
#define DEG_TO_RAD(deg)   ((deg) * 3.14159265359f / 180.0f)

//弧度转角度函数
#define RAD_TO_DEG(rad)   ((rad) * 180.0f / 3.14159265359f)

//限幅函数
#define CLAMP(x, min, max)  ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

//平方函数
#define SQUARE(x) ((x) * (x))

// 将浮点数放大并四舍五入，转换为 16位 有符号整数 (保留3位小数)
#define FLOAT_TO_INT16_4DEC(val)  ((int16_t)((val) * 1000.0f + ((val) >= 0 ? 0.5f : -0.5f)))

// 将 16位 有符号整数缩小，还原为浮点数
#define INT16_TO_FLOAT_4DEC(raw)  ((float)(raw) / 1000.0f)

// 提取 16位 整数的高八位 (MSB)
#define GET_INT16_MSB(raw)        ((uint8_t)(((int16_t)(raw) >> 8) & 0xFF))

// 提取 16位 整数的低八位 (LSB)
#define GET_INT16_LSB(raw)        ((uint8_t)((int16_t)(raw) & 0xFF))

// 将高低八位组合回 16位 整数
#define COMBINE_BYTES_TO_INT16(msb, lsb) ((int16_t)(((uint8_t)(msb) << 8) | (uint8_t)(lsb)))

// 1. 角度转弧度，结果直接四舍五入保留 3 位小数 (返回 float)
#define DEG_TO_RAD_4DEC(deg)  (roundf(((deg) * 3.14159265359f / 180.0f) * 1000.0f) / 1000.0f)

// 2. 弧度转角度，结果直接四舍五入保留 3 位小数 (返回 float)
#define RAD_TO_DEG_4DEC(rad)  (roundf(((rad) * 180.0f / 3.14159265359f) * 1000.0f) / 1000.0f)

#endif