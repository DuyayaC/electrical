#ifndef FUNCTION_H
#define FUNCTION_H

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
#endif