#ifndef ALG_FILTER_H
#define ALG_FILTER_H

#include <stdint.h>

/*IIR 滤波器（一阶低通）*/
typedef struct
{
    float alpha;      // 滤波系数（0~1），越大响应越快、平滑越弱
    float last_data;  // 上一次的滤波结果
    uint8_t is_valid; // 数据有效标志
} IIR_Filter_t;

void IIR_Init(IIR_Filter_t *filter, float alpha);
void IIR_SetAlpha(IIR_Filter_t *filter, float alpha);
void IIR_Calculate(IIR_Filter_t *filter, float *data);
void IIR_Clear(IIR_Filter_t *filter);

/*FIR 滤波器（滑动平均），缓冲区由调用方提供静态数组，不使用动态内存*/
typedef struct
{
    float *buffer;        // 数据缓冲区（调用方提供，静态数组）
    uint8_t buffer_len;   // 缓冲区长度
    uint8_t buffer_index; // 当前写入索引
    float sum;            // 窗口内数据和
    uint8_t is_valid;     // 数据有效标志
} FIR_Filter_t;

void FIR_Init(FIR_Filter_t *filter, float *buffer, uint8_t buffer_len);
void FIR_Calculate(FIR_Filter_t *filter, float *data);
void FIR_Clear(FIR_Filter_t *filter);

#endif
