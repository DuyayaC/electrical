#include "alg_filter.h"

/*IIR 滤波器（一阶低通）*/
void IIR_Init(IIR_Filter_t *filter, float alpha)
{
    if (filter == NULL)
    {
        return;
    }

    filter->alpha = alpha;
    filter->last_data = 0.0f;
    filter->is_valid = 0;
}

void IIR_SetAlpha(IIR_Filter_t *filter, float alpha)
{
    if (filter == NULL)
    {
        return;
    }

    filter->alpha = alpha;
}

void IIR_Calculate(IIR_Filter_t *filter, float *data)
{
    float output;

    if (filter == NULL || data == NULL)
    {
        return;
    }

    /*首次输入直接作为初值，避免开机从 0 跳变的毛刺*/
    if (filter->is_valid == 0)
    {
        filter->last_data = *data;
        filter->is_valid = 1;
        return;
    }

    output = filter->alpha * (*data) + (1.0f - filter->alpha) * filter->last_data;
    filter->last_data = output;
    *data = output;
}

void IIR_Clear(IIR_Filter_t *filter)
{
    if (filter == NULL)
    {
        return;
    }

    filter->last_data = 0.0f;
    filter->is_valid = 0;
}

/*FIR 滤波器（滑动平均）*/
void FIR_Init(FIR_Filter_t *filter, float *buffer, uint8_t buffer_len)
{
    if (filter == NULL || buffer == NULL || buffer_len == 0)
    {
        return;
    }

    filter->buffer = buffer;
    filter->buffer_len = buffer_len;
    filter->buffer_index = 0;
    filter->sum = 0.0f;
    filter->is_valid = 1;

    for (uint8_t i = 0; i < filter->buffer_len; i++)
    {
        filter->buffer[i] = 0.0f;
    }
}

void FIR_Calculate(FIR_Filter_t *filter, float *data)
{
    if (filter == NULL || data == NULL || filter->is_valid == 0)
    {
        return;
    }

    filter->sum -= filter->buffer[filter->buffer_index];
    filter->buffer[filter->buffer_index] = *data;
    filter->sum += *data;

    filter->buffer_index++;
    if (filter->buffer_index >= filter->buffer_len)
    {
        filter->buffer_index = 0;
    }

    *data = filter->sum / (float)filter->buffer_len;
}

void FIR_Clear(FIR_Filter_t *filter)
{
    if (filter == NULL || filter->is_valid == 0)
    {
        return;
    }

    for (uint8_t i = 0; i < filter->buffer_len; i++)
    {
        filter->buffer[i] = 0.0f;
    }

    filter->buffer_index = 0;
    filter->sum = 0.0f;
}