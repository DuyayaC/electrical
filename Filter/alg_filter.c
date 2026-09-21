/**
  * @file          alg_filter.c
  * @brief         数字滤波器：一阶低通 IIR + 滑动平均 FIR
  *
  * 自包含模块：仅依赖 <stdint.h>，不含 HAL / 硬件相关代码。
  * IIR 用系数 alpha 做指数加权平均，计算量最小，适合陀螺/速度这类高频噪声；
  * FIR 用调用方提供的静态环形缓冲做滑动平均，相位滞后比 IIR 明显。
 */

#include "alg_filter.h"

/* ==================== 使用说明 ====================
 * 两个滤波器都是"就地修改"接口：传入待滤波数据的地址，
 * 结果直接写回同一个变量，不需要额外的输出变量。
 *
 * 完整流程：
 *   static IIR_Filter_t iir;              // 1) 定义滤波器对象
 *   static float        fir_buf[8];       //    FIR 的缓冲区（调用方提供静态数组）
 *   static FIR_Filter_t fir;
 *
 *   IIR_Init(&iir, 0.3f);                 // 2) 初始化（必须先调用）
 *   FIR_Init(&fir, fir_buf, 8);
 *
 *   IIR_Calculate(&iir, &speed);          // 3) 每个控制周期调用一次
 *   FIR_Calculate(&fir, &current);
 *
 *   IIR_Clear(&iir);                      // 4) 需要重新起步时清掉历史数据
 *   FIR_Clear(&fir);
 *
 * 注意：
 *   1. 未 Init 就调用 Calculate 不会滤波（IIR 首次输入直接作为初值，FIR 直接返回）；
 *   2. alpha 越大越"跟手"但平滑越弱，0.1~0.4 是常用区间；
 *   3. FIR 的 buffer 必须是 static 数组，函数内部不做动态内存分配。
 * ================================================ */

/* ==================== 1. IIR 滤波器（一阶低通） ==================== */

/**
  * @brief          初始化 IIR 滤波器（清零历史数据，并标记为无效）
  * @param[in,out]  filter: IIR 滤波器结构体指针
  * @param[in]      alpha:  滤波系数（0~1），越大响应越快、平滑越弱
  * @retval         none
  */
void IIR_Init(IIR_Filter_t *filter, float alpha)
{
    filter->alpha = alpha;
    filter->last_data = 0.0f;
    filter->is_valid = 0;
}

/**
  * @brief          运行时修改滤波系数（不改变历史数据）
  * @param[in,out]  filter: IIR 滤波器结构体指针
  * @param[in]      alpha:  新的滤波系数（0~1）
  * @retval         none
  */
void IIR_SetAlpha(IIR_Filter_t *filter, float alpha)
{
    filter->alpha = alpha;
}

/**
  * @brief          执行一次 IIR 滤波（结果就地写回 *data）
  * @param[in,out]  filter: IIR 滤波器结构体指针
  * @param[in,out]  data:   待滤波数据指针；返回时被替换为滤波结果
  * @retval         none
  */
void IIR_Calculate(IIR_Filter_t *filter, float *data)
{
    float output;

    /*首次输入直接作为初值，避免开机从 0 跳变的毛刺*/
    if (filter->is_valid == 0)
    {
        filter->last_data = *data;
        filter->is_valid = 1;
        return;
    }

    /*一阶低通：y[n] = alpha * x[n] + (1 - alpha) * y[n-1]*/
    output = filter->alpha * (*data) + (1.0f - filter->alpha) * filter->last_data;
    filter->last_data = output;
    *data = output;
}

/**
  * @brief          清空 IIR 历史数据（下次调用会重新以第一帧作为初值）
  * @param[in,out]  filter: IIR 滤波器结构体指针
  * @retval         none
  */
void IIR_Clear(IIR_Filter_t *filter)
{
    filter->last_data = 0.0f;
    filter->is_valid = 0;
}

/* ==================== 2. FIR 滤波器（滑动平均） ==================== */

/**
  * @brief          初始化 FIR 滤波器（绑定缓冲区并清零）
  * @param[in,out]  filter:     FIR 滤波器结构体指针
  * @param[in]      buffer:     数据缓冲区，由调用方提供 static 数组
  * @param[in]      buffer_len: 缓冲区长度（窗口大小）；为 0 时直接返回不初始化
  * @retval         none
  */
void FIR_Init(FIR_Filter_t *filter, float *buffer, uint8_t buffer_len)
{
    if (buffer_len == 0)
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

/**
  * @brief          执行一次 FIR 滤波（结果就地写回 *data）
  * @param[in,out]  filter: FIR 滤波器结构体指针
  * @param[in,out]  data:   待滤波数据指针；返回时被替换为窗口平均值
  * @retval         none
  */
void FIR_Calculate(FIR_Filter_t *filter, float *data)
{
    if (filter->is_valid == 0)
    {
        return;
    }

    /*滑动窗口：减去最旧的一个样本、加上最新的一个样本，求和保持 O(1)*/
    filter->sum -= filter->buffer[filter->buffer_index];
    filter->buffer[filter->buffer_index] = *data;
    filter->sum += *data;

    /*环形缓冲：写满一圈后回到 0 号位置覆盖最旧数据*/
    filter->buffer_index++;
    if (filter->buffer_index >= filter->buffer_len)
    {
        filter->buffer_index = 0;
    }

    *data = filter->sum / (float)filter->buffer_len;
}

/**
  * @brief          清空 FIR 缓冲区与累加和（下次从空窗口重新开始）
  * @param[in,out]  filter: FIR 滤波器结构体指针
  * @retval         none
  */
void FIR_Clear(FIR_Filter_t *filter)
{
    if (filter->is_valid == 0)
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
