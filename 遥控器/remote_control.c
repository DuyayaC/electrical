/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      SBUS remote control DMA + IDLE handling
  */

#include "remote_control.h"
#include "main.h"
#include "usart.h"

/* Debug: count RC IDLE events to help verify ISR firing */
volatile uint32_t rc_idle_count = 0;

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

// remote control data
RC_ctrl_t rc_ctrl;

// receive buffers (double buffer)
uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief remote control init
  */
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
    /* RC_init() 已完成 USART3 DMA 双缓冲 + IDLE 中断配置并启动 DMA。
       这里不要再调用 HAL_UART_Receive_DMA()，否则会覆盖双缓冲配置，导致无法正确解包。 */
}

const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;

}

// static inline int16_t rc_switch_to_channel_value(char s)
// {
//     /* DJI 三段拨杆：UP=1, MID=3, DOWN=2；映射到 11bit 通道典型值 */
//     if (s == (char)RC_SW_UP) return (int16_t)RC_CH_VALUE_MAX;
//     if (s == (char)RC_SW_DOWN) return (int16_t)RC_CH_VALUE_MIN;
//     return (int16_t)RC_CH_VALUE_OFFSET;
// }

static void dbus_to_rc(volatile const uint8_t *dbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (dbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    /* 先给所有通道一个安全默认值（防止主控用到未解出的通道时一直为 0） */
    for (uint8_t i = 0; i < 10; i++)
    {
        rc_ctrl->rc.ch[i] = (int16_t)RC_CH_VALUE_OFFSET;
    }
		/* 拨杆从左往右分别为 1 2 3 4 */
	    rc_ctrl->rc.ch[0] = (((dbus_buf[1] | dbus_buf[2] << 8) & 0x07FF) - Rocker_Mid); // 右侧摇杆左右
		rc_ctrl->rc.ch[1] = (((dbus_buf[2] >> 3 | dbus_buf[3] << 5) & 0x07FF) - Rocker_Mid); // 右侧摇杆上下
		rc_ctrl->rc.ch[2] = (((dbus_buf[3] >> 6 | dbus_buf[4] << 2 | dbus_buf[5] << 10) & 0x07FF) - Rocker_Mid); // 左侧摇杆上下
		rc_ctrl->rc.ch[3] = (((dbus_buf[5] >> 1 | dbus_buf[6] << 7) & 0x07FF) - Rocker_Mid); // 左侧摇杆左右
		rc_ctrl->rc.ch[4] = (((dbus_buf[6] >> 4 | dbus_buf[7] << 4) & 0x07FF) - Rocker_Mid); // 左侧旋钮
		rc_ctrl->rc.ch[5] = (((dbus_buf[7] >> 7 | dbus_buf[8] << 1 | dbus_buf[9] << 9) & 0x07FF) - Rocker_Mid); // 右侧旋钮
		rc_ctrl->rc.ch[6] = (((dbus_buf[9] >> 2 | dbus_buf[10] << 6) & 0x07FF) - Rocker_Mid) / Rocker_Delt_Range; // 一拨杆
		rc_ctrl->rc.ch[7] = (((dbus_buf[10] >> 5 | dbus_buf[11] << 3) & 0x07FF) - Rocker_Mid) / Rocker_Delt_Range; // 二拨杆
		rc_ctrl->rc.ch[8] = (((dbus_buf[12] | dbus_buf[13] << 8) & 0x07FF) - Rocker_Mid) / Rocker_Delt_Range;  // 三拨杆
		rc_ctrl->rc.ch[9] = (((dbus_buf[13] >> 3 | dbus_buf[14] << 5) & 0x07FF) - Rocker_Mid) / Rocker_Delt_Range; // 四拨杆
}

 static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
    /* 先给所有通道一个安全默认值（防止主控用到未解出的通道时一直为 0） */
    for (uint8_t i = 0; i < 10; i++)
    {
        rc_ctrl->rc.ch[i] = (int16_t)RC_CH_VALUE_OFFSET;
    }
		/* 拨杆从左往右分别为 1 2 3 4 */
	    rc_ctrl->rc.ch[0] = (((sbus_buf[1] | sbus_buf[2] << 8) & 0x07FF) - Rocker_Mid); /// 右侧摇杆左右
		rc_ctrl->rc.ch[1] = (((sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07FF) - Rocker_Mid); // 右侧摇杆上下
		rc_ctrl->rc.ch[2] = (((sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) & 0x07FF) - Rocker_Mid); // 左侧摇杆上下
		rc_ctrl->rc.ch[3] = (((sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07FF) - Rocker_Mid); // 左侧摇杆左右
		rc_ctrl->rc.ch[4] = (((sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07FF) - Rocker_Mid); // 左侧旋钮
		rc_ctrl->rc.ch[5] = (((sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 9) & 0x07FF) - Rocker_Mid); // 右侧旋钮
		rc_ctrl->rc.ch[6] = (((sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07FF) - Rocker_Mid) / Rocker_Delt_Range; // 一拨杆
		rc_ctrl->rc.ch[7] = (((sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07FF) - Rocker_Mid) / Rocker_Delt_Range; // 二拨杆
		rc_ctrl->rc.ch[8] = (((sbus_buf[12] | sbus_buf[13] << 8) & 0x07FF) - Rocker_Mid) / Rocker_Delt_Range;  // 三拨杆
		rc_ctrl->rc.ch[9] = (((sbus_buf[13] >> 3 | sbus_buf[14] << 5) & 0x07FF) - Rocker_Mid) / Rocker_Delt_Range; // 四拨杆
}

void RC_USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        /* increment debug counter on IDLE */
        rc_idle_count++;
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH_SBUS)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
            else if(this_time_rx_len == RC_FRAME_LENGTH_DBUS)
            {
                dbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH_SBUS)
            {
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
            else if(this_time_rx_len == RC_FRAME_LENGTH_DBUS)
            {
                dbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}
