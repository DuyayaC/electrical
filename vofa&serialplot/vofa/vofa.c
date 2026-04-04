#include "vofa.h"

static vofa_send_t vofa_send = {.tail = {0x00, 0x00, 0x80, 0x7F}};

void vofa_send_data(uint8_t num, ...)
{
    if (num > MAX_CHANNELS) num = MAX_CHANNELS;

    va_list args;
    va_start(args, num);

    for (uint8_t i = 0; i < num; i++) 
    {
        vofa_send.data[i] = (float)va_arg(args, double);
    }

    va_end(args);

    memcpy(((uint8_t*)vofa_send.data) + (num * sizeof(float)), vofa_send.tail, 4);
    uint16_t send_len = (num * sizeof(float)) + 4;

    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&vofa_send, send_len);

}
