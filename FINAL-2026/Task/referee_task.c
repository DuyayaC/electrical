#include "referee_task.h"

extern osThreadId referee_taskHandle;
extern UART_HandleTypeDef huart6;
extern osThreadId calibrate_taskHandle;

static volatile uint16_t referee_dma_write_pos = 0;

void REFEREE_TASK(void const * argument)
{
    uint16_t last_pos = 0;
    uint16_t curr_pos = 0;

    Referee_Init(&referee, &huart6, 0xA5);

    HAL_UART_Receive_DMA(&huart6, referee.Rx_Buffer, referee.Rx_Buffer_Length);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    while(1)
    {
        while(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }

        curr_pos = referee_dma_write_pos;

        if (curr_pos != last_pos)
        {
            if (curr_pos > last_pos)
            {
                Referee_Unpack(&referee, &referee.Rx_Buffer[last_pos], curr_pos - last_pos);
            }
            else
            {
                Referee_Unpack(&referee, &referee.Rx_Buffer[last_pos], referee.Rx_Buffer_Length - last_pos);

                if (curr_pos > 0)
                {
                    Referee_Unpack(&referee, referee.Rx_Buffer, curr_pos);
                }
            }

            last_pos = curr_pos;
//						if (referee.Game_Status.Stage_Enum != 4)
//						{
//							calibrate.flag = 1;
//							BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//							vTaskNotifyGiveFromISR(calibrate_taskHandle, &xHigherPriorityTaskWoken);
//							portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//						}
        }
    }
}

void USART6_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);

        if (huart6.hdmarx != NULL)
        {
            uint16_t dma_counter = (uint16_t)__HAL_DMA_GET_COUNTER(huart6.hdmarx);
            referee_dma_write_pos = (uint16_t)(referee.Rx_Buffer_Length - dma_counter);
            if (referee_dma_write_pos >= referee.Rx_Buffer_Length)
            {
                referee_dma_write_pos = 0;
            }
        }

        if (referee_taskHandle != NULL)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(referee_taskHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    HAL_UART_IRQHandler(&huart6);
}
