#include "control_task.h"

VTM_Data_t key;
extern osThreadId calibrate_taskHandle;
extern UART_HandleTypeDef huart1;
extern osThreadId control_taskHandle;

static uint16_t crc16_init = 0xffff;
static const uint16_t crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
uint8_t VTM_Rx_Buf[RX_BUF_SIZE];

static bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len);
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16);
static void VTM_Data_Parsing(uint8_t *p_frame, VTM_Data_t *out_data);

void CONTROL_TASK(void const * argument)
{
    uint8_t frame_buf[RX_BUF_SIZE];
    VTM_Data_t parsed_key;

    VTM_UART_Init();

    while(1)
    {
        while(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }

        taskENTER_CRITICAL();
        memcpy(frame_buf, VTM_Rx_Buf, RX_BUF_SIZE);
        taskEXIT_CRITICAL();

        if (frame_buf[0] == 0xA9 && frame_buf[1] == 0x53 && verify_crc16_check_sum(frame_buf, VTM_FRAME_SIZE))
        {
            VTM_Data_Parsing(frame_buf, &parsed_key);

            taskENTER_CRITICAL();
            key = parsed_key;
            taskEXIT_CRITICAL();

            if (key.key_b == 1)
            {
                calibrate_signal = 1;
                if(calibrate_taskHandle != NULL)
                    xTaskNotifyGive(calibrate_taskHandle);
            }
            else if (key.key_b == 0)
            {
                calibrate_signal = 0;
            }
        }

    }
}



void VTM_UART_Init(void) 
{
    HAL_UART_Receive_DMA(&huart1, VTM_Rx_Buf, RX_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

static void VTM_Data_Parsing(uint8_t *p_frame, VTM_Data_t *out_data) 
{
    if (out_data == NULL)
    {
        return;
    }

    // 1. 基础校验：帧头校验与CRC校验
    if (p_frame[0] != 0xA9 || p_frame[1] != 0x53) return;
    if (!verify_crc16_check_sum(p_frame, VTM_FRAME_SIZE)) return;

    // 2. 摇杆与通道解析 (非字节对齐位移)
    out_data->ch0 = ((p_frame[2] >> 0) | (p_frame[3] << 8)) & 0x07FF;
    out_data->ch1 = ((p_frame[3] >> 3) | (p_frame[4] << 5)) & 0x07FF;
    out_data->ch2 = ((p_frame[4] >> 6) | (p_frame[5] << 2) | (p_frame[6] << 10)) & 0x07FF;
    out_data->ch3 = ((p_frame[6] >> 1) | (p_frame[7] << 7)) & 0x07FF;
    
    out_data->sw     = (p_frame[7] >> 4) & 0x03;
    out_data->pause  = (p_frame[7] >> 6) & 0x01;
    out_data->btn_l  = (p_frame[7] >> 7) & 0x01;
    out_data->btn_r  = (p_frame[8] >> 0) & 0x01;
    out_data->dial   = ((p_frame[8] >> 1) | (p_frame[9] << 7)) & 0x07FF;
    out_data->trigger = (p_frame[9] >> 4) & 0x01;

    // 3. 鼠标解析 (16位有符号)
    out_data->mouse_x = (int16_t)(p_frame[10] | (p_frame[11] << 8));
    out_data->mouse_y = (int16_t)(p_frame[12] | (p_frame[13] << 8));
    out_data->mouse_z = (int16_t)(p_frame[14] | (p_frame[15] << 8));
    
    out_data->mouse_btn_l = p_frame[16] & 0x03;
    out_data->mouse_btn_r = (p_frame[16] >> 2) & 0x03;
    out_data->mouse_btn_m = (p_frame[16] >> 4) & 0x03;

    // 4. 键盘所有按键读取 (提取位图中的每一位)
    uint16_t key_v = (uint16_t)(p_frame[17] | (p_frame[18] << 8));
    
    out_data->key_w     = (key_v & KEY_W_BIT)     ? 1 : 0;
    out_data->key_s     = (key_v & KEY_S_BIT)     ? 1 : 0;
    out_data->key_a     = (key_v & KEY_A_BIT)     ? 1 : 0;
    out_data->key_d     = (key_v & KEY_D_BIT)     ? 1 : 0;
    out_data->key_shift = (key_v & KEY_SHIFT_BIT) ? 1 : 0;
    out_data->key_ctrl  = (key_v & KEY_CTRL_BIT)  ? 1 : 0;
    out_data->key_q     = (key_v & KEY_Q_BIT)     ? 1 : 0;
    out_data->key_e     = (key_v & KEY_E_BIT)     ? 1 : 0;
    out_data->key_r     = (key_v & KEY_R_BIT)     ? 1 : 0;
    out_data->key_f     = (key_v & KEY_F_BIT)     ? 1 : 0;
    out_data->key_g     = (key_v & KEY_G_BIT)     ? 1 : 0;
    out_data->key_z     = (key_v & KEY_Z_BIT)     ? 1 : 0;
    out_data->key_x     = (key_v & KEY_X_BIT)     ? 1 : 0;
    out_data->key_c     = (key_v & KEY_C_BIT)     ? 1 : 0;
    out_data->key_v     = (key_v & KEY_V_BIT)     ? 1 : 0;
    out_data->key_b     = (key_v & KEY_B_BIT)     ? 1 : 0;

    out_data->crc = (uint16_t)(p_frame[19] | (p_frame[20] << 8));
}

static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;

    if(p_msg == NULL)
    {
        return 0xffff;
    }

    while(len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return crc16;
}

static bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    uint16_t w_expected = 0;

    if((p_msg == NULL) || (len <= 2))
    {
        return false;
    }
    w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}

void USART1_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        if( control_taskHandle != NULL ) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(control_taskHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
