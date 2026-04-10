/**
 * @file dvc_referee.c
 * @brief PM01裁判系统 (纯 C 语言版 - 适配 2026 V1.2.0 通信协议)
 */

#include "dvc_referee.h"

/* CRC8 校验表 */
static const uint8_t crc_8_table[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
};

/* CRC16 校验表 */
static const uint16_t crc_16_table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

static uint8_t Verify_CRC_8(uint8_t *Message, uint32_t Length) {
    uint8_t index;
    uint8_t check = 0xff;
    if (Message == NULL) return check;
    while (Length--) {
        index = *Message;
        Message++;
        check = crc_8_table[check ^ index];
    }
    return check;
}

static uint16_t Verify_CRC_16(uint8_t *Message, uint32_t Length) {
    uint8_t index;
    uint16_t check = 0xffff;
    if (Message == NULL) return check;
    while (Length--) {
        index = *Message;
        Message++;
        check = ((uint16_t)(check) >> 8) ^ crc_16_table[((uint16_t)(check) ^ (uint16_t)(index)) & 0xff];
    }
    return check;
}


/* ============================ 裁判系统核心函数 ============================ */

void Referee_Init(Referee_System_t *ref, UART_HandleTypeDef *huart, uint8_t frame_header) {
    memset(ref, 0, sizeof(Referee_System_t));
    ref->huart = huart;
    ref->Rx_Buffer_Length = UART_BUFFER_SIZE;
    ref->Frame_Header = frame_header;
    ref->Referee_Status = Referee_Status_DISABLE;
    ref->Referee_Trust_Status = Referee_Data_Status_ENABLE;

    // 开启 DMA 空闲中断接收
    if (ref->huart != NULL) {
        HAL_UARTEx_ReceiveToIdle_DMA(ref->huart, ref->Rx_Buffer, ref->Rx_Buffer_Length);
    }
}

void Referee_TIM_1000ms_Alive_PeriodElapsedCallback(Referee_System_t *ref) {
    if (ref->Flag == ref->Pre_Flag) {
        ref->Referee_Status = Referee_Status_DISABLE;
    } else {
        ref->Referee_Status = Referee_Status_ENABLE;
    }
    ref->Pre_Flag = ref->Flag;
}


/* ============================ UI 缓存构建接口 ============================ */

void Referee_Set_UI_Change_Flag_Clear(Referee_System_t *ref) {
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            ref->UI_Change_Flag[i][j] = 0;
        }
    }
}

struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_Clear(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num) {
    struct Struct_Referee_Data_Interaction_Graphic_Config *cfg = &ref->Graphic_Config[Layer_Num][Graphic_Num];
    cfg->Index[0] = '0';
    cfg->Index[1] = '0' + Layer_Num;
    cfg->Index[2] = '0' + Graphic_Num;
    cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_DELETE;
    ref->UI_Change_Flag[Layer_Num][Graphic_Num] = 0;
    return cfg;
}

struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_Line(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num, uint32_t Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t End_X, uint32_t End_Y) {
    struct Struct_Referee_Data_Interaction_Graphic_Config *cfg = &ref->Graphic_Config[Layer_Num][Graphic_Num];
    cfg->Index[0] = '0';
    cfg->Index[1] = '0' + Layer_Num;
    cfg->Index[2] = '0' + Graphic_Num;

    if (ref->UI_Change_Flag[Layer_Num][Graphic_Num] == 0) {
        cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        ref->UI_Change_Flag[Layer_Num][Graphic_Num] = 1;
    } else {
        cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    cfg->Type_Enum = Referee_Data_Interaction_Graphic_Type_LINE;
    cfg->Layer_Num = Layer_Num;
    cfg->Color_Enum = Color;
    cfg->Line_Width = Line_Width;
    cfg->Start_X = Start_X;
    cfg->Start_Y = Start_Y;
    cfg->Details_D = End_X;
    cfg->Details_E = End_Y;
    return cfg;
}

struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_Rectangle(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num, uint32_t Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t End_X, uint32_t End_Y) {
    struct Struct_Referee_Data_Interaction_Graphic_Config *cfg = &ref->Graphic_Config[Layer_Num][Graphic_Num];
    cfg->Index[0] = '0';
    cfg->Index[1] = '0' + Layer_Num;
    cfg->Index[2] = '0' + Graphic_Num;

    if (ref->UI_Change_Flag[Layer_Num][Graphic_Num] == 0) {
        cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        ref->UI_Change_Flag[Layer_Num][Graphic_Num] = 1;
    } else {
        cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    cfg->Type_Enum = Referee_Data_Interaction_Graphic_Type_RECTANGLE;
    cfg->Layer_Num = Layer_Num;
    cfg->Color_Enum = Color;
    cfg->Line_Width = Line_Width;
    cfg->Start_X = Start_X;
    cfg->Start_Y = Start_Y;
    cfg->Details_D = End_X;
    cfg->Details_E = End_Y;
    return cfg;
}

struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_Circle(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num, uint32_t Color, uint32_t Line_Width, uint32_t Center_X, uint32_t Center_Y, uint32_t Radius) {
    struct Struct_Referee_Data_Interaction_Graphic_Config *cfg = &ref->Graphic_Config[Layer_Num][Graphic_Num];
    cfg->Index[0] = '0';
    cfg->Index[1] = '0' + Layer_Num;
    cfg->Index[2] = '0' + Graphic_Num;

    if (ref->UI_Change_Flag[Layer_Num][Graphic_Num] == 0) {
        cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        ref->UI_Change_Flag[Layer_Num][Graphic_Num] = 1;
    } else {
        cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    cfg->Type_Enum = Referee_Data_Interaction_Graphic_Type_CIRCLE;
    cfg->Layer_Num = Layer_Num;
    cfg->Color_Enum = Color;
    cfg->Line_Width = Line_Width;
    cfg->Start_X = Center_X;
    cfg->Start_Y = Center_Y;
    cfg->Details_C = Radius;
    return cfg;
}

struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_Oval(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num, uint32_t Color, uint32_t Line_Width, uint32_t Center_X, uint32_t Center_Y, uint32_t Length_X, uint32_t Length_Y) {
    struct Struct_Referee_Data_Interaction_Graphic_Config *cfg = &ref->Graphic_Config[Layer_Num][Graphic_Num];
    cfg->Index[0] = '0';
    cfg->Index[1] = '0' + Layer_Num;
    cfg->Index[2] = '0' + Graphic_Num;

    if (ref->UI_Change_Flag[Layer_Num][Graphic_Num] == 0) {
        cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        ref->UI_Change_Flag[Layer_Num][Graphic_Num] = 1;
    } else {
        cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    cfg->Type_Enum = Referee_Data_Interaction_Graphic_Type_OVAL;
    cfg->Layer_Num = Layer_Num;
    cfg->Color_Enum = Color;
    cfg->Line_Width = Line_Width;
    cfg->Start_X = Center_X;
    cfg->Start_Y = Center_Y;
    cfg->Details_D = Length_X;
    cfg->Details_E = Length_Y;
    return cfg;
}

struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_Arc(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num, uint32_t Color, uint32_t Line_Width, uint32_t Center_X, uint32_t Center_Y, uint32_t Angle_Start, uint32_t Angle_End, uint32_t Length_X, uint32_t Length_Y) {
    struct Struct_Referee_Data_Interaction_Graphic_Config *cfg = &ref->Graphic_Config[Layer_Num][Graphic_Num];
    cfg->Index[0] = '0';
    cfg->Index[1] = '0' + Layer_Num;
    cfg->Index[2] = '0' + Graphic_Num;

    if (ref->UI_Change_Flag[Layer_Num][Graphic_Num] == 0) {
        cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        ref->UI_Change_Flag[Layer_Num][Graphic_Num] = 1;
    } else {
        cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    cfg->Type_Enum = Referee_Data_Interaction_Graphic_Type_ARC;
    cfg->Layer_Num = Layer_Num;
    cfg->Color_Enum = Color;
    cfg->Line_Width = Line_Width;
    cfg->Start_X = Center_X;
    cfg->Start_Y = Center_Y;
    cfg->Details_A = Angle_Start;
    cfg->Details_B = Angle_End;
    cfg->Details_D = Length_X;
    cfg->Details_E = Length_Y;
    return cfg;
}

struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_Float(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num, uint32_t Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t Font_Width, float Float) {
    struct Struct_Referee_Data_Interaction_Graphic_Config *cfg = &ref->Graphic_Config[Layer_Num][Graphic_Num];
    cfg->Index[0] = '0';
    cfg->Index[1] = '0' + Layer_Num;
    cfg->Index[2] = '0' + Graphic_Num;

    if (ref->UI_Change_Flag[Layer_Num][Graphic_Num] == 0) {
        cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        ref->UI_Change_Flag[Layer_Num][Graphic_Num] = 1;
    } else {
        cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    cfg->Type_Enum = Referee_Data_Interaction_Graphic_Type_FLOAT;
    cfg->Layer_Num = Layer_Num;
    cfg->Color_Enum = Color;
    cfg->Line_Width = Line_Width;
    cfg->Start_X = Start_X;
    cfg->Start_Y = Start_Y;
    cfg->Details_A = Font_Width;
    int32_t *tmp_pointer = (int32_t *) ((uint32_t) cfg + 11);
    *tmp_pointer = (int32_t) (Float * 1000.0f);
    return cfg;
}

struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_Integer(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num, uint32_t Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t Font_Width, int32_t Integer) {
    struct Struct_Referee_Data_Interaction_Graphic_Config *cfg = &ref->Graphic_Config[Layer_Num][Graphic_Num];
    cfg->Index[0] = '0';
    cfg->Index[1] = '0' + Layer_Num;
    cfg->Index[2] = '0' + Graphic_Num;

    if (ref->UI_Change_Flag[Layer_Num][Graphic_Num] == 0) {
        cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        ref->UI_Change_Flag[Layer_Num][Graphic_Num] = 1;
    } else {
        cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    cfg->Type_Enum = Referee_Data_Interaction_Graphic_Type_INTEGER;
    cfg->Layer_Num = Layer_Num;
    cfg->Color_Enum = Color;
    cfg->Line_Width = Line_Width;
    cfg->Start_X = Start_X;
    cfg->Start_Y = Start_Y;
    cfg->Details_A = Font_Width;
    int32_t *tmp_pointer = (int32_t *) ((uint32_t) cfg + 11);
    *tmp_pointer = Integer;
    return cfg;
}

struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_String(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num, uint32_t Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t Font_Width, uint32_t String_Length) {
    struct Struct_Referee_Data_Interaction_Graphic_Config *cfg = &ref->Graphic_Config[Layer_Num][Graphic_Num];
    cfg->Index[0] = '0';
    cfg->Index[1] = '0' + Layer_Num;
    cfg->Index[2] = '0' + Graphic_Num;

    if (ref->UI_Change_Flag[Layer_Num][Graphic_Num] == 0) {
        cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        ref->UI_Change_Flag[Layer_Num][Graphic_Num] = 1;
    } else {
        cfg->Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    cfg->Type_Enum = Referee_Data_Interaction_Graphic_Type_STRING;
    cfg->Layer_Num = Layer_Num;
    cfg->Color_Enum = Color;
    cfg->Line_Width = Line_Width;
    cfg->Start_X = Start_X;
    cfg->Start_Y = Start_Y;
    cfg->Details_A = Font_Width;
    cfg->Details_B = String_Length;
    return cfg;
}


/* ============================ UI 数据发送封装 ============================ */

static void _Referee_Prepare_Header(Referee_System_t *ref, uint16_t data_length) {
    struct Struct_Referee_UART_Data *tx_buf = (struct Struct_Referee_UART_Data *)ref->Tx_Buffer;
    tx_buf->Frame_Header = ref->Frame_Header;
    tx_buf->Data_Length = data_length;
    tx_buf->Sequence = ref->Sequence++;
    tx_buf->CRC_8 = Verify_CRC_8((uint8_t *)tx_buf, 4);
    tx_buf->Referee_Command_ID = Referee_Command_ID_INTERACTION;
}

static void _Referee_Fill_Interaction_Header(Referee_System_t *ref, uint8_t *data_ptr, uint16_t sub_id) {
    uint16_t sender_id = ref->Robot_Status.Robot_ID;
    uint16_t receiver_id = sender_id + 0x100;
    memcpy(data_ptr, &sub_id, 2);
    memcpy(data_ptr + 2, &sender_id, 2);
    memcpy(data_ptr + 4, &receiver_id, 2);
}

static void _Referee_Transmit(Referee_System_t *ref) {
    struct Struct_Referee_UART_Data *tx_buf = (struct Struct_Referee_UART_Data *)ref->Tx_Buffer;
    uint16_t total_len = 9 + tx_buf->Data_Length;
    uint16_t crc16 = Verify_CRC_16((uint8_t *)tx_buf, total_len - 2);
    memcpy((uint8_t *)tx_buf + total_len - 2, &crc16, 2);
    
    // 使用挂载的 huart 直接发送
    HAL_UART_Transmit(ref->huart, (uint8_t *)tx_buf, total_len, 100);
}

void Referee_UART_Send_Interaction_UI_Layer_Delete(Referee_System_t *ref, uint8_t Operation, uint8_t Layer) {
    uint16_t biz_len = 6 + 2; 
    _Referee_Prepare_Header(ref, biz_len);
    struct Struct_Referee_UART_Data *tx_buf = (struct Struct_Referee_UART_Data *)ref->Tx_Buffer;
    _Referee_Fill_Interaction_Header(ref, tx_buf->Data, 0x0100);
    tx_buf->Data[6] = Operation;
    tx_buf->Data[7] = Layer;
    _Referee_Transmit(ref);
}

void Referee_UART_Send_Interaction_UI_Graphic_1(Referee_System_t *ref, struct Struct_Referee_Data_Interaction_Graphic_Config *G1) {
    uint16_t biz_len = 6 + 15; 
    _Referee_Prepare_Header(ref, biz_len);
    struct Struct_Referee_UART_Data *tx_buf = (struct Struct_Referee_UART_Data *)ref->Tx_Buffer;
    _Referee_Fill_Interaction_Header(ref, tx_buf->Data, 0x0101);
    memcpy(tx_buf->Data + 6, G1, 15);
    _Referee_Transmit(ref);
}

void Referee_UART_Send_Interaction_UI_Graphic_2(Referee_System_t *ref, struct Struct_Referee_Data_Interaction_Graphic_Config *G1, struct Struct_Referee_Data_Interaction_Graphic_Config *G2) {
    uint16_t biz_len = 6 + 15 * 2; 
    _Referee_Prepare_Header(ref, biz_len);
    struct Struct_Referee_UART_Data *tx_buf = (struct Struct_Referee_UART_Data *)ref->Tx_Buffer;
    _Referee_Fill_Interaction_Header(ref, tx_buf->Data, 0x0102);
    memcpy(tx_buf->Data + 6, G1, 15);
    memcpy(tx_buf->Data + 6 + 15, G2, 15);
    _Referee_Transmit(ref);
}

void Referee_UART_Send_Interaction_UI_Graphic_5(Referee_System_t *ref, struct Struct_Referee_Data_Interaction_Graphic_Config *G1, struct Struct_Referee_Data_Interaction_Graphic_Config *G2, struct Struct_Referee_Data_Interaction_Graphic_Config *G3, struct Struct_Referee_Data_Interaction_Graphic_Config *G4, struct Struct_Referee_Data_Interaction_Graphic_Config *G5) {
    uint16_t biz_len = 6 + 15 * 5;
    _Referee_Prepare_Header(ref, biz_len);
    struct Struct_Referee_UART_Data *tx_buf = (struct Struct_Referee_UART_Data *)ref->Tx_Buffer;
    _Referee_Fill_Interaction_Header(ref, tx_buf->Data, 0x0103);
    memcpy(tx_buf->Data + 6, G1, 15);
    memcpy(tx_buf->Data + 6 + 15, G2, 15);
    memcpy(tx_buf->Data + 6 + 30, G3, 15);
    memcpy(tx_buf->Data + 6 + 45, G4, 15);
    memcpy(tx_buf->Data + 6 + 60, G5, 15);
    _Referee_Transmit(ref);
}

void Referee_UART_Send_Interaction_UI_Graphic_7(Referee_System_t *ref, struct Struct_Referee_Data_Interaction_Graphic_Config *G1, struct Struct_Referee_Data_Interaction_Graphic_Config *G2, struct Struct_Referee_Data_Interaction_Graphic_Config *G3, struct Struct_Referee_Data_Interaction_Graphic_Config *G4, struct Struct_Referee_Data_Interaction_Graphic_Config *G5, struct Struct_Referee_Data_Interaction_Graphic_Config *G6, struct Struct_Referee_Data_Interaction_Graphic_Config *G7) {
    uint16_t biz_len = 6 + 15 * 7;
    _Referee_Prepare_Header(ref, biz_len);
    struct Struct_Referee_UART_Data *tx_buf = (struct Struct_Referee_UART_Data *)ref->Tx_Buffer;
    _Referee_Fill_Interaction_Header(ref, tx_buf->Data, 0x0104);
    memcpy(tx_buf->Data + 6, G1, 15);
    memcpy(tx_buf->Data + 6 + 15, G2, 15);
    memcpy(tx_buf->Data + 6 + 30, G3, 15);
    memcpy(tx_buf->Data + 6 + 45, G4, 15);
    memcpy(tx_buf->Data + 6 + 60, G5, 15);
    memcpy(tx_buf->Data + 6 + 75, G6, 15);
    memcpy(tx_buf->Data + 6 + 90, G7, 15);
    _Referee_Transmit(ref);
}

void Referee_UART_Send_Interaction_UI_Graphic_String(Referee_System_t *ref, struct Struct_Referee_Data_Interaction_Graphic_Config *GStr, const char *Content) {
    uint16_t biz_len = 6 + 15 + 30; 
    _Referee_Prepare_Header(ref, biz_len);
    struct Struct_Referee_UART_Data *tx_buf = (struct Struct_Referee_UART_Data *)ref->Tx_Buffer;
    _Referee_Fill_Interaction_Header(ref, tx_buf->Data, 0x0110);
    memcpy(tx_buf->Data + 6, GStr, 15);
    uint8_t *str_ptr = tx_buf->Data + 6 + 15;
    memset(str_ptr, 0, 30);
    strncpy((char *)str_ptr, Content, 30);
    _Referee_Transmit(ref);
}

/* ============================ 接收回调与解析 ============================ */

/**
 * @brief 内部函数：解析单个完整的裁判系统数据包
 * @param ref 裁判系统句柄
 * @param packet_ptr 指向当前数据包起始位置 (0xA5) 的指针
 */
static void Referee_Solve_Single_Packet(Referee_System_t *ref, uint8_t *packet_ptr) {
    // 将指针转换为统一的消息结构体
    struct Struct_Referee_UART_Data *tmp_buffer = (struct Struct_Referee_UART_Data *)packet_ptr;

    // 根据命令码分发数据到对应的结构体
    switch (tmp_buffer->Referee_Command_ID) {
        case Referee_Command_ID_GAME_STATUS:
            memcpy(&ref->Game_Status, tmp_buffer->Data, sizeof(struct Struct_Referee_Rx_Data_Game_Status));
            break;
        case Referee_Command_ID_GAME_RESULT:
            memcpy(&ref->Game_Result, tmp_buffer->Data, sizeof(struct Struct_Referee_Rx_Data_Game_Result));
            break;
        case Referee_Command_ID_GAME_ROBOT_HP:
            memcpy(&ref->Game_Robot_HP, tmp_buffer->Data, sizeof(struct Struct_Referee_Rx_Data_Game_Robot_HP));
            break;
        case Referee_Command_ID_EVENT_SELF_DATA:
            memcpy(&ref->Event_Self_Data, tmp_buffer->Data, sizeof(struct Struct_Referee_Rx_Data_Event_Self_Data));
            break;
        case Referee_Command_ID_ROBOT_STATUS:
            memcpy(&ref->Robot_Status, tmp_buffer->Data, sizeof(struct Struct_Referee_Rx_Data_Robot_Status));
            break;
        case Referee_Command_ID_ROBOT_POWER_HEAT:
            memcpy(&ref->Robot_Power_Heat, tmp_buffer->Data, sizeof(struct Struct_Referee_Rx_Data_Robot_Power_Heat));
            break;
        case Referee_Command_ID_ROBOT_POSITION:
            memcpy(&ref->Robot_Position, tmp_buffer->Data, sizeof(struct Struct_Referee_Rx_Data_Robot_Position));
            break;
        case Referee_Command_ID_ROBOT_BUFF:
            memcpy(&ref->Robot_Buff, tmp_buffer->Data, sizeof(struct Struct_Referee_Rx_Data_Robot_Buff));
            break;
        case Referee_Command_ID_ROBOT_DAMAGE:
            memcpy(&ref->Robot_Damage, tmp_buffer->Data, sizeof(struct Struct_Referee_Rx_Data_Robot_Damage));
            break;
        case Referee_Command_ID_ROBOT_REMAINING_AMMO:
            memcpy(&ref->Robot_Remaining_Ammo, tmp_buffer->Data, sizeof(struct Struct_Referee_Rx_Data_Robot_Remaining_Ammo));
            break;
        case Referee_Command_ID_INTERACTION: {
            uint16_t copy_length = tmp_buffer->Data_Length > sizeof(struct Struct_Referee_Rx_Data_Interaction) ? 
                                   sizeof(struct Struct_Referee_Rx_Data_Interaction) : tmp_buffer->Data_Length;
            memcpy(&ref->Interaction_Data, tmp_buffer->Data, copy_length);
            break;
        }
        default:
            break;
    }
    
    // 每成功解析一包，增加标志位供在线检查使用
    ref->Flag++;
}

/**
 * @brief 裁判系统串口接收完成回调函数
 * @param ref 裁判系统句柄
 * @param Rx_Data 接收到的原始数据缓冲区
 * @param Length 接收到的总字节长度
 */
void Referee_UART_RxCpltCallback(Referee_System_t *ref, uint8_t *Rx_Data, uint16_t Length) {
    if (Rx_Data == NULL || Length < 9) return; // 长度不足一包最小长度

    uint16_t ptr = 0; // 滑动窗口指针

    // 遍历整个接收到的缓冲区
    while (ptr <= (Length - 9)) {
        // 1. 寻找帧头 0xA5
        if (Rx_Data[ptr] == 0xA5) {
            
            // 2. 验证帧头 CRC8 (对前 4 字节校验，结果与第 5 字节对比)
            if (Verify_CRC_8(&Rx_Data[ptr], 4) == Rx_Data[ptr + 4]) {
                
                // 3. 计算这一包的理论总长度
                uint16_t data_len = (Rx_Data[ptr + 2] << 8) | Rx_Data[ptr + 1];
                uint16_t whole_packet_len = data_len + 9;

                // 4. 确保剩余数据长度足够一整包，且进行整包 CRC16 校验
                if (ptr + whole_packet_len <= Length) {
                    uint16_t calc_crc16 = Verify_CRC_16(&Rx_Data[ptr], whole_packet_len - 2);
                    uint16_t frame_crc16 = (Rx_Data[ptr + whole_packet_len - 1] << 8) | Rx_Data[ptr + whole_packet_len - 2];

                    if (calc_crc16 == frame_crc16) {
                        // 5. 校验全部通过，此时 ptr 指向的就是合法的 0xA5
                        Referee_Solve_Single_Packet(ref, &Rx_Data[ptr]);

                        // 6. 跳过已解析的这一包，继续找下一包（防止一帧里有两包数据）
                        ptr += whole_packet_len;
                        continue; 
                    }
                }
            }
        }
        // 如果没找到 0xA5 或者校验失败，指针后移，继续寻找
        ptr++;
    }
}

/**
 * @brief 对外接口：串口空闲中断数据处理与重启（替代原先放在 drv_uart.c 里的回调）
 * @param ref 裁判系统句柄
 * @param Size DMA 接收到的实际数据长度
 */
void Referee_UART_RxEventCallback(Referee_System_t *ref, uint16_t Size) {
    if (ref == NULL || ref->huart == NULL) return;
    
    // 1. 调用滑窗解包逻辑解析当前这波数据
    Referee_UART_RxCpltCallback(ref, ref->Rx_Buffer, Size);
    
    // 2. 数据处理完后，重新挂载 DMA 接收准备抓取下一波数据
    HAL_UARTEx_ReceiveToIdle_DMA(ref->huart, ref->Rx_Buffer, ref->Rx_Buffer_Length);
}
