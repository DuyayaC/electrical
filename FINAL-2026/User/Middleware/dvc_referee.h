/**
 * @file dvc_referee.h
 * @brief PM01裁判系统 (纯 C 语言版 - 适配 2026 V1.2.0 通信协议)
 */

#ifndef DVC_REFEREE_H
#define DVC_REFEREE_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <string.h>

#define UART_BUFFER_SIZE 256

/* ============================ 1. 枚举定义 ============================ */

enum Enum_Referee_Status {
    Referee_Status_DISABLE = 0,
    Referee_Status_ENABLE,
};

enum Enum_Referee_Data_Status {
    Referee_Data_Status_DISABLE = 0,
    Referee_Data_Status_ENABLE,
};

enum Enum_Referee_Command_ID {
    Referee_Command_ID_GAME_STATUS = 0x0001,
    Referee_Command_ID_GAME_RESULT = 0x0002,
    Referee_Command_ID_GAME_ROBOT_HP = 0x0003,
    Referee_Command_ID_EVENT_SELF_DATA = 0x0101,
    Referee_Command_ID_EVENT_SELF_SUPPLY = 0x0102,
    Referee_Command_ID_EVENT_SELF_REFEREE_WARNING = 0x0104,
    Referee_Command_ID_EVENT_SELF_DART_STATUS = 0x0105,
    Referee_Command_ID_ROBOT_STATUS = 0x0201,
    Referee_Command_ID_ROBOT_POWER_HEAT = 0x0202,
    Referee_Command_ID_ROBOT_POSITION = 0x0203,
    Referee_Command_ID_ROBOT_BUFF = 0x0204,
    Referee_Command_ID_ROBOT_AERIAL_STATUS = 0x0205,
    Referee_Command_ID_ROBOT_DAMAGE = 0x0206,
    Referee_Command_ID_ROBOT_BOOSTER = 0x0207,
    Referee_Command_ID_ROBOT_REMAINING_AMMO = 0x0208,
    Referee_Command_ID_ROBOT_RFID = 0x0209,
    Referee_Command_ID_ROBOT_DART_COMMAND = 0x020A,
    Referee_Command_ID_ROBOT_GROUND_POSITION = 0x020B,
    Referee_Command_ID_ROBOT_RADAR_MARK = 0x020C,
    Referee_Command_ID_ROBOT_SENTRY_DECISION = 0x020D,
    Referee_Command_ID_ROBOT_RADAR_DECISION = 0x020E,
    Referee_Command_ID_INTERACTION = 0x0301,
    Referee_Command_ID_TO_CUSTOM_CLIENT = 0x0310,
    Referee_Command_ID_FROM_CUSTOM_CLIENT = 0x0311,
};

enum Enum_Referee_Interaction_Command_ID {
    Referee_Interaction_Command_ID_UI_LAYER_DELETE = 0x0100,
    Referee_Interaction_Command_ID_UI_GRAPHIC_1 = 0x0101,
    Referee_Interaction_Command_ID_UI_GRAPHIC_2 = 0x0102,
    Referee_Interaction_Command_ID_UI_GRAPHIC_5 = 0x0103,
    Referee_Interaction_Command_ID_UI_GRAPHIC_7 = 0x0104,
    Referee_Interaction_Command_ID_UI_GRAPHIC_STRING = 0x0110,
    Referee_Interaction_Command_ID_SENTRY = 0x0120,
    Referee_Interaction_Command_ID_RADAR = 0x0121,
};

enum Enum_Referee_Data_Robot_ID {
    Referee_Data_Robot_ID_NULL = 0,
    Referee_Data_Robot_ID_HERO_1 = 1,
    Referee_Data_Robot_ID_ENGINEER_2 = 2,
    Referee_Data_Robot_ID_INFANTRY_3 = 3,
    Referee_Data_Robot_ID_INFANTRY_4 = 4,
    Referee_Data_Robot_ID_INFANTRY_5 = 5,
    Referee_Data_Robot_ID_AERIAL_6 = 6,
    Referee_Data_Robot_ID_SENTRY_7 = 7,
    Referee_Data_Robot_ID_DART_8 = 8,
    Referee_Data_Robot_ID_RADAR_9 = 9,
    Referee_Data_Robot_ID_BASE_10 = 10,
    Referee_Data_Robot_ID_OUTPOST_11 = 11,
};

enum Enum_Referee_Data_Robots_ID {
    Referee_Data_Robots_ID_NO = 0,
    Referee_Data_Robots_ID_RED_HERO_1 = 1,
    Referee_Data_Robots_ID_RED_ENGINEER_2 = 2,
    Referee_Data_Robots_ID_RED_INFANTRY_3 = 3,
    Referee_Data_Robots_ID_RED_INFANTRY_4 = 4,
    Referee_Data_Robots_ID_RED_INFANTRY_5 = 5,
    Referee_Data_Robots_ID_RED_AERIAL_6 = 6,
    Referee_Data_Robots_ID_RED_SENTRY_7 = 7,
    Referee_Data_Robots_ID_RED_DART_8 = 8,
    Referee_Data_Robots_ID_RED_RADAR_9 = 9,
    Referee_Data_Robots_ID_RED_BASE_10 = 10,
    Referee_Data_Robots_ID_RED_OUTPOST_11 = 11,
    Referee_Data_Robots_ID_BLUE_HERO_1 = 101,
    Referee_Data_Robots_ID_BLUE_ENGINEER_2 = 102,
    Referee_Data_Robots_ID_BLUE_INFANTRY_3 = 103,
    Referee_Data_Robots_ID_BLUE_INFANTRY_4 = 104,
    Referee_Data_Robots_ID_BLUE_INFANTRY_5 = 105,
    Referee_Data_Robots_ID_BLUE_AERIAL_6 = 106,
    Referee_Data_Robots_ID_BLUE_SENTRY_7 = 107,
    Referee_Data_Robots_ID_BLUE_DART_8 = 108,
    Referee_Data_Robots_ID_BLUE_RADAR_9 = 109,
    Referee_Data_Robots_ID_BLUE_BASE_10 = 110,
    Referee_Data_Robots_ID_BLUE_OUTPOST_11 = 111,
};

enum Enum_Referee_Game_Result {
    Referee_Game_Result_DRAW = 0,
    Referee_Game_Result_RED_WIN,
    Referee_Game_Result_BLUE_WIN,
};

enum Enum_Referee_Data_Interaction_Layer_Delete_Operation {
    Referee_Data_Interaction_Layer_Delete_Operation_NULL = 0,
    Referee_Data_Interaction_Layer_Delete_Operation_CLEAR_ONE,
    Referee_Data_Interaction_Layer_Delete_Operation_CLEAR_ALL,
};

enum Enum_Referee_Data_Interaction_Graphic_Operation {
    Referee_Data_Interaction_Graphic_Operation_NULL = 0,
    Referee_Data_Interaction_Graphic_Operation_ADD,
    Referee_Data_Interaction_Graphic_Operation_CHANGE,
    Referee_Data_Interaction_Graphic_Operation_DELETE,
};

enum Enum_Referee_Data_Interaction_Graphic_Type {
    Referee_Data_Interaction_Graphic_Type_LINE = 0,
    Referee_Data_Interaction_Graphic_Type_RECTANGLE,
    Referee_Data_Interaction_Graphic_Type_CIRCLE,
    Referee_Data_Interaction_Graphic_Type_OVAL,
    Referee_Data_Interaction_Graphic_Type_ARC,
    Referee_Data_Interaction_Graphic_Type_FLOAT,
    Referee_Data_Interaction_Graphic_Type_INTEGER,
    Referee_Data_Interaction_Graphic_Type_STRING,
};

enum Enum_Referee_Data_Interaction_Graphic_Color {
    Referee_Data_Interaction_Graphic_Color_MAIN = 0,
    Referee_Data_Interaction_Graphic_Color_YELLOW,
    Referee_Data_Interaction_Graphic_Color_GREEN,
    Referee_Data_Interaction_Graphic_Color_ORANGE,
    Referee_Data_Interaction_Graphic_Color_PURPLE,
    Referee_Data_Interaction_Graphic_Color_PINK,
    Referee_Data_Interaction_Graphic_Color_CYAN,
    Referee_Data_Interaction_Graphic_Color_BLACK,
    Referee_Data_Interaction_Graphic_Color_WHITE,
};

/* ============================ 2. 结构体定义 ============================ */

struct Struct_Referee_UART_Data {
    uint8_t Frame_Header;
    uint16_t Data_Length;
    uint8_t Sequence;
    uint8_t CRC_8;
    uint16_t Referee_Command_ID; 
    uint8_t Data[320];
} __attribute__((packed));

struct Struct_Referee_Data_Interaction_Graphic_Config {
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Layer_Num : 4;
    uint32_t Color_Enum : 4;
    uint32_t Details_A : 9;
    uint32_t Details_B : 9;
    uint32_t Line_Width : 10;
    uint32_t Start_X : 11;
    uint32_t Start_Y : 11;
    uint32_t Details_C : 10;
    uint32_t Details_D : 11;
    uint32_t Details_E : 11;
} __attribute__((packed));

/* 各数据包内容结构体 (对应不同的 Command ID) */
struct Struct_Referee_Rx_Data_Game_Status {
    uint8_t Type_Enum : 4;
    uint8_t Stage_Enum : 4;
    uint16_t Remaining_Time;
    uint64_t Timestamp;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Game_Result {
    uint8_t Result;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Game_Robot_HP {
    uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;
    uint16_t reserved;
    uint16_t ally_7_robot_HP;
    uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Event_Self_Data {
    uint32_t event_data;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Event_Self_Supply {
    uint8_t Reserved;
    uint8_t Robot;
    uint8_t Status;
    uint8_t Ammo_Number;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Event_Referee_Warning {
    uint8_t Level;
    uint8_t Robot_ID;
    uint8_t Count;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Event_Dart_Status {
    uint8_t Dart_Remaining_Time;
    uint16_t dart_info;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Robot_Status {
    uint8_t Robot_ID;
    uint8_t Level;
    uint16_t HP;
    uint16_t HP_Max;
    uint16_t Booster_Heat_CD;
    uint16_t Booster_Heat_Max;
    uint16_t Chassis_Power_Max;
    uint8_t PM01_Gimbal_Status_Enum : 1;
    uint8_t PM01_Chassis_Status_Enum : 1;
    uint8_t PM01_Booster_Status_Enum : 1;
    uint8_t Reserved : 5;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Robot_Power_Heat {
    uint16_t reserved1;
    uint16_t reserved2;
    float reserved3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Robot_Position {
    float Location_X;
    float Location_Y;
    float Location_Yaw;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Robot_Buff {
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Robot_Damage {
    uint8_t Armor_ID : 4;
    uint8_t Type_Enum : 4;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Robot_Booster {
    uint8_t Ammo_Type;
    uint8_t Booster_Type;
    uint8_t Frequency;
    float Speed;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Robot_Remaining_Ammo {
    uint16_t Booster_17mm;
    uint16_t Booster_42mm;
    uint16_t Money;
    uint16_t Fortress_17mm;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Robot_RFID {
    uint32_t rfid_status;
    uint8_t rfid_status_2;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Robot_Dart_Command {
    uint8_t Status;
    uint8_t Reserved;
    uint16_t Switch_Remaining_Time;
    uint16_t Launch_Remaining_Time;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Robot_Sentry_Location {
    float Hero_X;
    float Hero_Y;
    float Engineer_X;
    float Engineer_Y;
    float Infantry_3_X;
    float Infantry_3_Y;
    float Infantry_4_X;
    float Infantry_4_Y;
    float Reserved_1;
    float Reserved_2;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Robot_Radar_Mark {
    uint16_t mark_progress;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Robot_Sentry_Decision {
    uint32_t sentry_info;
    uint16_t sentry_info_2;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Robot_Radar_Decision {
    uint8_t radar_info;
} __attribute__((packed));

struct Struct_Referee_Rx_Data_Interaction {
    uint16_t Sub_Command_ID;
    uint16_t Sender_ID;
    uint16_t Receiver_ID;
    uint8_t User_Data[112];
} __attribute__((packed));

struct Struct_Referee_Rx_Data_From_Custom_Client {
    uint8_t Data[30];
} __attribute__((packed));


/* ============================ 3. 裁判系统主结构体定义 ============================ */

typedef __packed struct {
    UART_HandleTypeDef *huart;
    uint8_t Tx_Buffer[UART_BUFFER_SIZE];
    uint8_t Rx_Buffer[UART_BUFFER_SIZE];
    uint16_t Rx_Buffer_Length;

    uint8_t Frame_Header;
    uint32_t Flag;
    uint32_t Pre_Flag;
    uint8_t Sequence;
    uint8_t UI_Change_Flag[10][10];

    enum Enum_Referee_Status Referee_Status;
    enum Enum_Referee_Data_Status Referee_Trust_Status;

    /* 各种缓存数据结构 */
    struct Struct_Referee_Rx_Data_Game_Status Game_Status;
    struct Struct_Referee_Rx_Data_Game_Result Game_Result;
    struct Struct_Referee_Rx_Data_Game_Robot_HP Game_Robot_HP;
    struct Struct_Referee_Rx_Data_Event_Self_Data Event_Self_Data;
    struct Struct_Referee_Rx_Data_Event_Self_Supply Event_Self_Supply;
    struct Struct_Referee_Rx_Data_Event_Referee_Warning Event_Referee_Warning;
    struct Struct_Referee_Rx_Data_Event_Dart_Status Event_Dart_Status;
    struct Struct_Referee_Rx_Data_Robot_Status Robot_Status;
    struct Struct_Referee_Rx_Data_Robot_Power_Heat Robot_Power_Heat;
    struct Struct_Referee_Rx_Data_Robot_Position Robot_Position;
    struct Struct_Referee_Rx_Data_Robot_Buff Robot_Buff;
    struct Struct_Referee_Rx_Data_Robot_Damage Robot_Damage;
    struct Struct_Referee_Rx_Data_Robot_Booster Robot_Booster;
    struct Struct_Referee_Rx_Data_Robot_Remaining_Ammo Robot_Remaining_Ammo;
    struct Struct_Referee_Rx_Data_Robot_RFID Robot_RFID;
    struct Struct_Referee_Rx_Data_Robot_Dart_Command Robot_Dart_Command;
    struct Struct_Referee_Rx_Data_Robot_Sentry_Location Robot_Sentry_Location;
    struct Struct_Referee_Rx_Data_Robot_Radar_Mark Robot_Radar_Mark;
    struct Struct_Referee_Rx_Data_Robot_Sentry_Decision Robot_Sentry_Decision;
    struct Struct_Referee_Rx_Data_Robot_Radar_Decision Robot_Radar_Decision;
    struct Struct_Referee_Rx_Data_Interaction Interaction_Data;
    struct Struct_Referee_Rx_Data_From_Custom_Client From_Custom_Client_Data;

    uint8_t To_Custom_Client_Data[300]; 
    struct Struct_Referee_Data_Interaction_Graphic_Config Graphic_Config[10][10];

} Referee_System_t;


/* ============================ 4. 函数声明 ============================ */

void Referee_Init(Referee_System_t *ref, UART_HandleTypeDef *huart, uint8_t frame_header);
void Referee_Unpack(Referee_System_t *ref, uint8_t *Rx_Data, uint16_t Length);
void Referee_Register_Task(Referee_System_t *ref, osThreadId task_handle);
uint16_t Referee_Get_RxEvent_Size(void);

extern Referee_System_t referee;
// void Referee_TIM_1000ms_Alive_PeriodElapsedCallback(Referee_System_t *ref);

/* UI 构建与修改接口 */
// void Referee_Set_UI_Change_Flag_Clear(Referee_System_t *ref);
// struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_Clear(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num);
// struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_Line(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num, uint32_t Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t End_X, uint32_t End_Y);
// struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_Rectangle(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num, uint32_t Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t End_X, uint32_t End_Y);
// struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_Circle(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num, uint32_t Color, uint32_t Line_Width, uint32_t Center_X, uint32_t Center_Y, uint32_t Radius);
// struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_Oval(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num, uint32_t Color, uint32_t Line_Width, uint32_t Center_X, uint32_t Center_Y, uint32_t Length_X, uint32_t Length_Y);
// struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_Arc(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num, uint32_t Color, uint32_t Line_Width, uint32_t Center_X, uint32_t Center_Y, uint32_t Angle_Start, uint32_t Angle_End, uint32_t Length_X, uint32_t Length_Y);
// struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_Float(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num, uint32_t Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t Font_Width, float Float);
// struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_Integer(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num, uint32_t Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t Font_Width, int32_t Integer);
// struct Struct_Referee_Data_Interaction_Graphic_Config *Referee_Set_UI_String(Referee_System_t *ref, uint8_t Layer_Num, uint8_t Graphic_Num, uint32_t Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t Font_Width, uint32_t String_Length);

/* 串口发送图传接口 */
// void Referee_UART_Send_Interaction_UI_Layer_Delete(Referee_System_t *ref, uint8_t Operation, uint8_t Layer);
// void Referee_UART_Send_Interaction_UI_Graphic_1(Referee_System_t *ref, struct Struct_Referee_Data_Interaction_Graphic_Config *G1);
// void Referee_UART_Send_Interaction_UI_Graphic_2(Referee_System_t *ref, struct Struct_Referee_Data_Interaction_Graphic_Config *G1, struct Struct_Referee_Data_Interaction_Graphic_Config *G2);
// void Referee_UART_Send_Interaction_UI_Graphic_5(Referee_System_t *ref, struct Struct_Referee_Data_Interaction_Graphic_Config *G1, struct Struct_Referee_Data_Interaction_Graphic_Config *G2, struct Struct_Referee_Data_Interaction_Graphic_Config *G3, struct Struct_Referee_Data_Interaction_Graphic_Config *G4, struct Struct_Referee_Data_Interaction_Graphic_Config *G5);
// void Referee_UART_Send_Interaction_UI_Graphic_7(Referee_System_t *ref, struct Struct_Referee_Data_Interaction_Graphic_Config *G1, struct Struct_Referee_Data_Interaction_Graphic_Config *G2, struct Struct_Referee_Data_Interaction_Graphic_Config *G3, struct Struct_Referee_Data_Interaction_Graphic_Config *G4, struct Struct_Referee_Data_Interaction_Graphic_Config *G5, struct Struct_Referee_Data_Interaction_Graphic_Config *G6, struct Struct_Referee_Data_Interaction_Graphic_Config *G7);
// void Referee_UART_Send_Interaction_UI_Graphic_String(Referee_System_t *ref, struct Struct_Referee_Data_Interaction_Graphic_Config *GStr, const char *Content);
#endif /* DVC_REFEREE_H */
