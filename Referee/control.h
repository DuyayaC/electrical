#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "usart.h"

#define VTM_FRAME_SIZE 21
#define RX_BUF_SIZE    42 // 两倍帧长
// 每一个按键对应的位偏移（根据说明书）
#define KEY_W_BIT      (0x01 << 0)
#define KEY_S_BIT      (0x01 << 1)
#define KEY_A_BIT      (0x01 << 2)
#define KEY_D_BIT      (0x01 << 3)
#define KEY_SHIFT_BIT  (0x01 << 4)
#define KEY_CTRL_BIT   (0x01 << 5)
#define KEY_Q_BIT      (0x01 << 6)
#define KEY_E_BIT      (0x01 << 7)
#define KEY_R_BIT      (0x01 << 8)
#define KEY_F_BIT      (0x01 << 9)
#define KEY_G_BIT      (0x01 << 10)
#define KEY_Z_BIT      (0x01 << 11)
#define KEY_X_BIT      (0x01 << 12)
#define KEY_C_BIT      (0x01 << 13)
#define KEY_V_BIT      (0x01 << 14)
#define KEY_B_BIT      (0x01 << 15)
// 21字节原始数据结构体
typedef struct {
    uint16_t ch0;        // 右摇杆水平 (11 bits)
    uint16_t ch1;        // 右摇杆竖直 (11 bits)
    uint16_t ch2;        // 左摇杆竖直 (11 bits)
    uint16_t ch3;        // 左摇杆水平 (11 bits)
    uint8_t  sw;         // 挡位切换 (2 bits): 0-C, 1-N, 2-S
    uint8_t  pause;      // 暂停按键 (1 bit)
    uint8_t  btn_l;      // 自定义左键 (1 bit)
    uint8_t  btn_r;      // 自定义右键 (1 bit)
    uint16_t dial;       // 拨轮 (11 bits)
    uint8_t  trigger;    // 扳机键 (1 bit)
    int16_t  mouse_x;    // 鼠标X轴 (16 bits, 有符号)
    int16_t  mouse_y;    // 鼠标Y轴 (16 bits, 有符号)
    int16_t  mouse_z;    // 鼠标滚轮 (16 bits, 有符号)
    uint8_t  mouse_btn_l;// 鼠标左键 (2 bits)
    uint8_t  mouse_btn_r;// 鼠标右键 (2 bits)
    uint8_t  mouse_btn_m;// 鼠标中键 (2 bits)
    uint16_t key_w;      // 键盘信息 (16 bits)
    uint16_t key_s;      
    uint16_t key_a;
    uint16_t key_d;
    uint16_t key_shift;
    uint16_t key_ctrl;
    uint16_t key_q;
    uint16_t key_e;
    uint16_t key_r;
    uint16_t key_f;
    uint16_t key_g;
    uint16_t key_z;
    uint16_t key_x;
    uint16_t key_c;
    uint16_t key_v;
    uint16_t key_b;
    uint16_t crc;        // CRC16校验 (16 bits)
} VTM_Data_t;

extern VTM_Data_t key; // 全局变量，存储接收的数据

extern void VTM_Data_Parsing(uint8_t *p_frame);