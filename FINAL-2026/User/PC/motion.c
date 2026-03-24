#include "motion.h"

usb_t usb;
motion_t motion;
// 定义全局指针，分配 4 字节的指针内存
AnglePacket_t* pAngleData = NULL;

/**
 * @brief  协议解析函数 (仅帧头帧尾验证)
 */
uint8_t usbreceive(uint8_t* data, uint16_t len) {
    if (len < sizeof(AnglePacket_t)) return 0;

    for (uint16_t i = 0; i <= (len - sizeof(AnglePacket_t)); i++) {
        // 1. 匹配帧头
        if (data[i] == 0xAA) {
            AnglePacket_t* pkg = (AnglePacket_t*)&data[i];

            // 2. 匹配帧尾 (终止检测)
            if (pkg->tail == 0xFF) {
                // 验证通过，直接映射地址
                pAngleData = pkg; 
                return 1; 
            }
        }
    }
    return 0;
}