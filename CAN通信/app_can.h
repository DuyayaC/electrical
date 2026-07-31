#ifndef APP_CAN_H
#define APP_CAN_H

#include "drv_CAN.h"

/* ==================== 方案C：运行时设备配置表 ====================
 * 每个设备一条配置记录（挂载的 CAN 句柄 + 发送 ID），
 * 运行时可改 .hcan 换总线 -> 换板子/改接线无需改代码。
 *
 * 用法：
 *   // 1) 换板时在 main 初始化里重新指定总线
 *   can_dev_yaw.hcan = &hcan1;
 *
 *   // 2) 按设备配置发送（与 CAN_cmd_* 等价，但总线可运行时切换）
 *   CAN_dev_cmd_chassis(&can_dev_chassis, m1, m2, m3, m4);
 *   CAN_dev_cmd_yaw(&can_dev_yaw, val);
 *   CAN_dev_cmd_pitch(&can_dev_pitch, val);
 *   CAN_dev_cmd_shoot(&can_dev_shoot, m1, m2, shoot);
 * ============================================================== */
typedef struct
{
    CAN_HandleTypeDef *hcan;    /* 设备挂载的 CAN 总线句柄 */
    uint32_t           std_id;  /* 发送标准帧 ID */
} can_tx_device_t;

/* 设备配置表实例（默认值见 app_can.c，运行时可重新赋值 .hcan） */
extern can_tx_device_t can_dev_chassis;   /* 底盘 3508 四合一 */
extern can_tx_device_t can_dev_friction;  /* 摩擦轮 */
extern can_tx_device_t can_dev_yaw;       /* yaw 6020 */
extern can_tx_device_t can_dev_pitch;     /* pitch 6020 */
extern can_tx_device_t can_dev_shoot;     /* 拨盘 2006 */

/**
  * @brief          (方案C) 按设备配置发送 8 字节数据（宏实现，内联展开，无函数调用开销）
  * @param[in]      dev:  设备配置指针（决定走哪条 CAN、用什么 ID）
  * @param[in]      data: 8 字节待发送数据
  * @retval         none
  */
#define CAN_dev_send(dev, data)                                                       \
    do {                                                                              \
        CAN_TxHeaderTypeDef tx_msg;                                                   \
        uint32_t send_mail_box;                                                       \
        tx_msg.StdId = (dev)->std_id;                                                 \
        tx_msg.IDE = CAN_ID_STD;                                                      \
        tx_msg.RTR = CAN_RTR_DATA;                                                    \
        tx_msg.DLC = 0x08;                                                            \
        HAL_CAN_AddTxMessage((dev)->hcan, &tx_msg, (data), &send_mail_box);           \
    } while (0)

/**
  * @brief          (方案C) 按设备配置发送底盘四电机电流
  * @param[in]      dev: 设备配置指针, 电流范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_dev_cmd_chassis(can_tx_device_t *dev, int16_t m1, int16_t m2, int16_t m3, int16_t m4);

/**
  * @brief          (方案C) 按设备配置发送 yaw 电流
  * @param[in]      dev: 设备配置指针, 电流范围 [-30000,30000]
  * @retval         none
  */
extern void CAN_dev_cmd_yaw(can_tx_device_t *dev, int16_t yaw);

/**
  * @brief          (方案C) 按设备配置发送 pitch 电流
  * @param[in]      dev: 设备配置指针, 电流范围 [-30000,30000]
  * @retval         none
  */
extern void CAN_dev_cmd_pitch(can_tx_device_t *dev, int16_t pitch);

/**
  * @brief          (方案C) 按设备配置发送摩擦轮 + 拨盘
  * @param[in]      dev: 设备配置指针
  * @retval         none
  */
extern void CAN_dev_cmd_shoot(can_tx_device_t *dev, int16_t m1, int16_t m2, int16_t shoot);

/* ==================== 滤波器配置（复制自 drv_CAN，app_can 独立副本） ====================
 * 与 drv_CAN.c 的 can_filter_init_can1/can2 逻辑相同，只是函数名加了 app_can_ 前缀，
 * 避免两处同名函数造成链接冲突。内部自动完成：配置滤波器 -> 启动总线 -> 开启 FIFO0 中断。
 *
 * 用法：
 *   app_can_filter_init_can1(CAN_FILTER_STD(0x000, 0x000, 0));    // CAN1 全接收, Bank 0
 *   app_can_filter_init_can2(CAN_FILTER_STD(0x201, 0x7E0, 14));   // CAN2 收 0x201~0x205, Bank 14
 * ============================================================================== */

/* 对应的滤波器配置宏（复制自 drv_CAN.h；若 drv_CAN.h 已定义则跳过，避免重复定义） */
#define CAN_FILTER_STD(id, mask, bank)                                                     \
    (CAN_FilterTypeDef)                                                                    \
    {                                                                                      \
        .FilterActivation = ENABLE,                                                        \
        .FilterMode = CAN_FILTERMODE_IDMASK,                                               \
        .FilterScale = CAN_FILTERSCALE_32BIT,                                              \
        .FilterIdHigh = (uint16_t)(((uint32_t)(id) << 5) & 0xFFFF),                        \
        .FilterIdLow = 0,                                                                  \
        .FilterMaskIdHigh = (uint16_t)(((uint32_t)(mask) << 5) & 0xFFFF),                  \
        .FilterMaskIdLow = 0,                                                              \
        .FilterFIFOAssignment = CAN_RX_FIFO0,                                              \
        .FilterBank = (bank),                                                              \
    }

/**
  * @brief          (方案C) 配置并启动 CAN1（滤波器 Bank [0,13]）
  * @param[in]      filter: 滤波器配置，可按值传入 CAN_FILTER_STD(...)
  * @retval         none
  */
extern void app_can_filter_init_can1(CAN_FilterTypeDef filter);

/**
  * @brief          (方案C) 配置并启动 CAN2（滤波器 Bank [14,27]）
  * @param[in]      filter: 滤波器配置，可按值传入 CAN_FILTER_STD(...)
  * @retval         none
  */
extern void app_can_filter_init_can2(CAN_FilterTypeDef filter);

/* ==================== 数据接收（复制自 drv_CAN，改为查表法） ====================
 * 使用 HAL 标准接收回调 HAL_CAN_RxFifo0MsgPendingCallback（收到帧自动触发），
 * 实现在 app_can.c 中。注意 drv_CAN.c 也定义了同名回调，需删除其一避免重复定义。
 * 读取数据：app_can_get_motor_measure(APP_CAN1, slot) 或 APP_CAN2。
 * 槽位映射（slot）：
 *   CAN1: 0=0x201, 1=0x202, 2=0x203(拨盘), 5=0x206(pitch)
 *   CAN2: 0=0x201, 1=0x202, 2=0x203, 3=0x204, 4=0x205(yaw)
 * ============================================================================== */
typedef enum
{
    APP_CAN1 = 0,
    APP_CAN2 = 1,
} app_can_bus_e;

/**
  * @brief          (方案C) 获取电机反馈数据
  * @param[in]      bus:  APP_CAN1 / APP_CAN2
  * @param[in]      slot: 电机槽位，见上方映射
  * @retval         motor_measure_t*，参数无效返回 NULL
  */
extern const motor_measure_t *app_can_get_motor_measure(app_can_bus_e bus, uint8_t slot);

#endif
