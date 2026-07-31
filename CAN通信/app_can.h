#ifndef APP_CAN_H
#define APP_CAN_H

#include "can.h"
#include "main.h"

/* ==================== 方案C：运行时设备配置表 ====================
 * 每个设备一条配置记录（挂载的 CAN 句柄 + 发送 ID），
 * 运行时可改 .hcan 换总线 -> 换板子/改接线无需改代码。
 *
 * 用法（按功能顺序）：
 *   // 1) 初始化滤波器（必须先调用，x 为 CAN 通道号）
 *   app_can1_filter_init(CAN_FILTER_STD(0x000, 0x000, 0));    // CAN1 全接收, Bank 0
 *   app_can2_filter_init(CAN_FILTER_STD(0x201, 0x7E0, 14));   // CAN2 收 0x201~0x205, Bank 14
 *
 *   // 2) 读取电机反馈（bus 由用户指定；该设备不在该总线时返回 NULL）
 *   const motor_measure_t *m = get_chassis_motor_measure_point(APP_CAN2, 0);
 *
 *   // 3) 按设备配置发送（总线可运行时切换）
 *   app_can_cmd_chassis(&can_dev_chassis, m1, m2, m3, m4);
 *   app_can_cmd_yaw(&can_dev_yaw, val);
 *   app_can_cmd_pitch(&can_dev_pitch, val);
 *   app_can_cmd_shoot(&can_dev_shoot, m1, m2, shoot);
 * ============================================================== */

/* ==================== 1. 宏定义 ==================== */

/* 电机反馈帧解包宏（把 8 字节原始数据解析到 motor_measure_t） */
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
    (ptr)->error = (data)[7];                                      \
  }

/* 滤波器配置宏（按 id/mask/bank 生成标准帧滤波器配置） */
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

/* 按设备配置发送 8 字节数据（宏实现，内联展开，无函数调用开销） */
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

/* ==================== 2. 类型定义 ==================== */

/* 电机反馈数据结构（存放解包后的 ecd / speed_rpm / given_current 等字段） */
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    uint8_t error;
} motor_measure_t __attribute__((aligned(4)));

/* CAN 发送/接收标准帧 ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x203,
    CAN_GIMBAL_ALL_ID = 0x1FE,

} can_msg_id_e;

/* 设备槽位下标 = 标准帧 ID - 0x201（0x201~0x208 -> 0~7），两条 CAN 线相同，无需查表 */
enum
{
    APP_IDX_M1   = CAN_3508_M1_ID       - CAN_3508_M1_ID, /* 0x201 -> 0 */
    APP_IDX_M2   = CAN_3508_M2_ID       - CAN_3508_M1_ID, /* 0x202 -> 1 */
    APP_IDX_M3   = CAN_3508_M3_ID       - CAN_3508_M1_ID, /* 0x203 -> 2 */
    APP_IDX_M4   = CAN_3508_M4_ID       - CAN_3508_M1_ID, /* 0x204 -> 3 */
    APP_IDX_YAW  = CAN_YAW_MOTOR_ID     - CAN_3508_M1_ID, /* 0x205 -> 4 */
    APP_IDX_PIT  = CAN_PIT_MOTOR_ID     - CAN_3508_M1_ID, /* 0x206 -> 5 */
    APP_IDX_TRIG = CAN_TRIGGER_MOTOR_ID - CAN_3508_M1_ID, /* 0x203 -> 2 */
};

/* CAN 总线枚举 */
typedef enum
{
    APP_CAN1 = 0,
    APP_CAN2 = 1,
} app_can_bus_e;

/* 设备配置记录（挂载的 CAN 总线句柄 + 发送 ID） */
typedef struct
{
    CAN_HandleTypeDef *hcan;    /* 设备挂载的 CAN 总线句柄 */
    uint32_t           std_id;  /* 发送标准帧 ID */
} can_tx_device_t;

/* ==================== 3. 变量声明 ==================== */

/* 设备配置表实例（默认值见 app_can.c，运行时可重新赋值 .hcan） */
extern can_tx_device_t can_dev_chassis;   /* 底盘 3508 四合一 */
extern can_tx_device_t can_dev_friction;  /* 摩擦轮 */
extern can_tx_device_t can_dev_yaw;       /* yaw 6020 */
extern can_tx_device_t can_dev_pitch;     /* pitch 6020 */
extern can_tx_device_t can_dev_shoot;     /* 拨盘 2006 */

/* ==================== 4. 函数声明 ==================== */

/**
  * @brief          (方案C) 配置并启动 CAN1（滤波器 Bank [0,13]）
  * @param[in]      filter: 滤波器配置，可按值传入 CAN_FILTER_STD(...)
  * @retval         none
  */
extern void app_can1_filter_init(CAN_FilterTypeDef filter);

/**
  * @brief          (方案C) 配置并启动 CAN2（滤波器 Bank [14,27]）
  * @param[in]      filter: 滤波器配置，可按值传入 CAN_FILTER_STD(...)
  * @retval         none
  */
extern void app_can2_filter_init(CAN_FilterTypeDef filter);

/**
  * @brief          (方案C) 按设备配置发送底盘四电机电流
  * @param[in]      dev: 设备配置指针, 电流范围 [-16384,16384]
  * @retval         none
  */
extern void app_can_cmd_chassis(can_tx_device_t *dev, int16_t m1, int16_t m2, int16_t m3, int16_t m4);

/**
  * @brief          (方案C) 按设备配置发送 yaw 电流
  * @param[in]      dev: 设备配置指针, 电流范围 [-30000,30000]
  * @retval         none
  */
extern void app_can_cmd_yaw(can_tx_device_t *dev, int16_t yaw);

/**
  * @brief          (方案C) 按设备配置发送 pitch 电流
  * @param[in]      dev: 设备配置指针, 电流范围 [-30000,30000]
  * @retval         none
  */
extern void app_can_cmd_pitch(can_tx_device_t *dev, int16_t pitch);

/**
  * @brief          (方案C) 按设备配置发送摩擦轮 + 拨盘
  * @param[in]      dev:   设备配置指针
  * @param[in]      m1:    摩擦轮电机 1（3508），电流范围 [-16384,16384]
  * @param[in]      m2:    摩擦轮电机 2（3508），电流范围 [-16384,16384]
  * @param[in]      shoot: 拨盘电机；若为 M3508 电流范围 [-16384,16384]，若为 2006 则 [-10000,10000]
  * @retval         none
  */
extern void app_can_cmd_shoot(can_tx_device_t *dev, int16_t m1, int16_t m2, int16_t shoot);

/* ---------- 解包函数（设备取数：返回指定总线上对应设备的电机数据，槽位 = ID - 0x201） ---------- */

/**
  * @brief          return the yaw 6020 motor data point on the given bus
  * @param[in]      bus: APP_CAN1 / APP_CAN2
  * @retval         motor data point（未收到帧时为全 0 数据）
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(app_can_bus_e bus);

/**
  * @brief          return the pitch 6020 motor data point on the given bus
  * @param[in]      bus: APP_CAN1 / APP_CAN2
  * @retval         motor data point（未收到帧时为全 0 数据）
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(app_can_bus_e bus);

/**
  * @brief          return the trigger 2006 motor data point on the given bus
  * @param[in]      bus: APP_CAN1 / APP_CAN2
  * @retval         motor data point（未收到帧时为全 0 数据）
  */
extern const motor_measure_t *get_trigger_motor_measure_point(app_can_bus_e bus);

/**
  * @brief          return the chassis 3508 motor data point on the given bus
  * @param[in]      bus: APP_CAN1 / APP_CAN2
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point（未收到帧时为全 0 数据）
  */
extern const motor_measure_t *get_chassis_motor_measure_point(app_can_bus_e bus, uint8_t i);

/**
  * @brief          return the friction 3508 motor data point on the given bus
  * @param[in]      bus: APP_CAN1 / APP_CAN2
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point（未收到帧时为全 0 数据）
  */
extern const motor_measure_t *get_friction_motor_measure_point(app_can_bus_e bus, uint8_t i);

#endif
