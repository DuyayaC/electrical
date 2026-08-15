/**
 * @file          app_can.c
 * @brief         (方案C) 运行时设备配置表：设备级 CAN 发送接口
 *
 * 自包含模块：仅依赖 STM32 HAL 的 can.h/main.h（含电机 ID 枚举与 motor_measure_t）。
 * 每个设备一条配置记录（挂载的 CAN 句柄 + 发送 ID），运行时可改 .hcan 换总线，
 * 换板子/改接线无需改代码，只需在 main 初始化里重新赋值即可。
 */

#include "app_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* ==================== 1. 变量 ==================== */

/* 电机反馈数据缓存：app_can_motor[总线][槽位] */
static motor_measure_t app_can_motor[2][8];

/* 设备配置表实例（默认值按当前接线设置，运行时可重新赋值 .hcan 换总线） */
can_tx_device_t can_dev_chassis  = { &hcan2, CAN_CHASSIS_ALL_ID };
can_tx_device_t can_dev_friction = { &hcan1, CAN_CHASSIS_ALL_ID };
can_tx_device_t can_dev_yaw      = { &hcan2, CAN_GIMBAL_ALL_ID };
can_tx_device_t can_dev_pitch    = { &hcan1, CAN_GIMBAL_ALL_ID };
can_tx_device_t can_dev_shoot    = { &hcan1, CAN_CHASSIS_ALL_ID };

/* ==================== 2. 中断回调 ====================
 * 使用 HAL 标准接收回调 HAL_CAN_RxFifo0MsgPendingCallback（收到帧自动触发）。
 * ==================================================== */
/**
  * @brief          HAL CAN 接收 FIFO0 中断回调（收到电机反馈帧自动触发）
  * @param[in]      hcan: 触发中断的 CAN 句柄（CAN1 / CAN2）
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  uint8_t bus = (hcan->Instance == CAN1) ? APP_CAN1 : APP_CAN2;
  uint8_t idx;

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  idx = rx_header.StdId - CAN_3508_M1_ID;
  if (idx < 8)
    get_motor_measure(&app_can_motor[bus][idx], rx_data);
}

/* ==================== 3. 初始化：滤波器配置 ====================
 * 内部自动完成：配置滤波器 -> 启动总线 -> 开启 FIFO0 中断。
 * ============================================================ */
/**
  * @brief          (方案C) 配置并启动 CAN1（滤波器 Bank [0,13]）
  * @param[in]      filter: 滤波器配置，可按值传入 CAN_FILTER_STD(...)
  * @retval         none
  */
void app_can1_filter_init(CAN_FilterTypeDef filter)
{
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief          (方案C) 配置并启动 CAN2（滤波器 Bank [14,27]）
  * @param[in]      filter: 滤波器配置，可按值传入 CAN_FILTER_STD(...)
  * @retval         none
  */
void app_can2_filter_init(CAN_FilterTypeDef filter)
{
    HAL_CAN_ConfigFilter(&hcan2, &filter);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* ==================== 4. 功能函数 ==================== */

/* ---------- 解包函数（设备取数：返回指定总线上对应设备的电机数据，槽位 = ID - 0x201） ---------- */

/**
  * @brief          获取指定总线上 yaw 6020 电机的反馈数据
  * @param[in]      bus: APP_CAN1 / APP_CAN2
  * @retval         电机反馈数据指针（未收到帧时为全 0 数据）
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(app_can_bus_e bus)
{
  return &app_can_motor[bus][APP_IDX_YAW];
}

/**
  * @brief          获取指定总线上 pitch 6020 电机的反馈数据
  * @param[in]      bus: APP_CAN1 / APP_CAN2
  * @retval         电机反馈数据指针（未收到帧时为全 0 数据）
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(app_can_bus_e bus)
{
  return &app_can_motor[bus][APP_IDX_PIT];
}

/**
  * @brief          获取指定总线上拨盘 2006 电机的反馈数据
  * @param[in]      bus: APP_CAN1 / APP_CAN2
  * @retval         电机反馈数据指针（未收到帧时为全 0 数据）
  */
const motor_measure_t *get_trigger_motor_measure_point(app_can_bus_e bus)
{
  return &app_can_motor[bus][APP_IDX_TRIG];
}

/**
  * @brief          获取指定总线上底盘 3508 电机（0x201+i）的反馈数据
  * @param[in]      bus: APP_CAN1 / APP_CAN2
  * @param[in]      i:   电机号，范围 [0,3]
  * @retval         电机反馈数据指针（未收到帧时为全 0 数据）
  */
const motor_measure_t *get_chassis_motor_measure_point(app_can_bus_e bus, uint8_t i)
{
  return &app_can_motor[bus][APP_IDX_M1 + (i & 0x03)];
}

/**
  * @brief          获取指定总线上摩擦轮 3508 电机（0x201+i）的反馈数据
  * @param[in]      bus: APP_CAN1 / APP_CAN2
  * @param[in]      i:   电机号，范围 [0,3]
  * @retval         电机反馈数据指针（未收到帧时为全 0 数据）
  */
const motor_measure_t *get_friction_motor_measure_point(app_can_bus_e bus, uint8_t i)
{
  return &app_can_motor[bus][APP_IDX_M1 + (i & 0x03)];
}

/**
  * @brief          (方案C) 按设备配置发送底盘四电机电流
  * @param[in]      dev: 设备配置指针, 电流范围 [-16384,16384]
  * @retval         none
  */
void app_can_cmd_chassis(can_tx_device_t *dev, int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
  uint8_t data[8];
  data[0] = m1 >> 8;
  data[1] = m1;
  data[2] = m2 >> 8;
  data[3] = m2;
  data[4] = m3 >> 8;
  data[5] = m3;
  data[6] = m4 >> 8;
  data[7] = m4;
  CAN_dev_send(dev, data);
}

/**
  * @brief          (方案C) 按设备配置发送 yaw 电流
  * @param[in]      dev: 设备配置指针, 电流范围 [-16384,16384]
  * @retval         none
  */
void app_can_cmd_yaw(can_tx_device_t *dev, int16_t yaw)
{
  uint8_t data[8] = {0};
  data[0] = yaw >> 8;
  data[1] = yaw;
  CAN_dev_send(dev, data);
}

/**
  * @brief          (方案C) 按设备配置发送 pitch 电流
  * @param[in]      dev: 设备配置指针, 电流范围 [-16384,16384]
  * @retval         none
  */
void app_can_cmd_pitch(can_tx_device_t *dev, int16_t pitch)
{
  uint8_t data[8] = {0};
  data[2] = pitch >> 8;
  data[3] = pitch;
  CAN_dev_send(dev, data);
}

/**
  * @brief          (方案C) 按设备配置发送摩擦轮 + 拨盘
  * @param[in]      dev:   设备配置指针
  * @param[in]      m1:    摩擦轮电机 1（3508），电流范围 [-16384,16384]
  * @param[in]      m2:    摩擦轮电机 2（3508），电流范围 [-16384,16384]
  * @param[in]      shoot: 拨盘电机；若为 M3508 电流范围 [-16384,16384]，若为 2006 则 [-10000,10000]
  * @retval         none
  */
void app_can_cmd_shoot(can_tx_device_t *dev, int16_t m1, int16_t m2, int16_t shoot)
{
  uint8_t data[8] = {0};
  data[0] = m1 >> 8;
  data[1] = m1;
  data[2] = m2 >> 8;
  data[3] = m2;
  data[4] = shoot >> 8;
  data[5] = shoot;
  CAN_dev_send(dev, data);
}
