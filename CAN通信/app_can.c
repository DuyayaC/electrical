/**
 * @file          app_can.c
 * @brief         (方案C) 运行时设备配置表：设备级 CAN 发送接口
 *
 * 与 drv_CAN.c 的宏定义方案并存、互不影响。
 * 每个设备一条配置记录（挂载的 CAN 句柄 + 发送 ID），运行时可改 .hcan 换总线，
 * 换板子/改接线无需修改 drv_CAN.*，只需在 main 初始化里重新赋值即可。
 *
 * 依赖：drv_CAN.h（提供 can.h/main.h 及电机 ID 枚举）
 */

#include "app_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* ===================== 方案C：运行时设备配置表 =====================
 * 默认值按当前接线设置，运行时可重新赋值 .hcan 实现换总线。
 * ================================================================== */
can_tx_device_t can_dev_chassis  = { &hcan2, CAN_CHASSIS_ALL_ID };
can_tx_device_t can_dev_friction = { &hcan1, CAN_CHASSIS_ALL_ID };
can_tx_device_t can_dev_yaw      = { &hcan2, CAN_GIMBAL_ALL_ID };
can_tx_device_t can_dev_pitch    = { &hcan1, CAN_GIMBAL_ALL_ID };
can_tx_device_t can_dev_shoot    = { &hcan1, CAN_CHASSIS_ALL_ID };

void CAN_dev_cmd_chassis(can_tx_device_t *dev, int16_t m1, int16_t m2, int16_t m3, int16_t m4)
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

void CAN_dev_cmd_yaw(can_tx_device_t *dev, int16_t yaw)
{
  uint8_t data[8] = {0};
  data[0] = yaw >> 8; 
  data[1] = yaw;
  CAN_dev_send(dev, data);
}

void CAN_dev_cmd_pitch(can_tx_device_t *dev, int16_t pitch)
{
  uint8_t data[8] = {0};
  data[2] = pitch >> 8; 
  data[3] = pitch;
  CAN_dev_send(dev, data);
}

void CAN_dev_cmd_shoot(can_tx_device_t *dev, int16_t m1, int16_t m2, int16_t shoot)
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

/* ==================== 滤波器配置（复制自 drv_CAN，app_can 独立副本） ==================== */

void app_can_filter_init_can1(CAN_FilterTypeDef filter)
{
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void app_can_filter_init_can2(CAN_FilterTypeDef filter)
{
    HAL_CAN_ConfigFilter(&hcan2, &filter);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* ==================== 数据接收（复制自 drv_CAN，查表法实现） ====================
 * 使用 HAL 标准接收回调 HAL_CAN_RxFifo0MsgPendingCallback（收到帧自动触发）。
 * 注意：drv_CAN.c 中也定义了同名回调，两处同时编译会「重复定义」报错，
 *       使用时请把 drv_CAN.c 中的那个回调删除/注释掉（按你要求本文件不动 drv_CAN）。
 * ============================================================================== */
static motor_measure_t app_can_motor[2][7];

/* ID(从 0x201 起) -> 电机槽位 查表，0xFF 表示该 ID 无效 */
static const uint8_t app_can_slot_map[2][7] = {
    {0, 1, 2, 0xFF, 0xFF, 5, 0xFF},   /* CAN1: 0x201,0x202,0x203,0x206 */
    {0, 1, 2, 3, 4, 0xFF, 0xFF},      /* CAN2: 0x201~0x205 */
};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  uint8_t bus = (hcan->Instance == CAN1) ? APP_CAN1 : APP_CAN2;
  uint8_t idx;

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  idx = rx_header.StdId - CAN_3508_M1_ID;
  if (idx < 7)
  {
    uint8_t slot = app_can_slot_map[bus][idx];
    if (slot != 0xFF)
      get_motor_measure(&app_can_motor[bus][slot], rx_data);
  }
}

const motor_measure_t *app_can_get_motor_measure(app_can_bus_e bus, uint8_t slot)
{
  if (bus > APP_CAN2 || slot > 6)
    return 0;
  return &app_can_motor[bus][slot];
}
