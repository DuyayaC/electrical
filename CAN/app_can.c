/**
 * @file          app_can.c
 * @brief         (方案C) 运行时设备配置表：设备级 CAN 发送接口
 *
 * 自包含模块：仅依赖 STM32 HAL 的 can.h/main.h（含电机 ID 枚举与 motor_measure_t）。
 * 每个设备一条配置记录（挂载的 CAN 句柄 + 发送 ID），运行时可改 .hcan 换总线，
 * 换板子/改接线无需改代码，只需在 main 初始化里重新赋值即可。
 */

#include "app_can.h"

/* ==================== CAN 滤波配置使用说明 ====================
 * 滤波器配置结构体（CAN_FilterTypeDef）由调用方（如 main）在外面创建，
 * 再通过指针传给 can_filter_std() 在函数内填充字段，最后传入初始化函数。
 *
 * 完整流程（CAN1 全接收，Bank 0）：
 *   CAN_FilterTypeDef filter_can1;                          // 1) 在外面创建结构体
 *   can_filter_std(&filter_can1, 0x000, 0x000, 0);          // 2) 函数内填充配置字段
 *   app_can1_filter_init(&filter_can1);                     // 3) 传入配置并启动总线
 *
 * can_filter_std(filter, id, mask, bank) 参数说明：
 *   filter : 外部创建的结构体指针（函数内填充，不创建）
 *   id     : 期望接收的标准帧 ID（如 0x201），与掩码配合决定收哪些帧
 *   mask   : 掩码，位为 0 表示"必须精确匹配该位"，位为 1 表示"该位不关心"
 *   bank   : 滤波器组编号，STM32F4 共 28 个 Bank（0~27）
 *            CAN1 占用 Bank [0,13]，CAN2 占用 Bank [14,27]，互不重叠
 *
 * 匹配规则（IDMASK 模式）：(接收帧ID & ~mask) == (id & ~mask)
 *   即掩码中为 0 的位必须与 id 一致，为 1 的位任意。
 *
 * 常用写法（只需改 id / mask 两个参数）：
 *   // 只收 0x201 一帧（mask = 0x7FF 全部精确匹配）
 *   can_filter_std(&f, 0x201, 0x7FF, 0)
 *
 *   // 收 0x200 ~ 0x207 共 8 帧（mask = 0x7F8 忽略低 3 位）
 *   can_filter_std(&f, 0x200, 0x7F8, 0)
 *
 *   // 全接收（mask = 0x000 所有位都不关心）
 *   can_filter_std(&f, 0x000, 0x000, 0)
 *
 * 注意：
 *   1. 两条 CAN 的 Bank 区间固定，CAN1 用 Bank 0~13，CAN2 用 Bank 14~27；
 *   2. 一个滤波器组只能用一条总线，若总线滤波器配置重复会覆盖前者；
 *   3. 本库约定电机反馈帧 ID 为 0x201~0x208，配置时可参考上面的"收 8 帧"写法。
 * ============================================================== */

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
  * @brief          解包电机反馈帧（把 8 字节原始数据解析到 motor_measure_t）
  * @param[in]      ptr:  电机数据缓存指针
  * @param[in]      data: CAN 接收到的 8 字节原始数据
  * @retval         none
  */
void get_motor_measure(motor_measure_t *ptr, const uint8_t *data)
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd = (uint16_t)(data[0] << 8 | data[1]);
  ptr->speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
  ptr->given_current = (uint16_t)(data[4] << 8 | data[5]);
  ptr->temperate = data[6];
  ptr->error = data[7];
}

/**
  * @brief          HAL CAN 接收 FIFO0 中断回调（收到电机反馈帧自动触发）
  * @param[in]      hcan: 触发中断的 CAN 句柄（CAN1 / CAN2）
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  uint8_t bus;
  uint8_t idx;

  /* 判断触发中断的是 CAN1 还是 CAN2，用于选择对应的反馈缓存区 */
  if (hcan->Instance == CAN1)
    bus = APP_CAN1;
  else
    bus = APP_CAN2;

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  idx = rx_header.StdId - CAN_3508_M1_ID;
  if (idx < 8)
    get_motor_measure(&app_can_motor[bus][idx], rx_data);
}

/* ==================== 3. 初始化：滤波器配置 ====================
 * 内部自动完成：配置滤波器 -> 启动总线 -> 开启 FIFO0 中断。
 * 滤波器配置结构体由调用方创建，can_filter_std() 填充，见文件头说明。
 * ============================================================ */
/**
  * @brief          生成标准帧滤波器配置（填充外部创建的结构体，不创建）
  * @param[out]     filter: 外部创建的滤波器配置结构体指针
  * @param[in]      id:    期望接收的标准帧 ID（如 0x201）
  * @param[in]      mask:  掩码，位为 0 必须精确匹配，位为 1 不关心
  * @param[in]      bank:  滤波器组编号（CAN1: 0~13, CAN2: 14~27）
  * @retval         none
  */
void can_filter_std(CAN_FilterTypeDef *filter, uint32_t id, uint32_t mask, uint32_t bank)
{
  filter->FilterActivation = ENABLE;
  filter->FilterMode = CAN_FILTERMODE_IDMASK;
  filter->FilterScale = CAN_FILTERSCALE_32BIT;
  filter->FilterIdHigh = (uint16_t)((id << 5) & 0xFFFF);
  filter->FilterIdLow = 0;
  filter->FilterMaskIdHigh = (uint16_t)((mask << 5) & 0xFFFF);
  filter->FilterMaskIdLow = 0;
  filter->FilterFIFOAssignment = CAN_RX_FIFO0;
  filter->FilterBank = bank;
}

/**
  * @brief          (方案C) 配置并启动 CAN1（滤波器 Bank [0,13]）
  * @param[in]      filter: 外部创建并填充好的滤波器配置指针
  * @retval         none
  */
void app_can1_filter_init(CAN_FilterTypeDef *filter)
{
    HAL_CAN_ConfigFilter(&hcan1, filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief          (方案C) 配置并启动 CAN2（滤波器 Bank [14,27]）
  * @param[in]      filter: 外部创建并填充好的滤波器配置指针
  * @retval         none
  */
void app_can2_filter_init(CAN_FilterTypeDef *filter)
{
    HAL_CAN_ConfigFilter(&hcan2, filter);
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

/* ---------- 发送函数（按设备配置发送 8 字节数据，供 app_can_cmd_* 调用） ---------- */

/**
  * @brief          按设备配置发送 8 字节数据
  * @param[in]      dev:  设备配置指针
  * @param[in]      data: 待发送的 8 字节数据
  * @retval         none
  */
static void can_dev_send(can_tx_device_t *dev, uint8_t *data)
{
  CAN_TxHeaderTypeDef tx_msg;
  uint32_t send_mail_box;

  tx_msg.StdId = dev->std_id;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;
  HAL_CAN_AddTxMessage(dev->hcan, &tx_msg, data, &send_mail_box);
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
  can_dev_send(dev, data);
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
  can_dev_send(dev, data);
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
  can_dev_send(dev, data);
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
  can_dev_send(dev, data);
}
