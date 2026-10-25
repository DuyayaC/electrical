/**
 * @file          app_can.c
 * @brief         (dm 版 / STM32H7 FDCAN) 运行时设备配置表：设备级 CAN 发送接口
 *
 * 与 dji/app_can.c 是同一套接口的两个平台版本：对外函数、全局变量、数据结构
 * 完全一致，工程里直接替换 dji 版的两个文件即可，调用代码（main.c 等）不用改。
 * 本文件内部把经典 bxCAN 的 HAL 接口全部换成 FDCAN 的 HAL 接口：
 *
 *   功能          dji 版（bxCAN）                                        dm 版（FDCAN）
 *   -----------   ---------------------------------------------------    ----------------------------------------------------------
 *   配置滤波器    HAL_CAN_ConfigFilter()                                 HAL_FDCAN_ConfigFilter()
 *   全局滤波      —（bxCAN 无此概念）                                    HAL_FDCAN_ConfigGlobalFilter()
 *   启动总线      HAL_CAN_Start()                                        HAL_FDCAN_Start()
 *   开接收中断    HAL_CAN_ActivateNotification(hcan,                      HAL_FDCAN_ActivateNotification(hfdcan,
 *                   CAN_IT_RX_FIFO0_MSG_PENDING)                            FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0)
 *   接收回调      HAL_CAN_RxFifo0MsgPendingCallback(hcan)                HAL_FDCAN_RxFifo0Callback(hfdcan, RxFifo0ITs)
 *   取接收帧      HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,               HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
 *                   &rx_header, rx_data)                                    &rx_header, rx_data)
 *   发送帧        HAL_CAN_AddTxMessage(hcan, &tx_msg,                    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan,
 *                   data, &send_mail_box)                                   &tx_header, data)
 *   接收帧头      CAN_RxHeaderTypeDef（StdId / DLC）                     FDCAN_RxHeaderTypeDef（Identifier / IdType / DataLength）
 *   发送帧头      CAN_TxHeaderTypeDef（StdId / IDE / RTR / DLC）         FDCAN_TxHeaderTypeDef（Identifier / IdType / TxFrameType /
 *                                                                         DataLength / ErrorStateIndicator / BitRateSwitch /
 *                                                                         FDFormat / TxEventFifoControl / MessageMarker）
 *   句柄类型      CAN_HandleTypeDef                                      FDCAN_HandleTypeDef
 *   滤波器类型    CAN_FilterTypeDef                                      FDCAN_FilterTypeDef
 *
 * 自包含模块：仅依赖 STM32 HAL 的 fdcan.h/main.h（含电机 ID 枚举与 motor_measure_t）。
 * 每个设备一条配置记录（挂载的 FDCAN 句柄 + 发送 ID），运行时可改 .hcan 换总线，
 * 换板子/改接线无需改代码，只需在 main 初始化里重新赋值即可。
 *
 * DM（达妙）电机：在 dm 版里额外支持达妙 DM 电机的 MIT / 位置速度模式控制帧
 * 以及反馈帧解析（协议见 app_can.h 第 2 节），对应函数 app_dm_*；DJI 相关接口不受影响。
 *
 * 移植注意（工程集成）：
 *   1. 本文件实现了 HAL 的弱回调 HAL_FDCAN_RxFifo0Callback()。达妙例程
 *      User/bsp_fdcan.c 里也有一个同名强符号，两者只能保留一个（或合并成一个），
 *      否则链接会报重复定义；本模块只接管 FDCAN1 / FDCAN2，工程若还要用 FDCAN3
 *      接收，需要把 FDCAN3 的处理合并进本回调（见回调函数注释）。
 *   2. 调用顺序：先 MX_FDCAN1_Init() / MX_FDCAN2_Init()（内部完成 HAL_FDCAN_Init），
 *      再调用 app_can1_filter_init() / app_can2_filter_init()；本函数内部会
 *      HAL_FDCAN_Start() 启动总线，不要再调用达妙 bsp_can_init() 重复启动。
 */

#include "app_can.h"

/* ==================== FDCAN 滤波配置使用说明 ====================
 * dji 版用 bxCAN 的 28 个滤波器组（CAN1 用 Bank 0~13，CAN2 用 Bank 14~27）；
 * FDCAN 没有"两条总线共用的 Bank"：FDCAN1、FDCAN2 每个实例各自维护一份
 * 标准 ID 滤波器列表，列表条数由 CubeMX 里的 "Std Filters Nbr" 决定，
 * C 代码里就是 hfdcan1.Init.StdFiltersNbr / hfdcan2.Init.StdFiltersNbr。
 *
 * 因此 can_filter_std(filter, id, mask, bank) 的 bank 在 dm 版里表示
 * "该实例自己的滤波器序号"，取值范围 0 ~ StdFiltersNbr-1。
 * 为了让 dji 工程的调用代码（CAN2 写 14~27）原样跑起来：
 *   app_can1_filter_init()：bank 原样使用（0~13）
 *   app_can2_filter_init()：bank >= 14 时先减 14（14~27 -> 0~13）
 * 折算后仍 >= StdFiltersNbr 的序号直接放弃配置（FDCAN 的滤波器写在消息 RAM 里，
 * 越界会踩坏后面的接收/发送缓冲区，比"收不到帧"危害大得多）。
 *
 * 滤波器配置结构体（FDCAN_FilterTypeDef）由调用方
 * （如 main）在外面创建，再通过指针传给 can_filter_std() 在函数内填充字段，
 * 最后传入初始化函数。
 *
 * 完整流程（FDCAN1 全接收，序号 0）：
 *   FDCAN_FilterTypeDef filter_can1;                        // 1) 在外面创建结构体
 *   can_filter_std(&filter_can1, 0x000, 0x000, 0);          // 2) 函数内填充配置字段
 *   app_can1_filter_init(&filter_can1);                     // 3) 传入配置并启动总线
 *
 * can_filter_std(filter, id, mask, bank) 参数说明：
 *   filter : 外部创建的结构体指针（函数内填充，不创建）
 *   id     : 期望接收的标准帧 ID（如 0x201），与掩码配合决定收哪些帧
 *   mask   : 掩码，位为 1 表示"必须匹配该位"，位为 0 表示"该位不关心"
 *   bank   : FDCAN 实例内的滤波器序号（折算规则见上面）
 *
 * 匹配规则（FDCAN_FILTER_MASK）：
 *   (接收帧ID & mask) == (id & mask)
 *   即掩码中为 1 的位必须与 id 一致，为 0 的位任意。
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
 * CubeMX / 工程侧必须满足（否则收不到电机反馈）：
 *   1. FDCANx -> "Std Filters Nbr" >= 1（DM-MC02 例程为 4），否则没有滤波器
 *      列表可写，HAL_FDCAN_ConfigFilter() 会写到消息 RAM 的非滤波器区；
 *   2. "Rx FIFO0 Elmt Nbr" >= 1（DM-MC02 例程为 8），否则取帧函数直接报错；
 *   3. "Tx FIFO Queue Elmt Nbr" >= 1（DM-MC02 例程为 8），否则发不出帧；
 *   4. 打开 FDCANx 的中断线 0（FDCANx_IT0_IRQn），接收中断才能进回调；
 *   5. 若 CubeMX 把 FDCAN 配置成 FD 模式（FDCAN_FRAME_FD_BRS），本模块仍然按
 *      经典 CAN 帧收发（DJI 电机只认经典帧），不需要改 CubeMX 配置。
 * ============================================================== */

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

/* ==================== 1. 变量 ==================== */

/* 电机反馈数据缓存：app_can_motor[总线][槽位] */
static motor_measure_t app_can_motor[2][8];

/* 设备配置表实例（默认值按当前接线设置，运行时可重新赋值 .hcan 换总线；
 * .hcan 类型是 FDCAN_HandleTypeDef *，直接填 &hfdcanN） */
can_tx_device_t can_dev_chassis  = { &hfdcan2, CAN_CHASSIS_ALL_ID };
can_tx_device_t can_dev_friction = { &hfdcan1, CAN_CHASSIS_ALL_ID };
can_tx_device_t can_dev_yaw      = { &hfdcan2, CAN_GIMBAL_ALL_ID };
can_tx_device_t can_dev_pitch    = { &hfdcan1, CAN_GIMBAL_ALL_ID };
can_tx_device_t can_dev_shoot    = { &hfdcan1, CAN_CHASSIS_ALL_ID };

/* DM 电机槽位表（can_id = 0 表示该槽位未使用；用 app_dm_motor_init() 初始化） */
dm_motor_t app_dm_motor[APP_DM_MOTOR_NUM];

/* ==================== 2. DM 电机（达妙）协议 ====================
 * 协议格式见 app_can.h 第 2 节（经典 CAN、标准帧 8 字节）。
 * CAN ID 超出 11 位标准帧范围（>0x7FF）时直接不下发。
 * 反馈帧由接收回调按 Master ID / D[0] 低 4 位分发到 app_dm_motor[] 对应槽位。
 * ============================================================ */

/* ---------- 内部工具 ---------- */
/**
  * @brief          float -> 无符号定点（线性映射，超出范围时先限幅）
  *                 用于 Kp / Kd（0~500 / 0~5）
  * @param[in]      x:     待转换的浮点值
  * @param[in]      x_min: 范围最小值
  * @param[in]      x_max: 范围最大值
  * @param[in]      bits:  目标位数（12）
  * @retval         无符号定点值（0 ~ 2^bits-1）
  */
static uint16_t app_dm_float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
  float span;
  uint32_t max_int;

  /* 本协议只用 16/12 位；其它位数不支持（bits >= 32 左移是未定义行为） */
  if ((bits == 0U) || (bits > 16U))
    return 0U;

  /* 限幅；NaN 与任何数比较都为假，这里会落到 x_min，避免非法转换 */
  if (!(x >= x_min))
    x = x_min;
  else if (x > x_max)
    x = x_max;

  span = x_max - x_min;
  if (span <= 0.0f)
    return 0U;

  max_int = ((uint32_t)1U << bits) - 1U;
  return (uint16_t)((x - x_min) * (float)max_int / span);
}

/**
  * @brief          float -> 有符号定点补码（0 对应 0，满量程 ±(2^(bits-1)-1)）
  *                 用于 8009P 的 Position(16) / Velocity(12) / Torque(12)：
  *                 范围对称，超范围先限幅，负值直接得到负码
  * @param[in]      x:     待转换的浮点值
  * @param[in]      range: 满量程（正值，如 P_MAX / V_MAX / T_MAX）
  * @param[in]      bits:  目标位数（16 / 12）
  * @retval         有符号定点值（-((2^(bits-1))-1) ~ +((2^(bits-1))-1)）
  */
static int16_t app_dm_float_to_sint(float x, float range, uint8_t bits)
{
  float max_code;

  /* 本协议只用 16/12 位；其它位数不支持（bits >= 32 左移是未定义行为） */
  if ((bits == 0U) || (bits > 16U))
    return 0;

  if (range < 0.0f)
    range = -range;
  if (!(range > 0.0f))      /* 范围非法（含 NaN）时无法映射 */
    return 0;

  if (x != x)               /* NaN -> 0 */
    x = 0.0f;
  else if (x > range)
    x = range;
  else if (x < -range)
    x = -range;

  max_code = (float)((1L << (bits - 1U)) - 1L);
  return (int16_t)(x / range * max_code);
}

/**
  * @brief          有符号定点补码 -> float（0 对应 0，满量程 ±(2^(bits-1)-1)）
  * @param[in]      x:     有符号定点值（已做符号扩展）
  * @param[in]      range: 满量程（正值，如 P_MAX / V_MAX / T_MAX）
  * @param[in]      bits:  原始位数（16 / 12）
  * @retval         浮点值
  */
static float app_dm_sint_to_float(int16_t x, float range, uint8_t bits)
{
  float max_code;

  if ((bits == 0U) || (bits > 16U))
    return 0.0f;

  if (range < 0.0f)
    range = -range;

  max_code = (float)((1L << (bits - 1U)) - 1L);
  if (max_code <= 0.0f)
    return 0.0f;

  return (float)x / max_code * range;
}

/**
  * @brief          把 float 按协议（低位在前）写进 4 字节
  * @param[out]     dst:   目标缓冲（4 字节）
  * @param[in]      value: 待写入的浮点数
  * @retval         none
  */
static void app_dm_put_float_le(uint8_t *dst, float value)
{
  union
  {
    float   f;
    uint8_t b[4];
  } u;

  u.f = value;
  dst[0] = u.b[0];
  dst[1] = u.b[1];
  dst[2] = u.b[2];
  dst[3] = u.b[3];
}

/**
  * @brief          发送一帧 DM 数据（内部函数）
  * @param[in]      motor: 电机配置（用 motor->hfdcan 发送）
  * @param[in]      id:    发送用的标准帧 ID
  * @param[in]      data:  8 字节数据
  * @retval         none
  */
static void app_dm_send(dm_motor_t *motor, uint16_t id, const uint8_t *data)
{
  FDCAN_TxHeaderTypeDef tx_header;

  /* 标准帧 ID 只有 11 位；CAN ID 超出范围时不下发 */
  if (id > 0x7FFU)
    return;

  tx_header.Identifier = id;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_BYTES_8;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  HAL_FDCAN_AddMessageToTxFifoQ(motor->hfdcan, &tx_header, data);
}

/**
  * @brief          解包 DM 反馈帧（内部函数，接收回调用）
  * @param[in,out]  motor: 电机配置（反馈写进 motor->measure）
  * @param[in]      data:  反馈帧 8 字节数据
  * @retval         none
  */
static void app_dm_decode(dm_motor_t *motor, const uint8_t *data)
{
  dm_motor_measure_t *m = &motor->measure;
  uint8_t id_err = data[0];
  uint16_t pos_bits;
  uint16_t vel_bits;
  uint16_t tor_bits;
  int16_t pos_code;
  int16_t vel_code;
  int16_t tor_code;

  /* 和 dji 版解包一样：D[0] 低 4 位是控制器 ID、高 4 位就是原始故障码，
   * 直接赋值（包里是 0xA/0xB 就保留 0xA/0xB），不做任何字符串/含义转换 */
  m->id = (uint8_t)(id_err & 0x0FU);
  m->error = (uint8_t)(id_err >> 4);

  pos_bits = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
  vel_bits = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
  tor_bits = (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8) | data[5]);

  m->pos_int = pos_bits;
  m->vel_int = vel_bits;
  m->tor_int = tor_bits;

  /* 8009P：位置/速度/扭矩都是有符号补码定点（16/12/12 位），
   * 负值要先把 12 位补码符号扩展到 16 位，再按 0 对应 0 的满量程换算 */
  pos_code = (int16_t)pos_bits;
  vel_code = (int16_t)(((vel_bits & 0x0800U) != 0U) ? (vel_bits | 0xF000U) : vel_bits);
  tor_code = (int16_t)(((tor_bits & 0x0800U) != 0U) ? (tor_bits | 0xF000U) : tor_bits);

  m->pos = app_dm_sint_to_float(pos_code, motor->p_max, 16U);
  m->vel = app_dm_sint_to_float(vel_code, motor->v_max, 12U);
  m->tor = app_dm_sint_to_float(tor_code, motor->t_max, 12U);

  m->t_mos = (float)data[6];
  m->t_rotor = (float)data[7];
  m->updated = 1U;
}

/* ---------- 对外接口 ---------- */
/**
  * @brief          初始化一台 DM 电机（填默认映射范围、绑定总线与 ID）
  * @param[in,out]  motor:  电机配置指针
  * @param[in]      hfdcan: 挂载的 FDCAN 句柄
  * @param[in]      can_id: 电机 CAN ID（0 = 不使用该槽位）
  * @param[in]      mst_id: 反馈帧 ID（Master ID，默认 0）
  * @retval         none
  */
void app_dm_motor_init(dm_motor_t *motor, FDCAN_HandleTypeDef *hfdcan,
                       uint16_t can_id, uint16_t mst_id)
{
  dm_motor_measure_t *m;

  if (motor == NULL)
    return;

  motor->hfdcan = hfdcan;
  /* 标准帧 ID 只有 11 位：非法 CAN ID 直接当作未使用槽位
   * （避免 uint16 回绕成一个"合法"但错误的 ID） */
  motor->can_id = (can_id <= 0x7FFU) ? can_id : 0U;
  motor->mst_id = mst_id;

  motor->p_min = APP_DM_DEFAULT_P_MIN;
  motor->p_max = APP_DM_DEFAULT_P_MAX;
  motor->v_min = APP_DM_DEFAULT_V_MIN;
  motor->v_max = APP_DM_DEFAULT_V_MAX;
  motor->t_min = APP_DM_DEFAULT_T_MIN;
  motor->t_max = APP_DM_DEFAULT_T_MAX;
  motor->kp_min = APP_DM_DEFAULT_KP_MIN;
  motor->kp_max = APP_DM_DEFAULT_KP_MAX;
  motor->kd_min = APP_DM_DEFAULT_KD_MIN;
  motor->kd_max = APP_DM_DEFAULT_KD_MAX;

  m = &motor->measure;
  m->pos = 0.0f;
  m->vel = 0.0f;
  m->tor = 0.0f;
  m->t_mos = 0.0f;
  m->t_rotor = 0.0f;
  m->pos_int = 0U;
  m->vel_int = 0U;
  m->tor_int = 0U;
  m->id = 0U;
  m->error = 0U;
  m->updated = 0U;
}

/**
  * @brief          使能 DM 电机（数据 0xFF...FF,0xFC，帧 ID = CAN ID + mode）
  * @param[in]      motor: 电机配置指针
  * @param[in]      mode:  目标模式 APP_DM_MODE_MIT / APP_DM_MODE_POS_VEL
  * @retval         none
  */
void app_dm_enable(dm_motor_t *motor, uint16_t mode)
{
  uint8_t data[8];
  uint8_t i;

  if ((motor == NULL) || (motor->hfdcan == NULL) || (motor->can_id == 0U))
    return;

  for (i = 0U; i < 7U; i++)
    data[i] = 0xFFU;
  data[7] = 0xFCU;

  app_dm_send(motor, (uint16_t)(motor->can_id + mode), data);
}

/**
  * @brief          失能 DM 电机（数据 0xFF...FF,0xFD，帧 ID = CAN ID + mode）
  * @param[in]      motor: 电机配置指针
  * @param[in]      mode:  目标模式 APP_DM_MODE_MIT / APP_DM_MODE_POS_VEL
  * @retval         none
  */
void app_dm_disable(dm_motor_t *motor, uint16_t mode)
{
  uint8_t data[8];
  uint8_t i;

  if ((motor == NULL) || (motor->hfdcan == NULL) || (motor->can_id == 0U))
    return;

  for (i = 0U; i < 7U; i++)
    data[i] = 0xFFU;
  data[7] = 0xFDU;

  app_dm_send(motor, (uint16_t)(motor->can_id + mode), data);
}

/**
  * @brief          MIT 模式控制（帧 ID = CAN ID）
  * @param[in]      motor: 电机配置指针
  * @param[in]      p_des: 位置给定（rad）
  * @param[in]      v_des: 速度给定（rad/s）
  * @param[in]      kp:    位置比例系数
  * @param[in]      kd:    位置微分系数
  * @param[in]      t_ff:  转矩前馈（N·m）
  * @retval         none
  */
void app_dm_cmd_mit(dm_motor_t *motor, float p_des, float v_des, float kp, float kd, float t_ff)
{
  uint8_t  data[8];
  int16_t  p_int;
  int16_t  v_int;
  int16_t  t_int;
  uint16_t kp_int;
  uint16_t kd_int;
  uint16_t p_bits;
  uint16_t v_bits;
  uint16_t t_bits;

  if ((motor == NULL) || (motor->hfdcan == NULL) || (motor->can_id == 0U))
    return;

  /* 8009P：位置/速度/扭矩有符号补码（0 对应 0，满量程 ±P_MAX/±V_MAX/±T_MAX），
   * 负值直接得到负码；Kp/Kd 仍为无符号 12 位（0~500 / 0~5）。 */
  p_int = app_dm_float_to_sint(p_des, motor->p_max, 16U);
  v_int = app_dm_float_to_sint(v_des, motor->v_max, 12U);
  t_int = app_dm_float_to_sint(t_ff, motor->t_max, 12U);
  kp_int = app_dm_float_to_uint(kp, motor->kp_min, motor->kp_max, 12U);
  kd_int = app_dm_float_to_uint(kd, motor->kd_min, motor->kd_max, 12U);

  p_bits = (uint16_t)p_int;
  v_bits = (uint16_t)v_int & 0x0FFFU;
  t_bits = (uint16_t)t_int & 0x0FFFU;

  data[0] = (uint8_t)(p_bits >> 8);
  data[1] = (uint8_t)(p_bits & 0xFFU);
  data[2] = (uint8_t)(v_bits >> 4);
  data[3] = (uint8_t)(((v_bits & 0x0FU) << 4) | (kp_int >> 8));
  data[4] = (uint8_t)(kp_int & 0xFFU);
  data[5] = (uint8_t)(kd_int >> 4);
  data[6] = (uint8_t)(((kd_int & 0x0FU) << 4) | (t_bits >> 8));
  data[7] = (uint8_t)(t_bits & 0xFFU);

  app_dm_send(motor, motor->can_id, data);
}

/**
  * @brief          位置速度模式控制（帧 ID = 0x100 + CAN ID）
  * @param[in]      motor: 电机配置指针
  * @param[in]      p_des: 位置给定（rad）
  * @param[in]      v_des: 速度给定（rad/s）
  * @retval         none
  */
void app_dm_cmd_pos_vel(dm_motor_t *motor, float p_des, float v_des)
{
  uint8_t data[8];

  if ((motor == NULL) || (motor->hfdcan == NULL) || (motor->can_id == 0U))
    return;

  app_dm_put_float_le(&data[0], p_des);
  app_dm_put_float_le(&data[4], v_des);

  app_dm_send(motor, (uint16_t)(motor->can_id + APP_DM_MODE_POS_VEL), data);
}

/**
  * @brief          读取 DM 电机反馈数据（只读指针）
  * @param[in]      motor: 电机配置指针
  * @retval         反馈数据指针；motor 为 NULL 时返回 NULL
  */
const dm_motor_measure_t *app_dm_get_measure(const dm_motor_t *motor)
{
  return (motor == NULL) ? NULL : &motor->measure;
}

/* ==================== 3. 中断回调 ====================
 * 使用 HAL 标准接收回调 HAL_FDCAN_RxFifo0Callback（收到帧自动触发）。
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
  * @brief          HAL FDCAN 接收 FIFO0 中断回调（收到电机反馈帧自动触发）
  * @param[in]      hfdcan:     触发中断的 FDCAN 句柄（FDCAN1 / FDCAN2）
  * @param[in]      RxFifo0ITs: FIFO0 中断源标志
  * @retval         none
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rx_header;
  /* HAL 按 DLC 拷贝数据，FD 帧最多 64 字节，缓冲区必须按最大值开，
   * 否则总线上一旦出现 FD 帧就会写穿栈（本模块只解包 8 字节帧） */
  uint8_t rx_data[64];
  uint8_t bus;
  uint8_t index;
  uint8_t i;
  uint8_t match;
  dm_motor_t *dm;

  /* 只处理"Rx FIFO0 收到新帧"这一中断源 */
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    return;

  /* 判断触发中断的是 FDCAN1 还是 FDCAN2，用于选择对应的反馈缓存区 */
  if (hfdcan->Instance == FDCAN1)
    bus = APP_CAN1;
  else if (hfdcan->Instance == FDCAN2)
    bus = APP_CAN2;
  else
    return;  /* 本模块只接管 FDCAN1 / FDCAN2；工程若还用 FDCAN3 收别的设备，
                需要在这个回调里自行补分支（或改用 HAL 回调注册函数） */

  /* 把 FIFO0 里已经排队的帧一次读空：
   * H7 FDCAN 的 RF0N 是"又写入一帧"的事件标志（不像 F4 bxCAN 的 FMP0 是电平），
   * 每次中断只读 1 帧的话，中断被延迟期间攒下的帧可能一直排不空。 */
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
      break;

    /* 只处理标准帧、数据帧、8 字节负载（DJI 用经典帧；DM 电机经典/FD 帧都可以） */
    if (rx_header.IdType != FDCAN_STANDARD_ID)
      continue;
    if (rx_header.RxFrameType != FDCAN_DATA_FRAME)
      continue;
    if (rx_header.DataLength != FDCAN_DLC_BYTES_8)
      continue;

    /* 1) DJI 电机反馈：经典帧 + ID 0x201~0x208（槽位 = ID - 0x201）。
     *    先做范围判断再相减，避免小 ID 减出下溢后被截断成 0~7 误写电机槽。 */
    if ((rx_header.FDFormat == FDCAN_CLASSIC_CAN) &&
        (rx_header.Identifier >= CAN_3508_M1_ID) &&
        (rx_header.Identifier <= (CAN_3508_M1_ID + 7U)))
    {
      index = (uint8_t)(rx_header.Identifier - CAN_3508_M1_ID);
      get_motor_measure(&app_can_motor[bus][index], rx_data);
      continue;
    }

    /* 2) DM（达妙）电机反馈：帧 ID = Master ID（MST_ID）；mst_id == 0 时
     *    按 D[0] 低 4 位 == CAN ID 低 4 位匹配（默认 Master ID = 0 的用法）。 */
    for (i = 0U; i < APP_DM_MOTOR_NUM; i++)
    {
      dm = &app_dm_motor[i];
      if ((dm->can_id == 0U) || (dm->hfdcan != hfdcan))
        continue;

      match = 0U;
      if (dm->mst_id != 0U)
      {
        if (dm->mst_id == rx_header.Identifier)
          match = 1U;
      }
      else if ((rx_header.Identifier == 0U) &&
               ((rx_data[0] & 0x0FU) == (dm->can_id & 0x0FU)))
      {
        match = 1U;
      }

      if (match != 0U)
      {
        app_dm_decode(dm, rx_data);
        break;
      }
    }
  }
}

/* ==================== 4. 初始化：滤波器配置 ====================
 * 内部自动完成：配置滤波器 -> 配置全局滤波 -> 启动总线 -> 开启 FIFO0 新帧中断。
 * 滤波器配置结构体由调用方创建，can_filter_std() 填充，见文件头说明。
 * ============================================================ */
/**
  * @brief          生成标准帧滤波器配置（填充外部创建的结构体，不创建）
  * @param[out]     filter: 外部创建的滤波器配置结构体指针
  * @param[in]      id:    期望接收的标准帧 ID（如 0x201）
  * @param[in]      mask:  掩码，位为 1 表示"必须匹配该位"，位为 0 表示"该位不关心"
  * @param[in]      bank:  dji 版风格的滤波器编号（CAN1 用 0~13，CAN2 用 14~27）；
  *                        写进 FDCAN 的其实是实例内序号，CAN1 原样使用，
  *                        CAN2 的大于等于 14 的编号由 app_can2_filter_init 折算
  * @retval         none
  */
void can_filter_std(FDCAN_FilterTypeDef *filter, uint32_t id, uint32_t mask, uint32_t bank)
{
  filter->IdType = FDCAN_STANDARD_ID;              /* 11 位标准帧 */
  filter->FilterIndex = bank;                      /* 实例内滤波器序号 */
  filter->FilterType = FDCAN_FILTER_MASK;          /* 掩码模式：FilterID1 = ID, FilterID2 = 掩码 */
  filter->FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  /* 命中后进 Rx FIFO0 */
  filter->FilterID1 = id & 0x7FFU;
  filter->FilterID2 = mask & 0x7FFU;
  filter->RxBufferIndex = 0;                       /* 仅 Rx Buffer 模式使用，这里清零 */
  filter->IsCalibrationMsg = 0;                    /* 仅 Rx Buffer 模式使用，这里清零 */
}

/**
  * @brief          配置一个 FDCAN 实例的滤波器并启动它（内部函数）
  * @param[in]      hfdcan:           目标 FDCAN 句柄
  * @param[in]      filter:           已填充的滤波器配置指针
  * @param[in]      dji_bank_offset:  dji 版 Bank 编号到实例内序号的偏移（CAN1 为 0，CAN2 为 14）
  * @retval         none
  */
static void can_filter_init_one(FDCAN_HandleTypeDef *hfdcan, FDCAN_FilterTypeDef *filter,
                                uint32_t dji_bank_offset)
{
  uint32_t index = filter->FilterIndex;

  /* dji 版 Bank 编号 -> FDCAN 实例内序号（CAN2 的 14~27 折算成 0~13） */
  if (index >= dji_bank_offset)
    index -= dji_bank_offset;

  /* 序号越界时直接放弃配置：FDCAN 的滤波器列表紧挨着接收/发送缓冲区，
   * 越界写会踩坏消息 RAM，比"收不到帧"危险得多 */
  if (index >= hfdcan->Init.StdFiltersNbr)
    return;

  filter->FilterIndex = index;

  HAL_FDCAN_ConfigFilter(hfdcan, filter);

  /* 不匹配的帧直接丢弃，与 dji 版 bxCAN 的默认行为一致。
   * 注意 FDCAN 复位后默认是把不匹配的标准帧也收进 FIFO0（GFC.ANFS = 00），
   * 不配置全局滤波的话，总线上所有标准帧都会混进接收回调。 */
  HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                               FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

  HAL_FDCAN_Start(hfdcan);
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/**
  * @brief          (方案C) 配置并启动 FDCAN1
  * @param[in]      filter: 外部创建并填充好的滤波器配置指针
  * @retval         none
  */
void app_can1_filter_init(FDCAN_FilterTypeDef *filter)
{
  can_filter_init_one(&hfdcan1, filter, 0);
}

/**
  * @brief          (方案C) 配置并启动 FDCAN2
  * @param[in]      filter: 外部创建并填充好的滤波器配置指针
  * @retval         none
  */
void app_can2_filter_init(FDCAN_FilterTypeDef *filter)
{
  can_filter_init_one(&hfdcan2, filter, 14);
}

/* ==================== 5. 功能函数 ==================== */

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

/* ---------- 发送函数（按设备配置发送 8 字节经典 CAN 帧，供 app_can_cmd_* 调用） ---------- */

/**
  * @brief          按设备配置发送 8 字节数据（经典 CAN 帧）
  * @param[in]      dev:  设备配置指针
  * @param[in]      data: 待发送的 8 字节数据
  * @retval         none
  */
static void can_dev_send(can_tx_device_t *dev, uint8_t *data)
{
  FDCAN_TxHeaderTypeDef tx_header;

  tx_header.Identifier = dev->std_id;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_BYTES_8;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;    /* 经典帧，不做速率切换 */
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;     /* DJI 电机/云台/拨盘只认经典帧 */
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  /* 发送结果不检查、不记录（与 dji 原始版本一致；总线没启动/FIFO 满时该帧丢弃） */
  HAL_FDCAN_AddMessageToTxFifoQ(dev->hcan, &tx_header, data);
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
