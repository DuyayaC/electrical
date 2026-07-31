/**
 * @file          drv_CAN.c
 * @brief         大疆 RoboMaster 电机 CAN 驱动（3508 / 6020 / 2006 / 摩擦轮）
 *
 * ============================ 使用说明 ============================
 *
 * [1] 初始化（在 CubeMX 生成 MX_CAN1_Init() / MX_CAN2_Init() 之后调用）
 *     内部自动完成：配置滤波器 -> 启动总线 -> 开启 FIFO0 接收中断。
 *
 *     // CAN1：接收所有标准帧，Bank 0（滤波器 Bank 范围 [0,13]）
 *     can_filter_init_can1(CAN_FILTER_STD(0x000, 0x000, 0));
 *
 *     // CAN2：只接收 0x201~0x205（底盘 3508 + yaw 6020），Bank 14（范围 [14,27]）
 *     can_filter_init_can2(CAN_FILTER_STD(0x201, 0x7E0, 14));
 *
 *     掩码规则：mask=0x000 全接收；mask=0x7FF 精确匹配单个 ID；
 *              中间值(如 0x7E0/0x7F8) 匹配一段连续 ID 区间。
 *
 * ================================================================
 */

#include "drv_CAN.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
// motor data read
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
    (ptr)->error = (data)[7];                                      \
  }

static motor_measure_t motor_chassis_can1[7];
static motor_measure_t motor_chassis_can2[7];

static CAN_TxHeaderTypeDef gimbal_tx_message;
static CAN_TxHeaderTypeDef shoot_tx_message;
static uint8_t shoot_can_send_data[8];
static uint8_t yaw_can_send_data[8];
static uint8_t pitch_can_send_data[8];

static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

/**
 * @brief          configure and start CAN1 with user-specified filter
 * @param[in]      filter: filter config by value, CAN1 uses bank [0,13]
 * @retval         none
 */
void can_filter_init_can1(CAN_FilterTypeDef filter)
{
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief          configure and start CAN2 with user-specified filter
 * @param[in]      filter: filter config by value, CAN2 uses bank [14,27]
 * @retval         none
 */
void can_filter_init_can2(CAN_FilterTypeDef filter)
{
    HAL_CAN_ConfigFilter(&hcan2, &filter);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header_can1;
  CAN_RxHeaderTypeDef rx_header_can2;
  uint8_t rx_data_can1[8];
  uint8_t rx_data_can2[8];

  if (hcan->Instance == CAN1)
  {
	  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header_can1, rx_data_can1);
      switch (rx_header_can1.StdId)
      {
		  case CAN_3508_M1_ID:
		  case CAN_3508_M2_ID:
		  case CAN_PIT_MOTOR_ID:
		  case CAN_TRIGGER_MOTOR_ID:
		  {
			static uint8_t i = 0;
			// get motor id
			i = rx_header_can1.StdId - CAN_3508_M1_ID;
			get_motor_measure(&motor_chassis_can1[i], rx_data_can1);
			break;
		  }
		  default:
		  {
			break;
		  }
      }
  }
  else if (hcan->Instance == CAN2)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header_can2, rx_data_can2);
      switch (rx_header_can2.StdId)
      {
		  case CAN_3508_M1_ID:
		  case CAN_3508_M2_ID:
		  case CAN_3508_M3_ID:
		  case CAN_3508_M4_ID:
		  case CAN_YAW_MOTOR_ID:
		  {
			static uint8_t i = 0;
			// get motor id
			i = rx_header_can2.StdId - CAN_3508_M1_ID;
			get_motor_measure(&motor_chassis_can2[i], rx_data_can2);
			break;
		  }
		  default:
		  {
			break;
		  }
      }
  }
}

void CAN_cmd_pitch(int16_t pitch)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  pitch_can_send_data[2] = (pitch >> 8);
  pitch_can_send_data[3] = pitch;
  HAL_CAN_AddTxMessage(&GIMBAL_PITCH_CAN, &gimbal_tx_message, pitch_can_send_data, &send_mail_box);
}

void CAN_cmd_yaw(int16_t yaw)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  yaw_can_send_data[0] = (yaw >> 8);
  yaw_can_send_data[1] = yaw;
  HAL_CAN_AddTxMessage(&GIMBAL_YAW_CAN, &gimbal_tx_message, yaw_can_send_data, &send_mail_box);
}

void CAN_cmd_chassis_reset_ID(void)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x700;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0;
  chassis_can_send_data[1] = 0;
  chassis_can_send_data[2] = 0;
  chassis_can_send_data[3] = 0;
  chassis_can_send_data[4] = 0;
  chassis_can_send_data[5] = 0;
  chassis_can_send_data[6] = 0;
  chassis_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
 * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384]
 * @retval         none
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_shoot(int16_t motor1, int16_t motor2, int16_t shoot)
{
  uint32_t send_mail_box;
  shoot_tx_message.StdId = 0x200;
  shoot_tx_message.IDE = CAN_ID_STD;
  shoot_tx_message.RTR = CAN_RTR_DATA;
  shoot_tx_message.DLC = 0x08;
	
  shoot_can_send_data[0] = motor1 >> 8;
  shoot_can_send_data[1] = motor1;
  shoot_can_send_data[2] = motor2 >> 8;
  shoot_can_send_data[3] = motor2;
  shoot_can_send_data[4] = shoot >> 8;
  shoot_can_send_data[5] = shoot;
  shoot_can_send_data[6] = 0;
  shoot_can_send_data[7] = 0;
  HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}


const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_chassis_can2[4];
}

const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_chassis_can1[5];
}

const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_chassis_can1[2];
}

const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis_can2[(i & 0x03)];
}
	

const motor_measure_t *get_friction_motor_measure_point(uint8_t i)
{
  return &motor_chassis_can1[(i & 0x03)];
}
