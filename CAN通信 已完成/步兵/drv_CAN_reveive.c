#include "drv_CAN_receive.h"


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
    (prt)->error = (data)[7];                                      \ 
  }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
*/
// static motor_measure_t motor_chassis[7];
static motor_measure_t motor_chassis_can1[7];
static motor_measure_t motor_chassis_can2[7];

static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static uint8_t gimbal_yaw_can_send_data[8];

static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

/**
 * @brief          hal CAN fifo call back, receive motor data
 * @param[in]      hcan, the point to CAN handle
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header_can1;
  CAN_RxHeaderTypeDef rx_header_can2;
  uint8_t rx_data_can1[8];
  uint8_t rx_data_can2[8];

  if (hcan->Instance == CAN1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header_can1, rx_data_can1);
    if (rx_header_can1.StdId == CAN_IMU_RECEIVE_ID)
    {
      IMU_UpdateData(rx_data_can1);
    }
    else
    {
      switch (rx_header_can1.StdId)
      {
      case CAN_3508_M1_ID:
      case CAN_3508_M2_ID:
      case CAN_3508_M3_ID:
      case CAN_3508_M4_ID:
      case CAN_YAW_MOTOR_ID:
      case CAN_PIT_MOTOR_ID:
      case CAN_BARREL_MOTOR_ID:
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
  }
  else if (hcan->Instance == CAN2)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header_can2, rx_data_can2);
    if (rx_header_can2.StdId == CAN_IMU_RECEIVE_ID)
    {
      IMU_UpdateData(rx_data_can2);
    }
    else
    {
      switch (rx_header_can2.StdId)
      {
      case CAN_3508_M1_ID:
      case CAN_3508_M2_ID:
      case CAN_3508_M3_ID:
      case CAN_3508_M4_ID:
      case CAN_YAW_MOTOR_ID:
      case CAN_PIT_MOTOR_ID:
      case CAN_BARREL_MOTOR_ID:
      case CAN_TRIGGER_MOTOR_ID:
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
}

/**
 * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
 * @param[in]      yaw: (0x205) 6020 motor control current, range [-16384,16384]
 * @param[in]      pitch: (0x206) 6020 motor control current, range [-16384,16384]
 * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
 * @param[in]      rev: (0x208) reserve motor control current
 * @retval         none
 */

void CAN_cmd_pitch(int16_t pitch)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[2] = (pitch >> 8);
  gimbal_can_send_data[3] = pitch;
  HAL_CAN_AddTxMessage(&GIMBAL_PITCH_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void CAN_cmd_yaw(int16_t yaw)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_yaw_can_send_data[0] = (yaw >> 8);
  gimbal_yaw_can_send_data[1] = yaw;
  HAL_CAN_AddTxMessage(&GIMBAL_YAW_CAN, &gimbal_tx_message, gimbal_yaw_can_send_data, &send_mail_box);
}

/**
 * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
 * @param[in]      none
 * @retval         none
 */

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

#if (Robot_ID == 1)
void CAN_cmd_friction(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
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

  HAL_CAN_AddTxMessage(&FRICTION_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

//?????????��?��?��??????????
void CAN_cmd_shoot(int16_t shoot, int16_t rev)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = 0x1FF;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[4] = (shoot >> 8);
  gimbal_can_send_data[5] = shoot;
  gimbal_can_send_data[6] = (rev >> 8);
  gimbal_can_send_data[7] = rev;
  HAL_CAN_AddTxMessage(&SHOOT_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

#elif (Robot_ID == 3)
void CAN_cmd_friction(int16_t motor1, int16_t motor2)
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

  HAL_CAN_AddTxMessage(&FRICTION_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_shoot(int16_t shoot, int16_t rev)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = 0x1FF;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[4] = (shoot >> 8);
  gimbal_can_send_data[5] = shoot;
  gimbal_can_send_data[6] = (rev >> 8);
  gimbal_can_send_data[7] = rev;
  HAL_CAN_AddTxMessage(&SHOOT_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

#elif (Robot_ID == 4)
void CAN_cmd_friction(int16_t motor1, int16_t motor2)
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

  HAL_CAN_AddTxMessage(&FRICTION_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_shoot(int16_t shoot, int16_t barrel)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = 0x1FF;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[4] = (shoot >> 8);
  gimbal_can_send_data[5] = shoot;
  gimbal_can_send_data[6] = (barrel >> 8);
  gimbal_can_send_data[7] = barrel;
  HAL_CAN_AddTxMessage(&SHOOT_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
#endif

/**
 * @brief          return the yaw 6020 motor data point
 * @param[in]      none
 * @retval         motor data point
 */
#if (GIMBAL_YAW_CAN == hcan1)
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_chassis_can1[4];
}
#elif (GIMBAL_YAW_CAN == hcan2)
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_chassis_can2[4];
}
#endif
/**
 * @brief          return the pitch 6020 motor data point
 * @param[in]      none
 * @retval         motor data point
 */
#if (GIMBAL_PITCH_CAN == hcan1)

const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_chassis_can1[5];
}
#elif (GIMBAL_PITCH_CAN == hcan2)
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_chassis_can2[5];
}
#endif
/**
 * @brief          return the trigger 2006 motor data point
 * @param[in]      none
 * @retval         motor data point
 */
#if (SHOOT_CAN == hcan1)
const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_chassis_can1[6];
}
#elif (SHOOT_CAN == hcan2)
const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_chassis_can2[6];
}
#endif

/**
 * @brief          return the chassis 3508 motor data point
 * @param[in]      i: motor number,range [0,3]
 * @retval         motor data point
 */
#if (CHASSIS_CAN == hcan1)
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis_can1[(i & 0x03)];
}
#elif (CHASSIS_CAN == hcan2)
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis_can2[(i & 0x03)];
}
#endif

#if (FRICTION_CAN == hcan1)
const motor_measure_t *get_friction_motor_measure_point(uint8_t i)
{
  return &motor_chassis_can1[(i & 0x03)];
}
#elif (FRICTION_CAN == hcan2)
const motor_measure_t *get_friction_motor_measure_point(uint8_t i)
{
  return &motor_chassis_can2[(i & 0x03)];
}
#endif  