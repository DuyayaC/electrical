#include "drv_CAN_receive.h"
#include "string.h"

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
    (ptr)->error_code = (data)[7];                                 \
  }



// static motor_measure_t motor_chassis[7];
static motor_measure_t motor_chassis_can1[8];
static motor_measure_t motor_chassis_can2[8];

static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static uint8_t gimbal_yaw_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header_can1;
  CAN_RxHeaderTypeDef rx_header_can2;
  uint8_t rx_data_can1[8];
  uint8_t rx_data_can2[8];

  uint8_t rx_cmd = 0;

  if (hcan->Instance == CAN1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header_can1, rx_data_can1);
    if (rx_header_can1.StdId == CAN_IMU_Back_ID)
    {
      IMU_UpdateData(rx_data_can1);
    }
    else if (rx_header_can1.StdId == CAN_MOTOR1_ID ||
             rx_header_can1.StdId == CAN_MOTOR2_ID ||
             rx_header_can1.StdId == CAN_MOTOR3_ID ||
             rx_header_can1.StdId == CAN_MOTOR4_ID)
    {
      rx_cmd = rx_data_can1[0];
      GIM8115_CAN_Receive(rx_cmd, rx_data_can1);
    }

    else
    {
      switch (rx_header_can1.StdId)
      {
      case CAN_3508_M1_ID:
      case CAN_3508_M2_ID:
      case CAN_YAW_MOTOR_ID:
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
    if (rx_header_can2.StdId == CAN_IMU_Back_ID)
    {
      IMU_UpdateData(rx_data_can2);
    }
    else
    {
      switch (rx_header_can2.StdId)
      {
      case CAN_3508_M1_ID:
      case CAN_3508_M2_ID:
      case CAN_PIT_MOTOR_ID:
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


void CAN_cmd_chassis(int16_t motor1, int16_t motor2)
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

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

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

// 依据实际改写确定
void CAN_cmd_shoot(int16_t shoot, int16_t barrel)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[4] = (shoot >> 8);
  gimbal_can_send_data[5] = shoot;
  gimbal_can_send_data[6] = (barrel >> 8);
  gimbal_can_send_data[7] = barrel;
  HAL_CAN_AddTxMessage(&SHOOT_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
/*
void SetAbsPosition_Count(int32_t target_pos_1, int32_t target_pos_2, int32_t target_pos_3, int32_t target_pos_4)
{
  CommCan_SetAbsPosition_Count(1, target_pos_1);
  CommCan_SetAbsPosition_Count(2, target_pos_2);
  CommCan_SetAbsPosition_Count(3, target_pos_3);
  CommCan_SetAbsPosition_Count(4, target_pos_4);
}

void SetPosVelCtrlMaxIq(float target_param_1, float target_param_2, float target_param_3, float target_param_4)
{
  CommCan_SetPosVelCtrlMaxIq(1, target_param_1);
  CommCan_SetPosVelCtrlMaxIq(2, target_param_2);
  CommCan_SetPosVelCtrlMaxIq(3, target_param_3);
  CommCan_SetPosVelCtrlMaxIq(4, target_param_4);
}

void SetPosCtrlMaxVelocity(float target_param_1, float target_param_2, float target_param_3, float target_param_4)
{
  CommCan_SetPosCtrlMaxVelocity(1, target_param_1);
  CommCan_SetPosCtrlMaxVelocity(2, target_param_2);
  CommCan_SetPosCtrlMaxVelocity(3, target_param_3);
  CommCan_SetPosCtrlMaxVelocity(4, target_param_4);
}

void SetPosKpParam(float target_param_1, float target_param_2, float target_param_3, float target_param_4)
{
  CommCan_SetPosKpParam(1, target_param_1);
  CommCan_SetPosKpParam(2, target_param_2);
  CommCan_SetPosKpParam(3, target_param_3);
  CommCan_SetPosKpParam(4, target_param_4);
}

void SetPosKiParam(float target_param_1, float target_param_2, float target_param_3, float target_param_4)
{
  CommCan_SetPosKiParam(1, target_param_1);
  CommCan_SetPosKiParam(2, target_param_2);
  CommCan_SetPosKiParam(3, target_param_3);
  CommCan_SetPosKiParam(4, target_param_4);
}

void MotorDisableCtrl(uint8_t dev_addr_1, uint8_t dev_addr_2, uint8_t dev_addr_3, uint8_t dev_addr_4)
{
    dev_addr_1 && CommCan_MotorDisableCtrl(dev_addr_1);
    dev_addr_2 && CommCan_MotorDisableCtrl(dev_addr_2);
    dev_addr_3 && CommCan_MotorDisableCtrl(dev_addr_3);
    dev_addr_4 && CommCan_MotorDisableCtrl(dev_addr_4);
}

void GetPosition(uint8_t dev_addr_1, uint8_t dev_addr_2, uint8_t dev_addr_3, uint8_t dev_addr_4)
{
  CommCan_GetPosition(dev_addr_1);
  CommCan_GetPosition(dev_addr_2);
  CommCan_GetPosition(dev_addr_3);
  CommCan_GetPosition(dev_addr_4);
}

void GetIq(uint8_t dev_addr_1, uint8_t dev_addr_2, uint8_t dev_addr_3, uint8_t dev_addr_4)
{
  CommCan_GetIq(dev_addr_1);
  CommCan_GetIq(dev_addr_2);
  CommCan_GetIq(dev_addr_3);
  CommCan_GetIq(dev_addr_4);
}

void GetRunStatus(uint8_t dev_addr_1, uint8_t dev_addr_2, uint8_t dev_addr_3, uint8_t dev_addr_4)
{
  CommCan_CAN_GetRunStatus(dev_addr_1);
  CommCan_CAN_GetRunStatus(dev_addr_2);
  CommCan_CAN_GetRunStatus(dev_addr_3);
  CommCan_CAN_GetRunStatus(dev_addr_4);
}
*/
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_chassis_can1[4];
}

const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_chassis_can2[5];
}

//拨弹电机
const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_chassis_can2[2];
}

const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis_can1[(i & 0x03)];
}

const motor_measure_t *get_friction_motor_measure_point(uint8_t i)
{
  return &motor_chassis_can2[(i & 0x03)];
}

//平衡哨兵特供