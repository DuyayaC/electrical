#ifndef DRV_CAN_RECEIVE_H
#define DRV_CAN_RECEIVE_H

#include "can.h"
#include "main.h"

#define CHASSIS_CAN hcan2
#define FRICTION_CAN hcan1
#define GIMBAL_PITCH_CAN hcan1
#define SHOOT_CAN hcan1
#define GIMBAL_YAW_CAN hcan2

// motor data read (将电机反馈帧解析到 motor_measure_t)
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
    (ptr)->error = (data)[7];                                      \
  }

/**
  * @brief          build a filter config inline for standard 11-bit ID
  * @param[in]      id:   standard ID to accept
  * @param[in]      mask: mask, 0x000 = accept all, 0x7FF = exact match
  * @param[in]      bank: filter bank, CAN1 uses [0,13], CAN2 uses [14,27]
  * @retval         CAN_FilterTypeDef compound literal, pass to can_filter_init_can1/can2
  */
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
  * @brief          configure and start CAN1 with user-specified filter (bank [0,13])
  * @param[in]      filter: filter config by value,
  *                 e.g. can_filter_init_can1(CAN_FILTER_STD(0x000, 0x7FF, 0))
  * @retval         none
  */
extern void can_filter_init_can1(CAN_FilterTypeDef filter);

/**
  * @brief          configure and start CAN2 with user-specified filter (bank [14,27])
  * @param[in]      filter: filter config by value,
  *                 e.g. can_filter_init_can2(CAN_FILTER_STD(0x201, 0x7E0, 14))
  * @retval         none
  */
extern void can_filter_init_can2(CAN_FilterTypeDef filter);

/* CAN send and receive ID */
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

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    uint8_t error;
} motor_measure_t __attribute__((aligned(4)));


/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
extern void CAN_cmd_pitch(int16_t pitch);

extern void CAN_cmd_yaw(int16_t yaw);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern void CAN_cmd_shoot(int16_t motor1, int16_t motor2, int16_t shoot);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

extern const motor_measure_t *get_friction_motor_measure_point(uint8_t i);

extern const motor_measure_t *get_barrel_motor_measure_point(void);
#endif
