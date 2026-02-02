#ifndef DRV_CAN_RECEIVE_H
#define DRV_CAN_RECEIVE_H

#include "struct_typedef.h"
#include "comm_can.h"
#include "dm_imu.h"
#include "can.h"
#include "main.h"
#include "Robot_Property.h"

#define CHASSIS_CAN hcan2
#define FRICTION_CAN hcan1
#define GIMBAL_PITCH_CAN hcan1
#define SHOOT_CAN hcan1
#define GIMBAL_YAW_CAN hcan2
#define PWOER_CAN hcan2

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
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_BARREL_MOTOR_ID = 0x208,
    CAN_GIMBAL_ALL_ID = 0x1FE,

    CAN_MOTOR1_ID = 0x01,
    CAN_MOTOR2_ID = 0x02,
    CAN_MOTOR3_ID = 0x03,
    CAN_MOTOR4_ID = 0x04,
	
		CAN_IMU_ID = 0x05,
    CAN_IMU_Back_ID = 0x11,

} can_msg_id_e __attribute__((aligned(4)));

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
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
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2);

extern void CAN_cmd_friction(int16_t motor1, int16_t motor2);

extern void CAN_cmd_shoot(int16_t shoot, int16_t rev);

extern void SetAbsPosition_Count(int32_t target_pos_1, int32_t target_pos_2, int32_t target_pos_3, int32_t target_pos_4);
extern void MotorDisableCtrl(uint8_t dev_addr_1, uint8_t dev_addr_2, uint8_t dev_addr_3, uint8_t dev_addr_4);
extern void SetPosVelCtrlMaxIq(float target_param_1, float target_param_2, float target_param_3, float target_param_4);
extern void SetPosCtrlMaxVelocity(float target_param_1, float target_param_2, float target_param_3, float target_param_4);
extern void SetPosKpParam(float target_param_1, float target_param_2, float target_param_3, float target_param_4); 
extern void SetPosKiParam(float target_param_1, float target_param_2, float target_param_3, float target_param_4); 
extern void GetPosition(uint8_t dev_addr_1, uint8_t dev_addr_2, uint8_t dev_addr_3, uint8_t dev_addr_4);
extern void GetIq(uint8_t dev_addr_1, uint8_t dev_addr_2, uint8_t dev_addr_3, uint8_t dev_addr_4);
extern void GetRunStatus(uint8_t dev_addr_1, uint8_t dev_addr_2, uint8_t dev_addr_3, uint8_t dev_addr_4);

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
