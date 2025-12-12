#ifndef DRV_CAN_RECEIVE_H
#define DRV_CAN_RECEIVE_H

#include "struct_typedef.h"
#include "dm_imu.h"
#include "can.h"
#include "main.h"

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

} can_msg_id_e __attribute__((aligned(4)));

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t /**__attribute__((aligned(4)))**/;


typedef struct
{
	uint16_t BootAppStatus;
	uint16_t ApplicationStatus;
	uint16_t HardwardStatus;
	uint8_t  CAN_SelfStatues;
}CAN_GetVerInfo_t;

typedef struct
{
	uint16_t BusVoltage_multi100;
	uint16_t BusCurrent_multi100;
	uint8_t Temperature;
	uint8_t RunMode;
	uint8_t SysFault;	
}CAN_GetStatusInfo_t;

typedef struct
{
	uint8_t motor_pole;
	float moment_constant;
	uint8_t  reduction_ratio;
}CAN_GetMotorPara_t;

typedef struct
{
	uint16_t SingleAbsPosCtrlTragetVal;
	int32_t MultiAbsPosCtrlTragetVal;
}CAN_GetPosition_t;

typedef struct
{
	uint16_t pos_max_get;
	uint16_t vel_max_get;
	uint16_t t_max_get;
}CAN_GetConfigMitCtrlModePara_t;

typedef struct
{
	float curr_Pos;
	float curr_vel;
	float curr_torque;
	uint8_t status;
}CAN_GetCurrentValue_t;

typedef struct
{
	uint8_t Temperature;
	int16_t IqCurrentValue;
	int16_t RealtimeVelocity;
	uint16_t SingleAbsPosCtrlTragetVal;
}CAN_GetRunStatus_t;

typedef struct
{
	float Pos_kp_param;
	float Pos_ki_param;
	float Vel_kp_param;
	float Vel_ki_param;
	int32_t IqCurrentValue;
	int32_t RealtimeVelocity;
	uint8_t SysFault;
	uint8_t offset_angle;
	uint32_t pos_outlimit;
	uint32_t pos_vel_maxiq;
	uint32_t iqslope;
	uint32_t vel_acc;
	uint8_t break_status ;
	CAN_GetStatusInfo_t CAN_GetStatusInfo;
	CAN_GetVerInfo_t CAN_GetVerInfo;
	CAN_GetMotorPara_t CAN_GetMotorPara;
	CAN_GetPosition_t CAN_GetPosition;
	CAN_GetConfigMitCtrlModePara_t eCAN_GetConfigMitCtrlModePara;
	CAN_GetCurrentValue_t eCAN_GetCurrentValue;
	CAN_GetRunStatus_t CAN_GetRunStatus;
}_CAN_Para_t;
extern _CAN_Para_t ZE300_CANBACK_DATA;
/* ����ö�� */
enum{
    eCAN_Restart                    = 0x00,     /**< ������λ */
    eCAN_GetVerInfo                 = 0xA0,     /**< ��ȡBoot�����汾��Ӧ�������汾��Ӳ���汾��CAN-AUXЭ��汾 */
    eCAN_GetIq                      = 0xA1,     /**< ��ȡʵʱQ����� */
    eCAN_GetVelocity                = 0xA2,     /**< ��ȡʵʱ�ٶ� */
    eCAN_GetPosition                = 0xA3,     /**< ��ʵʱ��Ȧ����ֵ�Ƕȡ���Ȧ����ֵ�Ƕ� */
	eCAN_GetRunStatus              	= 0xA4,     /**< ��ȡ����˶�ʱ��Q��������ٶȡ���Ȧ����ֵ */
    eCAN_GetStatusInfo              = 0xAE,     /**< ��ȡ��ѹ���������¶ȡ�״̬������ʵʱ״̬��Ϣ */
    eCAN_ResetFault                 = 0xAF,     /**< ������� */
    
    eCAN_GetMotorPara               = 0xB0,     /**< ��ȡ���Ӳ������ */
    eCAN_SetZeroByCurrentRawAngle   = 0xB1,     /**< ���õ�ǰλ��Ϊԭ�� */
	
    eCAN_SetPosCtrlMaxVelocity      = 0xB2,     /**< ����λ��ģʽ����ٶ� */
    eCAN_SetPosVelCtrlMaxIq         = 0xB3,     /**< ����λ�û�/�ٶȻ�ģʽ����Q����� */
    eCAN_SetIqCtrlSlope             = 0xB4,     /**< ����Q��������Ƶ�����б�� */
    eCAN_SetVelCtrlAcc              = 0xB5,     /**< �����ٶȿ���ʱ�ļ��ٶ� */
    
	eCAN_Pos_Kp						= 0xB6,     /**< λ�ñջ�PID-kp */
	eCAN_Pos_Ki						= 0xB7,     /**< λ�ñջ�PID-ki */
	eCAN_Vel_Kp						= 0xB8,     /**< �ٶȱջ�PID-kp */
	eCAN_Vel_Ki						= 0xB9,     /**< �ٶȱջ�PID-ki */
	
    eCAN_SetIqCtrlTragetVal         = 0xC0,     /**< Q���������Ŀ��ֵ */
    eCAN_SetVelCtrlTragetVal        = 0xC1,     /**< �ٶȿ��Ƶ�Ŀ��ֵ */
    eCAN_SetAbsPosCtrlTragetVal     = 0xC2,     /**< ����ֵλ�ÿ��Ƶ�Ŀ��ֵ */
    eCAN_SetRelateiveTragetVal      = 0xC3,     /**< ���λ�ÿ��Ƶ�Ŀ��ֵ */
    eCAN_ShortestHomePosition       = 0xC4,     /**< ��̾����ԭ�� */
    eCAN_BreakCtrl									= 0xCE,			/**< ��������������� */
    eCAN_DisableCtrl                = 0xCF,     /**< ʧ�ܵ����� */
    
    eCAN_GetConfigMitCtrlModePara   = 0xF0,     /**< ����MITģʽ���Ʋ��� */
    eCAN_GetMitCtrlModeRealTimeData = 0xF1,     /**< ��ȡMITģʽ����ʵʱ���� */
};

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

extern void CAN_cmd_friction(int16_t motor1, int16_t motor2);

extern void CAN_cmd_shoot(int16_t shoot, int16_t rev);


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
