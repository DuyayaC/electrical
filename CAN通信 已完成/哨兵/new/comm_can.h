#ifndef __COMM_CAN_H
#define __COMM_CAN_H

#include "main.h"

typedef struct
{
	uint16_t BootAppStatus;
	uint16_t ApplicationStatus;
	uint16_t HardwardStatus;
	uint8_t  CAN_SelfStatues;
}CAN_GetVerInfo_t __attribute__((aligned(4)));

typedef struct
{
	uint16_t BusVoltage_multi100;
	uint16_t BusCurrent_multi100;
	uint8_t Temperature;
	uint8_t RunMode;
	uint8_t SysFault;	
}CAN_GetStatusInfo_t __attribute__((aligned(4)));

typedef struct
{
	uint8_t motor_pole;
	float moment_constant;
	uint8_t  reduction_ratio;
}CAN_GetMotorPara_t __attribute__((aligned(4)));

typedef struct
{
	uint16_t SingleAbsPosCtrlTragetVal;
	int32_t MultiAbsPosCtrlTragetVal;
}CAN_GetPosition_t __attribute__((aligned(4)));

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
}CAN_GetCurrentValue_t __attribute__((aligned(4)));

typedef struct
{
	uint8_t Temperature;
	int16_t IqCurrentValue;
	int16_t RealtimeVelocity;
	uint16_t SingleAbsPosCtrlTragetVal;
}CAN_GetRunStatus_t __attribute__((aligned(4)));

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
}_CAN_Para_t __attribute__((aligned(4)));

/* 命令枚举 */
enum{
    eCAN_Restart                    = 0x00,     /**< 软件复位 */
    eCAN_GetVerInfo                 = 0xA0,     /**< 获取Boot软件版本、应用软件版本、硬件版本、CAN-AUX协议版本 */
    eCAN_GetIq                      = 0xA1,     /**< 读取实时Q轴电流 */
    eCAN_GetVelocity                = 0xA2,     /**< 读取实时速度 */
    eCAN_GetPosition                = 0xA3,     /**< 读实时单圈绝对值角度、多圈绝对值角度 */
	eCAN_GetRunStatus              	= 0xA4,     /**< 读取电机运动时的Q轴电流、速度、单圈绝对值 */
    eCAN_GetStatusInfo              = 0xAE,     /**< 读取电压、电流、温度、状态、故障实时状态信息 */
    eCAN_ResetFault                 = 0xAF,     /**< 清除故障 */
    
    eCAN_GetMotorPara               = 0xB0,     /**< 读取电机硬件参数 */
    eCAN_SetZeroByCurrentRawAngle   = 0xB1,     /**< 设置当前位置为原点 */
	
    eCAN_SetPosCtrlMaxVelocity      = 0xB2,     /**< 设置位置模式最大速度 */
    eCAN_SetPosVelCtrlMaxIq         = 0xB3,     /**< 设置位置环/速度环模式最大的Q轴电流 */
    eCAN_SetIqCtrlSlope             = 0xB4,     /**< 设置Q轴电流控制电流的斜率 */
    eCAN_SetVelCtrlAcc              = 0xB5,     /**< 设置速度控制时的加速度 */
    
	eCAN_Pos_Kp						= 0xB6,     /**< 位置闭环PID-kp */
	eCAN_Pos_Ki						= 0xB7,     /**< 位置闭环PID-ki */
	eCAN_Vel_Kp						= 0xB8,     /**< 速度闭环PID-kp */
	eCAN_Vel_Ki						= 0xB9,     /**< 速度闭环PID-ki */
	
    eCAN_SetIqCtrlTragetVal         = 0xC0,     /**< Q轴电流控制目标值 */
    eCAN_SetVelCtrlTragetVal        = 0xC1,     /**< 速度控制的目标值 */
    eCAN_SetAbsPosCtrlTragetVal     = 0xC2,     /**< 绝对值位置控制的目标值 */
    eCAN_SetRelateiveTragetVal      = 0xC3,     /**< 相对位置控制的目标值 */
    eCAN_ShortestHomePosition       = 0xC4,     /**< 最短距离回原点 */
    eCAN_BreakCtrl									= 0xCE,			/**< 抱阀开关输出控制 */
    eCAN_DisableCtrl                = 0xCF,     /**< 失能电机输出 */
    
    eCAN_GetConfigMitCtrlModePara   = 0xF0,     /**< 配置MIT模式控制参数 */
    eCAN_GetMitCtrlModeRealTimeData = 0xF1,     /**< 读取MIT模式控制实时数据 */
};


void CommCan_Init(void);

void CommCan_SysMotorRestart(uint8_t dev_addr);                             /* 重启电机 */
void CommCan_GetVerInfo(uint8_t dev_addr);                                  /* 读取电机版本信息 */
void CommCan_GetIq(uint8_t dev_addr);                                       /* 读取实时Q轴电流 */
void CommCan_GetVelocity(uint8_t dev_addr);                                 /* 读取实时速度 */
void CommCan_GetPosition(uint8_t dev_addr);                                 /*读实时单圈绝对值角度、多圈绝对值角度 */
void CommCan_GetStatusInfo(uint8_t dev_addr);                               /* 读取电压、电流、温度、状态、故障实时状态信息 */
void CommCan_GetMotorPara(uint8_t dev_addr);                                /* 读取电机硬件参数 */
void CommCan_CAN_GetRunStatus(uint8_t dev_addr);							/* 读取电机运动时的Q轴电流、速度、单圈绝对值 */
void CommCan_SysMotorResetFault(uint8_t dev_addr);                          /* 电机清除故障 */
void CommCan_SetZeroByCurrentRawAngle(uint8_t dev_addr);                    /* 设置电机当前位置为原点*/
void CommCan_SetPosCtrlMaxVelocity(uint8_t dev_addr, float target_param);   /* 设置位置模式最大速度 */
void CommCan_SetPosVelCtrlMaxIq(uint8_t dev_addr, float target_param);      /* 设置位置或速度模式最大力矩 */
void CommCan_SetIqCtrlSlope(uint8_t dev_addr, float target_param);          /* 设置力矩速度电流斜率 */	
void CommCan_SetVelCtrlAcc(uint8_t dev_addr, float target_param);           /* 设置速度模式下最大加速度 */
void CommCan_SetPosKpParam(uint8_t dev_addr, float target_param);          /* 位置环Kp参数设置*/
void CommCan_SetPosKiParam(uint8_t dev_addr, float target_param);          /* 位置环Ki参数设置*/
void CommCan_SetVelKpParam(uint8_t dev_addr, float target_param);          /* 速度环Kp参数设置*/
void CommCan_SetVelKiParam(uint8_t dev_addr, float target_param);          /* 速度环Ki参数设置*/
void CommCan_SetIq(uint8_t dev_addr, float target_Iq);                      /* 力矩模式设置Q轴电流 */
void CommCan_SetVelocity(uint8_t dev_addr, float target_vel);               /* 速度模式设置速度 */
void CommCan_SetAbsPosition_Count(uint8_t dev_addr, int32_t traget_pos);    /* 绝对值位置控制 */
void CommCan_SetRelateive_Count(uint8_t dev_addr, int32_t target_pos);      /* 相对值位置控制，单位Count，一圈为16384Counts */
void CommCan_ShortestHomePosition(uint8_t dev_addr);                        /* 最短距离回到原点 */
void CommCan_BreakCtrl(uint8_t dev_addr, uint8_t open_or_close);            /* 抱闸输出控制*/
void CommCan_MotorDisableCtrl(uint8_t dev_addr);                            /* 电机失能 */

void CommCan_GetMitCtrlModePara(uint8_t dev_addr);
void CommCan_ConfigMitCtrlModePara(uint8_t dev_addr, float config_pos_max, float config_vel_max, float config_t_max);
void CommCan_GetMitCtrlModeRealTimeData(uint8_t dev_addr);
void CommCan_MotorRunCtrlMode_Control(uint32_t dev_addr, float pos_target, float vel_target, float pos_gain_kp, float vel_gain_kd, float torque_target);

void GIM8115_CAN_Receive(uint8_t rx_cmd, uint8_t rx_data);

extern _CAN_Para_t ZE300_CANBACK_DATA;
#endif
