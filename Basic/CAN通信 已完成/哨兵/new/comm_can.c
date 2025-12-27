#include "comm_can.h"
#include "can.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"

#define CAN_POINTER_COMM        &hcan2

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

#define POS_MAX_Default     95.5f
#define VEL_MAX_DEFAULT     45.0f
#define T_MAX_DEFAULT       18.0f
#define KP_MAX              500.0f
#define KD_MAX              5.0f

_CAN_Para_t ZE300_CANBACK_DATA;

float pos_max =  POS_MAX_Default;
float vel_max =  VEL_MAX_DEFAULT;
float t_max   =  T_MAX_DEFAULT;

uint8_t canTxBuf[8] = {0};		/**!< can数据发送缓存 */

/* Converts a float to an unsigned int, given range and number of bits */
static int float_to_uint(float x, float x_min, float x_max, int bits)
{ 
    float span = x_max - x_min;
    float offset = x_min;
       
    return (int) ((LIMIT_MIN_MAX(x, x_min, x_max)-offset)*((float)((1<<bits)-1))/span);
}
    
/* converts unsigned int to float, given range and number of bits */  
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


void CommCan_Init(void)
{
    CAN_FilterTypeDef   sCAN_Filter;
    
    sCAN_Filter.FilterBank = 0;                         /* 指定将被初始化的过滤器 */  
    sCAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;     /* 过滤模式为屏蔽位模式 */
    sCAN_Filter.FilterScale = CAN_FILTERSCALE_16BIT;    /* 指定滤波器的规模 */
    sCAN_Filter.FilterIdHigh = 0;
    sCAN_Filter.FilterIdLow = 0;             
    sCAN_Filter.FilterMaskIdHigh = 0;
    sCAN_Filter.FilterMaskIdLow = 0;
    sCAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sCAN_Filter.FilterActivation = ENABLE;              /* 启用或禁用过滤器 */
    sCAN_Filter.SlaveStartFilterBank = 0;               /* 选择启动从过滤器组 */
    
    HAL_CAN_ConfigFilter(CAN_POINTER_COMM, &sCAN_Filter);
    
    HAL_CAN_Start(CAN_POINTER_COMM);       /* 开启CAN通信 */ 
    HAL_CAN_ActivateNotification(CAN_POINTER_COMM, CAN_IT_RX_FIFO0_MSG_PENDING);   /* 开启挂起中断允许 */
}

static void _Transmit(CAN_HandleTypeDef *canHandle, uint16_t dev_id, uint8_t cmd_id, uint8_t data_size, uint8_t* data)
{
    CAN_TxHeaderTypeDef CAN_Header;
    uint32_t canTxMailbox;
    uint8_t  i = 0;
    
    if(data_size <= 7)
	{
		canTxBuf[0] = cmd_id;
		
		if(data_size != 0)
		{
			if(data != NULL)
			{
				for(i = 0; i< data_size; i++)
				{
					canTxBuf[i+1] = data[i];
				}
			}
			else
			{
				return;
			}
		}
		
		CAN_Header.StdId    = dev_id;				/* 指定标准标识符，该值在0x00-0x7FF */
		CAN_Header.IDE      = CAN_ID_STD;       	/* 指定将要传输消息的标识符类型 */
		CAN_Header.RTR      = CAN_RTR_DATA;     	/* 指定消息传输帧类型 */
		CAN_Header.DLC      = (data_size + 1);		/* 指定将要传输的帧长度 */
		
		HAL_CAN_AddTxMessage(canHandle, &CAN_Header, canTxBuf, (uint32_t *)&canTxMailbox);
	}
}

/* 重启电机 */
void CommCan_SysMotorRestart(uint8_t dev_addr)
{
    uint8_t tmp_buf[7];
	tmp_buf[0] = 0xFF;
	tmp_buf[1] = 0x00;
	tmp_buf[2] = 0xFF;
	tmp_buf[3] = 0x00;
	tmp_buf[4] = 0xFF;
	tmp_buf[5] = 0x00;
	tmp_buf[6] = 0xFF;
    _Transmit(CAN_POINTER_COMM, dev_addr, eCAN_Restart, 7, tmp_buf);
}

/*   ****************************** 电机参数读取*********************************   */
/* 读取电机版本信息 */
void CommCan_GetVerInfo(uint8_t dev_addr)
{
    _Transmit(CAN_POINTER_COMM, dev_addr, eCAN_GetVerInfo, 0, NULL);	
}

/* 读取实时Q轴电流 */
void CommCan_GetIq(uint8_t dev_addr)
{
    _Transmit(CAN_POINTER_COMM, dev_addr, eCAN_GetIq, 0, NULL);	
}

/* 读取实时速度 */
void CommCan_GetVelocity(uint8_t dev_addr)
{
    _Transmit(CAN_POINTER_COMM, dev_addr, eCAN_GetVelocity, 0, NULL);	
}

/*读实时单圈绝对值角度、多圈绝对值角度 */
void CommCan_GetPosition(uint8_t dev_addr)
{
    _Transmit(CAN_POINTER_COMM, dev_addr, eCAN_GetPosition, 0, NULL);	
}

/* 读取电压、电流、温度、状态、故障实时状态信息 */
void CommCan_GetStatusInfo(uint8_t dev_addr)
{
    _Transmit(CAN_POINTER_COMM, dev_addr, eCAN_GetStatusInfo, 0, NULL);	
}

/* 读取电机硬件参数 */
void CommCan_GetMotorPara(uint8_t dev_addr)
{
    _Transmit(CAN_POINTER_COMM, dev_addr, eCAN_GetMotorPara, 0, NULL);	
}

/* 读取电机运动时的Q轴电流、速度、单圈绝对值 */
void CommCan_CAN_GetRunStatus(uint8_t dev_addr)
{
	_Transmit(CAN_POINTER_COMM, dev_addr, eCAN_GetRunStatus, 0, NULL);	
}

/*   ****************************** 电机参数设置*********************************   */
/* 电机清除故障 */
void CommCan_SysMotorResetFault(uint8_t dev_addr)
{   
    _Transmit(CAN_POINTER_COMM, dev_addr, eCAN_ResetFault, 0, NULL);		
}

/* 设置电机当前位置为原点*/
void CommCan_SetZeroByCurrentRawAngle(uint8_t dev_addr)
{
    _Transmit(CAN_POINTER_COMM, dev_addr, eCAN_SetZeroByCurrentRawAngle, 0, NULL);		
}

/* 设置位置模式最大速度 */
void CommCan_SetPosCtrlMaxVelocity(uint8_t dev_addr, float target_param)
{	
	uint32_t target_val =  (uint32_t)(target_param*100);
    _Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_SetPosCtrlMaxVelocity, 4, (uint8_t *)&target_val);
}

/* 设置位置或速度模式最大力矩 */
void CommCan_SetPosVelCtrlMaxIq(uint8_t dev_addr, float target_param)
{	
	uint32_t target_val =  (uint32_t)(target_param*1000);
    _Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_SetPosVelCtrlMaxIq, 4, (uint8_t *)&target_val);
}

/* 设置力矩速度电流斜率 */
void CommCan_SetIqCtrlSlope(uint8_t dev_addr, float target_param)
{	
	uint32_t target_val =  (uint32_t)(target_param*1000);
    _Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_SetIqCtrlSlope, 4, (uint8_t *)&target_val);
}

/* 设置速度模式下最大加速度 */
void CommCan_SetVelCtrlAcc(uint8_t dev_addr, float target_param)
{	
	uint32_t target_val =  (uint32_t)(target_param*100);
    _Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_SetVelCtrlAcc, 4, (uint8_t *)&target_val);
}

/* 位置环Kp参数设置*/
void CommCan_SetPosKpParam(uint8_t dev_addr, float target_param )
{
	_Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_Pos_Kp, 4 , (uint8_t *)&target_param);
}
 
/* 位置环Ki参数设置*/
void CommCan_SetPosKiParam(uint8_t dev_addr, float target_param )
{
	_Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_Pos_Ki, 4 , (uint8_t *)&target_param);
}
 
/* 速度环Kp参数设置*/
void CommCan_SetVelKpParam(uint8_t dev_addr, float target_param )
{
	_Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_Vel_Kp, 4 , (uint8_t *)&target_param);
}
 
/* 速度环Ki参数设置*/
void CommCan_SetVelKiParam(uint8_t dev_addr, float target_param )
{
	_Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_Vel_Ki, 4 , (uint8_t *)&target_param);
}

/* 力矩模式设置Q轴电流 */
void CommCan_SetIq(uint8_t dev_addr, float target_Iq)
{
    int32_t tmp_val = (int32_t)(target_Iq*1000);
    _Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_SetIqCtrlTragetVal, 4, (uint8_t*)&tmp_val);
}

/* 速度模式设置速度 */
void CommCan_SetVelocity(uint8_t dev_addr, float target_vel)
{
    int32_t tmp_val = (int32_t)(target_vel*100);

    _Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_SetVelCtrlTragetVal, 4 , (uint8_t*)&tmp_val);
}

/* 绝对值位置控制，单位Count，一圈为16384Counts */
void CommCan_SetAbsPosition_Count(uint8_t dev_addr, int32_t target_pos)
{
	_Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_SetAbsPosCtrlTragetVal, 4 , (uint8_t *) &target_pos);
}

/* 相对值位置控制，单位Count，一圈为16384Counts */
void CommCan_SetRelateive_Count(uint8_t dev_addr, int32_t target_pos)
{
	_Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_SetRelateiveTragetVal, 4 ,(uint8_t *) &target_pos);
}

/* 最短距离回到原点 */
void CommCan_ShortestHomePosition(uint8_t dev_addr)
{
	_Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_ShortestHomePosition, 0 , NULL);
}

/* 抱阀控制开关输出；参数0x00，抱闸失能开关断开；0x01，抱闸使能开关闭合；0xFF，读取抱闸开关状态 */
void CommCan_BreakCtrl(uint8_t dev_addr, uint8_t open_or_close)
{
	_Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_BreakCtrl, 1 , &open_or_close);
}

/* 电机失能 */
void CommCan_MotorDisableCtrl(uint8_t dev_addr)
{  
	_Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_DisableCtrl, 0 , NULL);
}

/* Mit类型协议运控模式，读取参数*/
void CommCan_GetMitCtrlModePara(uint8_t dev_addr)
{
	_Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_GetConfigMitCtrlModePara, 0 , NULL);
}

/* Mit类型协议运控模式，配置参数*/
void CommCan_ConfigMitCtrlModePara(uint8_t dev_addr, float pos_max_config, float vel_max_config, float t_max_config)
{
	uint16_t config_data[3];
	config_data[0] = (uint16_t)(pos_max_config*10);
	config_data[1] = (uint16_t)(vel_max_config*100);
	config_data[2] = (uint16_t)(t_max_config*100);
	_Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_GetConfigMitCtrlModePara, 6 ,(uint8_t *)&config_data);
}

/* it类型协议运控模式，实时读取位置、速度、力矩和状态信息*/
void CommCan_GetMitCtrlModeRealTimeData(uint8_t dev_addr)
{
	_Transmit(CAN_POINTER_COMM,  dev_addr, eCAN_GetMitCtrlModeRealTimeData, 0 , NULL);
}

/* 运控模式下运动控制命令 */
void CommCan_MotorRunCtrlMode_Control(uint32_t dev_addr, float pos_target, float vel_target, float pos_gain_kp, float vel_gain_kd, float torque_target)
{
    uint8_t tmp_buf[8];
    
	uint16_t position = float_to_uint(pos_target, -pos_max, pos_max, 16); 
	uint16_t velocity = float_to_uint(vel_target, -vel_max, vel_max, 12);
	uint16_t kp  = float_to_uint(pos_gain_kp, 0, KP_MAX, 12);
	uint16_t kd  = float_to_uint(vel_gain_kd, 0, KD_MAX, 12);
	uint16_t torque   = float_to_uint(torque_target, -t_max, t_max, 12);
    
	tmp_buf[0] = (position >> 8);
	tmp_buf[1] = (uint8_t)(position & 0xFF);
	tmp_buf[2] = (uint8_t)(velocity >> 4);
	tmp_buf[3] = (uint8_t)(((velocity & 0x0F) << 4) | (kp >> 8));
	tmp_buf[4] = (uint8_t)(kp & 0xFF);
	tmp_buf[5] = (uint8_t)(kd >> 4);
    tmp_buf[6] = (uint8_t)(((kd & 0x0F) << 4) | (torque >> 8));
	tmp_buf[7] =  (uint8_t)(torque & 0xFF);
    
    /* 数据发送 */
    {
        CAN_TxHeaderTypeDef CAN_Header;
        uint32_t canTxMailbox;
        
        CAN_Header.StdId    =  0x400 | dev_addr;    /* 当前命令标识符的最高位Bit[10]需置1 */
        CAN_Header.IDE      = CAN_ID_STD;
        CAN_Header.RTR      = CAN_RTR_DATA;
        CAN_Header.DLC      = sizeof(tmp_buf);

        HAL_CAN_AddTxMessage(CAN_POINTER_COMM, &CAN_Header, tmp_buf, (uint32_t *)&canTxMailbox)
    }
}


/* can总线数据的接收 */
void GIM8115_CAN_Receive(uint8_t rx_cmd, uint8_t *rx_data)
{
    switch(rx_cmd)
    {
		case eCAN_GetVerInfo:
		{            
			memcpy(&ZE300_CANBACK_DATA.CAN_GetVerInfo.BootAppStatus,&(rx_data[1]),sizeof(ZE300_CANBACK_DATA.CAN_GetVerInfo.BootAppStatus));
			memcpy(&ZE300_CANBACK_DATA.CAN_GetVerInfo.ApplicationStatus,&(rx_data[3]),sizeof(ZE300_CANBACK_DATA.CAN_GetVerInfo.ApplicationStatus));
			memcpy(&ZE300_CANBACK_DATA.CAN_GetVerInfo.HardwardStatus,&(rx_data[5]),sizeof(ZE300_CANBACK_DATA.CAN_GetVerInfo.HardwardStatus));
			memcpy(&ZE300_CANBACK_DATA.CAN_GetVerInfo.CAN_SelfStatues,&(rx_data[7]),sizeof(ZE300_CANBACK_DATA.CAN_GetVerInfo.CAN_SelfStatues));
		}
		break;
		
		case eCAN_Pos_Kp:
		{
			memcpy(&ZE300_CANBACK_DATA.Pos_kp_param,&(rx_data[1]),sizeof(ZE300_CANBACK_DATA.Pos_kp_param));	
		}
		break;
		
		case eCAN_Pos_Ki:
		{
			memcpy(&ZE300_CANBACK_DATA.Pos_ki_param,&(rx_data[1]),sizeof(ZE300_CANBACK_DATA.Pos_ki_param));		
		}
		break;
		
		case eCAN_Vel_Kp:
		{
			memcpy(&ZE300_CANBACK_DATA.Vel_kp_param,&(rx_data[1]),sizeof(ZE300_CANBACK_DATA.Vel_kp_param));	
		}
		break;
		
		case eCAN_Vel_Ki:
		{
			memcpy(&ZE300_CANBACK_DATA.Vel_ki_param,&(rx_data[1]),sizeof(ZE300_CANBACK_DATA.Vel_ki_param));	
		}
		break;
		
		case eCAN_SetIqCtrlTragetVal:
		case eCAN_GetIq:
		{
			memcpy(&ZE300_CANBACK_DATA.IqCurrentValue,&(rx_data[1]),sizeof(ZE300_CANBACK_DATA.IqCurrentValue));
		}
		break;

        case eCAN_SetVelCtrlTragetVal:		
		case eCAN_GetVelocity:
		{
			memcpy(&ZE300_CANBACK_DATA.RealtimeVelocity,&(rx_data[1]),sizeof(ZE300_CANBACK_DATA.RealtimeVelocity));  		
		}
		break;


		case eCAN_DisableCtrl:
		case eCAN_GetStatusInfo:
		{
			memcpy(&ZE300_CANBACK_DATA.CAN_GetStatusInfo.BusVoltage_multi100,&(rx_data[1]),sizeof(ZE300_CANBACK_DATA.CAN_GetStatusInfo.BusVoltage_multi100));
			memcpy(&ZE300_CANBACK_DATA.CAN_GetStatusInfo.BusCurrent_multi100,&(rx_data[3]),sizeof(ZE300_CANBACK_DATA.CAN_GetStatusInfo.BusCurrent_multi100));
			memcpy(&ZE300_CANBACK_DATA.CAN_GetStatusInfo.Temperature,&(rx_data[5]),sizeof(ZE300_CANBACK_DATA.CAN_GetStatusInfo.Temperature));
			memcpy(&ZE300_CANBACK_DATA.CAN_GetStatusInfo.RunMode,&(rx_data[6]),sizeof(ZE300_CANBACK_DATA.CAN_GetStatusInfo.RunMode));
			memcpy(&ZE300_CANBACK_DATA.CAN_GetStatusInfo.SysFault,&(rx_data[7]),sizeof(ZE300_CANBACK_DATA.CAN_GetStatusInfo.SysFault));
		}
		break;

		case eCAN_ResetFault:
		{				
			memcpy(&ZE300_CANBACK_DATA.SysFault,&(rx_data[1]),sizeof(ZE300_CANBACK_DATA.SysFault));
		}
		break;
		
		case eCAN_GetMotorPara:
		{
			memcpy(&ZE300_CANBACK_DATA.CAN_GetMotorPara.motor_pole,&(rx_data[1]),sizeof(ZE300_CANBACK_DATA.CAN_GetMotorPara.motor_pole));
			memcpy(&ZE300_CANBACK_DATA.CAN_GetMotorPara.moment_constant,&(rx_data[2]),sizeof(ZE300_CANBACK_DATA.CAN_GetMotorPara.moment_constant));
			memcpy(&ZE300_CANBACK_DATA.CAN_GetMotorPara.reduction_ratio,&(rx_data[6]),sizeof(ZE300_CANBACK_DATA.CAN_GetMotorPara.reduction_ratio));		
		}
		break;		

		case eCAN_SetZeroByCurrentRawAngle:
		{
			memcpy(&ZE300_CANBACK_DATA.offset_angle,&(rx_data[1]),sizeof(ZE300_CANBACK_DATA.offset_angle));		
		}
		break;	
		
		case eCAN_SetPosCtrlMaxVelocity:
		{
			memcpy(&ZE300_CANBACK_DATA.pos_outlimit,&(rx_data[1]),sizeof(ZE300_CANBACK_DATA.pos_outlimit));
		}
		break;	
		
        case eCAN_SetPosVelCtrlMaxIq:
        {
			memcpy(&ZE300_CANBACK_DATA.pos_vel_maxiq,&(rx_data[1]),sizeof(ZE300_CANBACK_DATA.pos_vel_maxiq));
		}
        break;
        
		case eCAN_SetIqCtrlSlope:
        {
			memcpy(&ZE300_CANBACK_DATA.iqslope,&(rx_data[1]),sizeof(ZE300_CANBACK_DATA.iqslope));    
        }
        break;
        
		case eCAN_SetVelCtrlAcc:
        {
			memcpy(&ZE300_CANBACK_DATA.vel_acc,&(rx_data[1]),sizeof(ZE300_CANBACK_DATA.vel_acc));   
        }
        break;
        
        case eCAN_BreakCtrl:
        {
            ZE300_CANBACK_DATA.break_status = rx_data[1];
        }
        break;
        
		case eCAN_GetPosition:
        case eCAN_SetAbsPosCtrlTragetVal:
        case eCAN_SetRelateiveTragetVal:
        case eCAN_ShortestHomePosition:
        {
			memcpy(&ZE300_CANBACK_DATA.CAN_GetPosition.SingleAbsPosCtrlTragetVal,&(rx_data[1]),sizeof(ZE300_CANBACK_DATA.CAN_GetPosition.SingleAbsPosCtrlTragetVal));
			memcpy(&ZE300_CANBACK_DATA.CAN_GetPosition.MultiAbsPosCtrlTragetVal,&(rx_data[3]),sizeof(ZE300_CANBACK_DATA.CAN_GetPosition.MultiAbsPosCtrlTragetVal));            
        }
        break;
        
		case eCAN_GetConfigMitCtrlModePara:
		{
			memcpy(&ZE300_CANBACK_DATA.eCAN_GetConfigMitCtrlModePara.pos_max_get, &(rx_data[1]), sizeof(ZE300_CANBACK_DATA.eCAN_GetConfigMitCtrlModePara.pos_max_get));
			memcpy(&ZE300_CANBACK_DATA.eCAN_GetConfigMitCtrlModePara.vel_max_get, &(rx_data[3]), sizeof(ZE300_CANBACK_DATA.eCAN_GetConfigMitCtrlModePara.vel_max_get));	
			memcpy(&ZE300_CANBACK_DATA.eCAN_GetConfigMitCtrlModePara.t_max_get, &(rx_data[5]),sizeof(ZE300_CANBACK_DATA.eCAN_GetConfigMitCtrlModePara.t_max_get));
            
            pos_max = ZE300_CANBACK_DATA.eCAN_GetConfigMitCtrlModePara.pos_max_get * 0.1f;
            vel_max = ZE300_CANBACK_DATA.eCAN_GetConfigMitCtrlModePara.vel_max_get * 0.01f;
            t_max = ZE300_CANBACK_DATA.eCAN_GetConfigMitCtrlModePara.t_max_get * 0.01f;		
		}
        break;
		
		case eCAN_GetMitCtrlModeRealTimeData:
		{
			uint8_t *pBuf = rx_data;
								
		    ZE300_CANBACK_DATA.eCAN_GetCurrentValue.curr_Pos =  uint_to_float(((pBuf[1]<<8) | pBuf[2]), -pos_max, pos_max, 16);
			ZE300_CANBACK_DATA.eCAN_GetCurrentValue.curr_vel =  uint_to_float(((pBuf[3]<<4) | ((pBuf[4]&0xF0) >> 4)), -vel_max, vel_max, 12);
			ZE300_CANBACK_DATA.eCAN_GetCurrentValue.curr_torque =  uint_to_float((((pBuf[4]&0x0F) << 8) | pBuf[5]), -t_max, t_max, 12);
            ZE300_CANBACK_DATA.eCAN_GetCurrentValue.status = pBuf[6];			
		}
        break;
		
        case eCAN_GetRunStatus:
		{	
			memcpy(&ZE300_CANBACK_DATA.CAN_GetRunStatus.Temperature, &(rx_data[1]), sizeof(ZE300_CANBACK_DATA.CAN_GetRunStatus.IqCurrentValue));
			memcpy(&ZE300_CANBACK_DATA.CAN_GetRunStatus.IqCurrentValue, &(rx_data[2]), sizeof(ZE300_CANBACK_DATA.CAN_GetRunStatus.IqCurrentValue));
			memcpy(&ZE300_CANBACK_DATA.CAN_GetRunStatus.RealtimeVelocity, &(rx_data[4]), sizeof(ZE300_CANBACK_DATA.CAN_GetRunStatus.RealtimeVelocity));	
			memcpy(&ZE300_CANBACK_DATA.CAN_GetRunStatus.SingleAbsPosCtrlTragetVal, &(rx_data[6]),sizeof(ZE300_CANBACK_DATA.CAN_GetRunStatus.SingleAbsPosCtrlTragetVal));
		}
		break;
		
        default:
        break;
    }
}










