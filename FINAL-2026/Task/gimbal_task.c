#include "gimbal_task.h"

Sliding yaw;
Sliding pitch;

extern usb_t usb;
extern VTM_Data_t key;
gimbal_t gimbal;

static void km_control(gimbal_t *gimbal);
static void rc_control(gimbal_t *gimbal);
static void pitch_constraint(gimbal_t *gimbal);
static void yaw_constraint(gimbal_t *gimbal);

void GIMBAL_TASK(void const *pvParameters)
{
    uint32_t xLastWakeTime = osKernelSysTick();
    const portTickType xFrequency = 1;

    SMC_Init(&pitch);
    SMC_Init(&yaw);

    gimbal.target_yaw = 0.0f;
    gimbal.target_pitch = 0.0f;

    SMC_SetParam_Exponent(&yaw, 0.8f, 120.0f, 15.0f, 0.8f, 2.0f, 16384.0f, 0.0f);
	SMC_SetParam_EISMC(&pitch, 1.0f, 140.0f, 12.0f, 6.0f, 8.0f, 1.0f, 16384.0f, 0.0f);

	osDelay(500);
    while (1)
    {   
			if (key.sw == 1)
			{
				km_control(&gimbal);
			}
			else if (key.sw == 0)
			{
				rc_control(&gimbal);
			}
				
      zerocheckangle(&gimbal.target_yaw, &mahony.YAW);
			pitch_constraint(&gimbal);
			yaw_constraint(&gimbal);

        SMC_ErrorUpdate(&yaw, gimbal.target_yaw, mahony.YAW, mahony.g_smc[2] * 57.2958f);
			SMC_ErrorUpdate(&pitch, gimbal.target_pitch, mahony.PITCH, mahony.g_smc[1] * 57.2958f);
        gimbal.yaw_output = SMC_Calculate(&yaw);
			gimbal.pitch_output = SMC_Calculate(&pitch);

        gimbal.pitch_output = -1.0f * gimbal.pitch_output;
			
			if (calibrate.flag != 1)
			{
        CAN_cmd_pitch((int16_t)gimbal.pitch_output);
        CAN_cmd_yaw((int16_t)gimbal.yaw_output);
			}
			else if (calibrate.flag == 1)
			{
				CAN_cmd_pitch(0);
				CAN_cmd_shoot(0,0,0);
				gimbal.target_yaw = 0.0f;
				gimbal.target_pitch = 0.0f;
			}
        osDelayUntil(&xLastWakeTime,xFrequency);
    }
}

static void km_control(gimbal_t *gimbal)
{
    if (key.mouse_btn_r == 0)
    {
        gimbal->target_yaw += MOUSE_X_MAP * key.mouse_x * 0.001f;
        gimbal->target_pitch += MOUSE_Y_MAP * key.mouse_y * 0.001f;
    }
    else if (key.mouse_btn_r == 1)
    {
        gimbal->target_yaw +=  usb.deltaYaw;
        gimbal->target_pitch +=  usb.deltaPitch;
    }
	usb.deltaPitch = 0.0f;
	usb.deltaYaw = 0.0f;
}

static void rc_control(gimbal_t *gimbal)
{
  gimbal->target_yaw += MOUSE_X_MAP * (1024 - key.ch3)  * 0.0001f;
  gimbal->target_pitch += MOUSE_Y_MAP * (key.ch2 - 1024) * 0.0001f;
}

static void pitch_constraint(gimbal_t *gimbal)
{
    gimbal->target_pitch = (gimbal->target_pitch > 32.0f) ? 32.0f : 
                           (gimbal->target_pitch < -32.0f) ? -32.0f : 
                           gimbal->target_pitch;
}

static void yaw_constraint(gimbal_t *gimbal)
{
    gimbal->target_yaw = (gimbal->target_yaw > 180.0f) ? 180.0f : 
                         (gimbal->target_yaw < -180.0f) ? -180.0f : 
                         gimbal->target_yaw;
}
