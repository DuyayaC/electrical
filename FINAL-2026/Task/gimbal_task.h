#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "cmsis_os.h"
#include "ins_task.h"
#include "alg_pid.h"
#include "drv_CAN_receive.h"
#include "ZeroCheck.h"
#include "control_task.h"
#include "usbreceive_task.h"
#include "SMC.h"
#include "calibrate_task.h"

#define MOUSE_X_MAP (-1.0f)
#define MOUSE_Y_MAP (-1.0f)

typedef struct
{
	float target_yaw;
	float target_pitch;
	float yaw_output;
	float pitch_output;
}gimbal_t;	

extern gimbal_t gimbal;

void GIMBAL_TASK(void const *pvParameters);

#endif