#ifndef SHOOTER_TASK_H
#define SHOOTER_TASK_H

#include "cmsis_os.h"
#include "alg_pid.h"
#include "drv_CAN_receive.h"
#include "motion.h"
#include "heat_control.h"
#include "control_task.h"
#include "stdbool.h"
#include "calibrate_task.h"

typedef struct
{
    float friction_omega[2];
    float trigger_omega;
    float friction_output[2];
    float trigger_output;
	float trigger_current;
	float target_trigger_omega;
	float desire_trigger_omega;
	float max_trigger_omega;
	uint8_t shoot_flag;
}shooter_control_t;

extern shooter_control_t shooter_control;

void SHOOTER_TASK(void const * argument);

#endif