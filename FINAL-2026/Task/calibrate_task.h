#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H

#include "cmsis_os.h"
#include "BMI088driver.h"
#include "MahonyAHRS_DSP.h"
#include "drv_CAN_receive.h"
#include "SMC.h"
#include "alg_pid.h"

typedef struct
{
	uint8_t flag;
}calibrate_t;
extern calibrate_t calibrate;

extern void CALIBRATE_TASK(void const *pvParameters);

#endif