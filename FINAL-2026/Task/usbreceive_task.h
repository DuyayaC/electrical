#ifndef USB_RECEIVE_TASK_H
#define USB_RECEIVE_TASK_H

#include "motion.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"

void USBRECEIVE_TASK(void const * argument);

#endif