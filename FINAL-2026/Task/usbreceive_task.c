#include "usbreceive_task.h"

extern usb_t usb;

void USBRECEIVE_TASK(void const * argument)
{
    while(1)
    {
		while(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
		{
		}
        usbreceive(UserRxBufferFS, UserRxLen);
		usb.deltaYaw = pAngleData->yaw;
		usb.deltaPitch = pAngleData->pitch;
		usb.auto_fire_flag = pAngleData->flag;
        UserRxLen = 0;
        pAngleData = NULL; // Clear the pointer after processing
        
    }
}

