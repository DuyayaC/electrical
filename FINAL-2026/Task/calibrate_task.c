#include "calibrate_task.h"

calibrate_t calibrate;

void CALIBRATE_TASK(void const *pvParameters)
{
    while (1)
    {
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
				{
				}
        
            MahonyAHRS_Reset(&mahony);
						SMC_IntegralClear(&yaw);
						SMC_IntegralClear(&pitch);
					PID_Clear_Error(&chassis[0]);
					PID_Clear_Error(&chassis[1]);
					PID_Clear_Error(&chassis[2]);
					PID_Clear_Error(&chassis[3]);
					PID_Clear_Error(&shooter[0]);
					PID_Clear_Error(&shooter[1]);
					PID_Clear_Error(&shooter[2]);
						

            if (BMI088.temperature < 40.0f)
            {
                IMU_Temperature_Control(BMI088.temperature);
            }
            else
            {
                TIM10->CCR1 = 0;
            }
    }
}
