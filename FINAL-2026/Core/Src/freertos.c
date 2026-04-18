/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ins_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shooter_task.h"
#include "control_task.h"
#include "usbreceive_task.h"
#include "referee_task.h"
#include "calibrate_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId ins_taskHandle;
osThreadId chassis_taskHandle;
osThreadId gimbal_taskHandle;
osThreadId shooter_taskHandle;
osThreadId usbreceive_taskHandle;
osThreadId referee_taskHandle;
osThreadId calibrate_taskHandle;
osThreadId control_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void INS_TASK(void const * argument);
void CHASSIS_TASK(void const * argument);
void GIMBAL_TASK(void const * argument);
void SHOOTER_TASK(void const * argument);
void USBRECEIVE_TASK(void const * argument);
void REFEREE_TASK(void const * argument);
void CALIBRATE_TASK(void const * argument);
void CONTROL_TASK(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of ins_task */
  osThreadDef(ins_task, INS_TASK, osPriorityRealtime, 0, 512);
  ins_taskHandle = osThreadCreate(osThread(ins_task), NULL);

  /* definition and creation of chassis_task */
  osThreadDef(chassis_task, CHASSIS_TASK, osPriorityHigh, 0, 512);
  chassis_taskHandle = osThreadCreate(osThread(chassis_task), NULL);

  /* definition and creation of gimbal_task */
  osThreadDef(gimbal_task, GIMBAL_TASK, osPriorityHigh, 0, 512);
  gimbal_taskHandle = osThreadCreate(osThread(gimbal_task), NULL);

  /* definition and creation of shooter_task */
  osThreadDef(shooter_task, SHOOTER_TASK, osPriorityAboveNormal, 0, 512);
  shooter_taskHandle = osThreadCreate(osThread(shooter_task), NULL);

  /* definition and creation of usbreceive_task */
  osThreadDef(usbreceive_task, USBRECEIVE_TASK, osPriorityHigh, 0, 256);
  usbreceive_taskHandle = osThreadCreate(osThread(usbreceive_task), NULL);

  /* definition and creation of referee_task */
  osThreadDef(referee_task, REFEREE_TASK, osPriorityNormal, 0, 256);
  referee_taskHandle = osThreadCreate(osThread(referee_task), NULL);

  /* definition and creation of calibrate_task */
  osThreadDef(calibrate_task, CALIBRATE_TASK, osPriorityBelowNormal, 0, 128);
  calibrate_taskHandle = osThreadCreate(osThread(calibrate_task), NULL);

  /* definition and creation of control_task */
  osThreadDef(control_task, CONTROL_TASK, osPriorityHigh, 0, 512);
  control_taskHandle = osThreadCreate(osThread(control_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_INS_TASK */
/**
  * @brief  Function implementing the ins_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_TASK */
__weak void INS_TASK(void const * argument)
{
  /* init code for USB_DEVICE */
  
  /* USER CODE BEGIN INS_TASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_TASK */
}

/* USER CODE BEGIN Header_CHASSIS_TASK */
/**
* @brief Function implementing the chassis_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CHASSIS_TASK */
__weak void CHASSIS_TASK(void const * argument)
{
  /* USER CODE BEGIN CHASSIS_TASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CHASSIS_TASK */
}

/* USER CODE BEGIN Header_GIMBAL_TASK */
/**
* @brief Function implementing the gimbal_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GIMBAL_TASK */
__weak void GIMBAL_TASK(void const * argument)
{
  /* USER CODE BEGIN GIMBAL_TASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GIMBAL_TASK */
}

/* USER CODE BEGIN Header_SHOOTER_TASK */
/**
* @brief Function implementing the shooter_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SHOOTER_TASK */
__weak void SHOOTER_TASK(void const * argument)
{
  /* USER CODE BEGIN SHOOTER_TASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SHOOTER_TASK */
}

/* USER CODE BEGIN Header_USBRECEIVE_TASK */
/**
* @brief Function implementing the usbreceive_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_USBRECEIVE_TASK */
__weak void USBRECEIVE_TASK(void const * argument)
{
  /* USER CODE BEGIN USBRECEIVE_TASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END USBRECEIVE_TASK */
}

/* USER CODE BEGIN Header_REFEREE_TASK */
/**
* @brief Function implementing the referee_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_REFEREE_TASK */
__weak void REFEREE_TASK(void const * argument)
{
  /* USER CODE BEGIN REFEREE_TASK */
   /* Infinite loop */
   for(;;)
   {
     osDelay(1);
   }
  /* USER CODE END REFEREE_TASK */
}

/* USER CODE BEGIN Header_CALIBRATE_TASK */
/**
* @brief Function implementing the calibrate_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CALIBRATE_TASK */
__weak void CALIBRATE_TASK(void const * argument)
{
  /* USER CODE BEGIN CALIBRATE_TASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CALIBRATE_TASK */
}

/* USER CODE BEGIN Header_CONTROL_TASK */
/**
* @brief Function implementing the control_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CONTROL_TASK */
__weak void CONTROL_TASK(void const * argument)
{
  /* USER CODE BEGIN CONTROL_TASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CONTROL_TASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
