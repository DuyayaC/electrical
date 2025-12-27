/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv_CAN_receive.h"
#include "drv_uart.h"
#include "drv_math.h"
#include "drv_bsp_can.h"
#include "dvc_serialplot.h"
#include "arm_math.h"

#include "alg_pid.h"
#include "alg_filter.h"
#include "MahonyAHRS_DSP.h"
#include "Sliding.hpp"


#include "remote_control.h"
#include "usbd_cdc_if.h"
#include "bsp_usart.h"
#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "dm_imu.h"

#include <stdio.h>
#include <stdarg.h>
#include "string.h"

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
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static char Variable_Assignment_List[][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
  "po",

	"po1",

	"po2",

	"po3",

};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  /* USER CODE BEGIN 2 */
	can_filter_init();
	UART_Init(&huart1, NULL, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH);
	serialplot.Init(&huart1, 12, (char **)Variable_Assignment_List);
	while(1)
	{
		serialplot.Set_Data(1, &omega);
		serialplot.TIM_Write_PeriodElapsedCallback();
		TIM_UART_PeriodElapsedCallback();
		HAL_Delay(0);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}