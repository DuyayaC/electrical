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
#include "dvc_key.h"
#include "dvc_referee.h"
#include "arm_math.h"
#include "comm_can.h"

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
float Yaw, Pitch, Roll, Yaw0;

float pitch_median_ecd = 3165.0f;
float pitch_lower_ecd = 2515.0f;
float pitch_higher_ecd = 3620.0f;
float pitch_target_angle = 0.0f; 
float pitch_output_current = 0.0f;

float test_pitch_ecd = 0;
float test_pitch_speed = 0;

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
//Class_Gimbal Gimbal;
Class_Serialplot serialplot;
Class_Filter_XYZ_Kalman Accel, Gyro0;
cSMC SMC, SMC0;
Class_PID pid_pitch;
//Class_PID PID_Omega;

/*void usart_printf(const char *fmt,...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);

    //return length of string 
    len = vsprintf((char *)tx_buf, fmt, ap);

    va_end(ap);
		HAL_Delay(0);
		HAL_UART_Transmit_DMA(&huart1, tx_buf, len);

}*/

/*void angle(float *Yaw, float *YAW0)
{
	
}*/



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

	while (BMI088_init())
	{
		;
	}
	UART_Init(&huart1, NULL, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH);
	serialplot.Init(&huart1, 12, (char **)Variable_Assignment_List);

	Accel.Init();
	Gyro0.Init();
	SMC.Init();
	pid_pitch.Init(20.0f, 0.0f, 0.0f, 0.0f, 5000.0f, 16384.0f);

	


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		
    const motor_measure_t *pitch_ptr = get_pitch_gimbal_motor_measure_point();

		float current_relative_pos = (float)pitch_ptr->ecd - pitch_median_ecd;
		
		pid_pitch.Set_Values(pitch_target_angle, current_relative_pos);
		pid_pitch.Calculate(&pitch_output_current);
		
		
		serialplot.Set_Data(1, &current_relative_pos);
		serialplot.TIM_Write_PeriodElapsedCallback();
		TIM_UART_PeriodElapsedCallback();
		
		CAN_cmd_pitch(pitch_output_current);
		HAL_Delay(0);
		

		
		
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
