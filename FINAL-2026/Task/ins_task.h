#ifndef INS_TASK_H
#define INS_TASK_H

#include "MahonyAHRS_DSP.h"
#include "BMI088driver.h"
#include "bsp_spi.h"
#include "alg_filter.h"
#include "tim.h"
#include "cmsis_os.h"
#include "alg_pid.h"

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4

#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS    2
#define IMU_NOTIFY_SHFITS    3

#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

typedef struct
{
    float output;
    uint16_t PWM;
} IMU_t;
extern IMU_t imu_data;

extern void INS_TASK(void const *pvParameters);

#endif