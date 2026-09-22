/* Minimal stand-in for stm32h7xx_hal.h: only what app_iic.c touches (DMA only). */
#ifndef STM32H7XX_HAL_H
#define STM32H7XX_HAL_H

#include <stdint.h>

typedef enum
{
    HAL_OK      = 0x00,
    HAL_ERROR   = 0x01,
    HAL_BUSY    = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

#define HAL_I2C_ERROR_NONE      0x00000000U
#define HAL_I2C_ERROR_BERR      0x00000001U
#define HAL_I2C_ERROR_ARLO      0x00000002U
#define HAL_I2C_ERROR_AF        0x00000004U
#define HAL_I2C_ERROR_DMA_PARAM 0x00000080U

#define I2C_MEMADD_SIZE_8BIT  0x00000001U
#define I2C_MEMADD_SIZE_16BIT 0x00000010U

typedef struct
{
    void *Instance;
    uint32_t ErrorCode;
} I2C_HandleTypeDef;

HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                       uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                        uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);

#endif
