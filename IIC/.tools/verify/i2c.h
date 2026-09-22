#ifndef I2C_H
#define I2C_H
#include "main.h"
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
#endif
