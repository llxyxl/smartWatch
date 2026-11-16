/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */
#define BSP_I2C_TIMEOUT_MS 100U /* 单次I2C默认操作100ms超时 */
#define BSP_I2C_RETRY 1U        /* 失败后的重试次数，1=失败再试一次，仍失败则记录并恢复总线 */

typedef struct
{
        uint32_t err;           /* 累计错误次数 */
        uint32_t recover;       /* 累计总线恢复次数(通过9个SCL时钟+STOP恢复) */
        uint32_t busy;          /* 上一次操作因为总线忙/被占用而被拒绝的次数 */
        uint32_t ok;            /* 累计成功次数 */
}BspI2cStats_t;

void bsp_i2c_init(void);

void bsp_i2c_recover(void);

HAL_StatusTypeDef bsp_i2c_mem_write(uint8_t addr7, uint8_t reg, const uint8_t* data, uint16_t len, uint32_t timeout_ms);

HAL_StatusTypeDef bsp_i2c_mem_read(uint8_t addr7, uint8_t reg, uint8_t* data, uint16_t len, uint32_t timeout_ms);

HAL_StatusTypeDef bsp_i2c_write(uint8_t addr7, const uint8_t* data, uint16_t len, uint32_t timeout_ms);

uint8_t bsp_i2c_is_device_ready(uint8_t addr7, uint32_t trials, uint32_t timeout_ms);

uint32_t bsp_i2c_last_error(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

