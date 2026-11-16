/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */

static BspI2cStats_t s_stats;                   /* 统计快照 */
static uint32_t s_last_err = 0U;                /* 最近一次HAL错误码 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
static void i2c_bus_clear(void)
{
        GPIO_InitTypeDef gpio={0};

        __HAL_RCC_GPIOB_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_6|GPIO_PIN_7;
        gpio.Mode = GPIO_MODE_OUTPUT_OD;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB,&gpio);

        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
        HAL_Delay(1);

        for (uint8_t i = 0U;i<9U;i++)
        {
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
                HAL_Delay(1U);
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
                HAL_Delay(1U);
        }

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(1U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_Delay(1U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_Delay(1U);
}

static void i2c_after_error(void)
{
        uint32_t err = HAL_I2C_GetError(&hi2c1);
        uint8_t need_recover = 0U;
        s_last_err = err;
        s_stats.err++;

        if ((err&(HAL_I2C_ERROR_BERR|HAL_I2C_ERROR_ARLO|HAL_I2C_ERROR_TIMEOUT))!=0U)
        {
                need_recover=1U;
        }
        if (hi2c1.State!=HAL_I2C_STATE_BUSY)
        {
                need_recover=1U;
        }
        if (err==HAL_I2C_ERROR_NONE)
        {
                need_recover=1U;
        }
        // TODO 后续加入freertos后，还需加一个调度器是否启动的判断条件
        if (need_recover!=0U)
        {
                bsp_i2c_recover();
        }
}

static HAL_StatusTypeDef i2c_finish(HAL_StatusTypeDef st)
{
        if (st == HAL_OK)
        {
                s_stats.ok++;
                return st;
        }

        if (st == HAL_BUSY)
        {
                s_stats.busy++;
        }
        i2c_after_error();
        return st;
}

void bsp_i2c_recover(void)
{
        s_stats.recover++;
        i2c_bus_clear();
        HAL_I2C_DeInit(&hi2c1);
        MX_I2C1_Init();
}

HAL_StatusTypeDef bsp_i2c_mem_write(uint8_t addr7, uint8_t reg, const uint8_t* data, uint16_t len, uint32_t timeout_ms)
{
        HAL_StatusTypeDef st;
        uint8_t retry = 0U;

        if ((data == NULL) || (len == 0U))
        {
                return HAL_ERROR;
        }

        do
        {
                st = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)((uint16_t)addr7 << 1U), reg,
                                       I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, len, timeout_ms);
        } while ((st != HAL_OK) && (retry++ < BSP_I2C_RETRY));

        return i2c_finish(st);
}

HAL_StatusTypeDef bsp_i2c_mem_read(uint8_t addr7, uint8_t reg, uint8_t* data, uint16_t len, uint32_t timeout_ms)
{
        HAL_StatusTypeDef st;
        uint8_t retry = 0U;

        if ((data == NULL) || (len == 0U))
        {
                return HAL_ERROR;
        }

        do
        {
                st = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)((uint16_t)addr7 << 1U), reg,
                                      I2C_MEMADD_SIZE_8BIT, data, len, timeout_ms);
        } while ((st != HAL_OK) && (retry++ < BSP_I2C_RETRY));

        return i2c_finish(st);
}

void bsp_i2c_init(void)
{
        s_stats.err = 0U;
        s_stats.recover = 0U;
        s_stats.busy = 0U;
        s_stats.ok = 0U;
        s_last_err = 0U;
}

HAL_StatusTypeDef bsp_i2c_write(uint8_t addr7, const uint8_t* data, uint16_t len, uint32_t timeout_ms)
{
        HAL_StatusTypeDef st;
        uint8_t retry = 0U;

        if ((data == NULL) || (len == 0U))
        {
                return HAL_ERROR;
        }

        do
        {
                st = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)((uint16_t)addr7 << 1U),
                                             (uint8_t*)data, len, timeout_ms);
        } while ((st != HAL_OK) && (retry++ < BSP_I2C_RETRY));

        return i2c_finish(st);
}

uint8_t bsp_i2c_is_device_ready(uint8_t addr7, uint32_t trials, uint32_t timeout_ms)
{
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)((uint16_t)addr7 << 1U),
                                  trials, timeout_ms) == HAL_OK)
        {
                return 1U;
        }

        s_last_err = HAL_I2C_GetError(&hi2c1);
        return 0U;
}

uint32_t bsp_i2c_last_error(void)
{
        return s_last_err;
}

/* USER CODE END 1 */

