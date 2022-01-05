/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32h7xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                    /**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

///**
//* @brief DMA2D MSP Initialization
//* This function configures the hardware resources used in this example
//* @param hdma2d: DMA2D handle pointer
//* @retval None
//*/
//void HAL_DMA2D_MspInit(DMA2D_HandleTypeDef* hdma2d)
//{
//  if(hdma2d->Instance==DMA2D)
//  {
//  /* USER CODE BEGIN DMA2D_MspInit 0 */
//
//  /* USER CODE END DMA2D_MspInit 0 */
//    /* Peripheral clock enable */
//    __HAL_RCC_DMA2D_CLK_ENABLE();
//  /* USER CODE BEGIN DMA2D_MspInit 1 */
//
//  /* USER CODE END DMA2D_MspInit 1 */
//  }
//
//}
//
///**
//* @brief DMA2D MSP De-Initialization
//* This function freeze the hardware resources used in this example
//* @param hdma2d: DMA2D handle pointer
//* @retval None
//*/
//void HAL_DMA2D_MspDeInit(DMA2D_HandleTypeDef* hdma2d)
//{
//  if(hdma2d->Instance==DMA2D)
//  {
//  /* USER CODE BEGIN DMA2D_MspDeInit 0 */
//
//  /* USER CODE END DMA2D_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_DMA2D_CLK_DISABLE();
//  /* USER CODE BEGIN DMA2D_MspDeInit 1 */
//
//  /* USER CODE END DMA2D_MspDeInit 1 */
//  }
//
//}
//
///**
//* @brief I2C MSP Initialization
//* This function configures the hardware resources used in this example
//* @param hi2c: I2C handle pointer
//* @retval None
//*/
//void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
//  if(hi2c->Instance==I2C4)
//  {
//  /* USER CODE BEGIN I2C4_MspInit 0 */
//////
//  /* USER CODE END I2C4_MspInit 0 */
//  /** Initializes the peripherals clock
//  */
//    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C4;
//    PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
//    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//    {
//      Error_Handler();
//    }
//
//    __HAL_RCC_GPIOD_CLK_ENABLE();
//    /**I2C4 GPIO Configuration
//    PD12     ------> I2C4_SCL
//    PD13     ------> I2C4_SDA
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
//    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//
//    /* Peripheral clock enable */
//    __HAL_RCC_I2C4_CLK_ENABLE();
//  /* USER CODE BEGIN I2C4_MspInit 1 */
//////
//  /* USER CODE END I2C4_MspInit 1 */
//  }
//
//}
//
///**
//* @brief I2C MSP De-Initialization
//* This function freeze the hardware resources used in this example
//* @param hi2c: I2C handle pointer
//* @retval None
//*/
//void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
//{
//  if(hi2c->Instance==I2C4)
//  {
//  /* USER CODE BEGIN I2C4_MspDeInit 0 */
//////
//  /* USER CODE END I2C4_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_I2C4_CLK_DISABLE();
//
//    /**I2C4 GPIO Configuration
//    PD12     ------> I2C4_SCL
//    PD13     ------> I2C4_SDA
//    */
//    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12);
//
//    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_13);
//
//  /* USER CODE BEGIN I2C4_MspDeInit 1 */
//////
//  /* USER CODE END I2C4_MspDeInit 1 */
//  }
//
//}
//
///**
//* @brief LTDC MSP Initialization
//* This function configures the hardware resources used in this example
//* @param hltdc: LTDC handle pointer
//* @retval None
//*/
//void HAL_LTDC_MspInit(LTDC_HandleTypeDef* hltdc)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
//  if(hltdc->Instance==LTDC)
//  {
//  /* USER CODE BEGIN LTDC_MspInit 0 */
//
//  /* USER CODE END LTDC_MspInit 0 */
//  /** Initializes the peripherals clock
//  */
//    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
//    PeriphClkInitStruct.PLL3.PLL3M = 5;
//    PeriphClkInitStruct.PLL3.PLL3N = 160;
//    PeriphClkInitStruct.PLL3.PLL3P = 2;
//    PeriphClkInitStruct.PLL3.PLL3Q = 2;
//    PeriphClkInitStruct.PLL3.PLL3R = 83;
//    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
//    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
//    PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
//    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//    {
//      Error_Handler();
//    }
//
//    /* Peripheral clock enable */
//    __HAL_RCC_LTDC_CLK_ENABLE();
//
//    __HAL_RCC_GPIOK_CLK_ENABLE();
//    __HAL_RCC_GPIOI_CLK_ENABLE();
//    __HAL_RCC_GPIOJ_CLK_ENABLE();
//    __HAL_RCC_GPIOH_CLK_ENABLE();
//    /**LTDC GPIO Configuration
//    PK5     ------> LTDC_B6
//    PI1     ------> LTDC_G6
//    PI0     ------> LTDC_G5
//    PK4     ------> LTDC_B5
//    PJ15     ------> LTDC_B3
//    PK6     ------> LTDC_B7
//    PK3     ------> LTDC_B4
//    PK7     ------> LTDC_DE
//    PJ14     ------> LTDC_B2
//    PJ12     ------> LTDC_B0
//    PI9     ------> LTDC_VSYNC
//    PJ13     ------> LTDC_B1
//    PI12     ------> LTDC_HSYNC
//    PI14     ------> LTDC_CLK
//    PK2     ------> LTDC_G7
//    PJ11     ------> LTDC_G4
//    PJ10     ------> LTDC_G3
//    PJ9     ------> LTDC_G2
//    PJ0     ------> LTDC_R1
//    PJ8     ------> LTDC_G1
//    PJ7     ------> LTDC_G0
//    PJ6     ------> LTDC_R7
//    PI15     ------> LTDC_R0
//    PJ1     ------> LTDC_R2
//    PJ5     ------> LTDC_R6
//    PH9     ------> LTDC_R3
//    PJ3     ------> LTDC_R4
//    PJ4     ------> LTDC_R5
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_3
//                          |GPIO_PIN_7|GPIO_PIN_2;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
//    HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);
//
//    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_9|GPIO_PIN_12
//                          |GPIO_PIN_14|GPIO_PIN_15;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
//    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
//
//    GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_12|GPIO_PIN_13
//                          |GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_0
//                          |GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_1
//                          |GPIO_PIN_5|GPIO_PIN_3|GPIO_PIN_4;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
//    HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);
//
//    GPIO_InitStruct.Pin = GPIO_PIN_9;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
//    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN LTDC_MspInit 1 */
//
//  /* USER CODE END LTDC_MspInit 1 */
//  }
//
//}
//
///**
//* @brief LTDC MSP De-Initialization
//* This function freeze the hardware resources used in this example
//* @param hltdc: LTDC handle pointer
//* @retval None
//*/
//void HAL_LTDC_MspDeInit(LTDC_HandleTypeDef* hltdc)
//{
//  if(hltdc->Instance==LTDC)
//  {
//  /* USER CODE BEGIN LTDC_MspDeInit 0 */
//
//  /* USER CODE END LTDC_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_LTDC_CLK_DISABLE();
//
//    /**LTDC GPIO Configuration
//    PK5     ------> LTDC_B6
//    PI1     ------> LTDC_G6
//    PI0     ------> LTDC_G5
//    PK4     ------> LTDC_B5
//    PJ15     ------> LTDC_B3
//    PK6     ------> LTDC_B7
//    PK3     ------> LTDC_B4
//    PK7     ------> LTDC_DE
//    PJ14     ------> LTDC_B2
//    PJ12     ------> LTDC_B0
//    PI9     ------> LTDC_VSYNC
//    PJ13     ------> LTDC_B1
//    PI12     ------> LTDC_HSYNC
//    PI14     ------> LTDC_CLK
//    PK2     ------> LTDC_G7
//    PJ11     ------> LTDC_G4
//    PJ10     ------> LTDC_G3
//    PJ9     ------> LTDC_G2
//    PJ0     ------> LTDC_R1
//    PJ8     ------> LTDC_G1
//    PJ7     ------> LTDC_G0
//    PJ6     ------> LTDC_R7
//    PI15     ------> LTDC_R0
//    PJ1     ------> LTDC_R2
//    PJ5     ------> LTDC_R6
//    PH9     ------> LTDC_R3
//    PJ3     ------> LTDC_R4
//    PJ4     ------> LTDC_R5
//    */
//    HAL_GPIO_DeInit(GPIOK, GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_3
//                          |GPIO_PIN_7|GPIO_PIN_2);
//
//    HAL_GPIO_DeInit(GPIOI, GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_9|GPIO_PIN_12
//                          |GPIO_PIN_14|GPIO_PIN_15);
//
//    HAL_GPIO_DeInit(GPIOJ, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_12|GPIO_PIN_13
//                          |GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_0
//                          |GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_1
//                          |GPIO_PIN_5|GPIO_PIN_3|GPIO_PIN_4);
//
//    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_9);
//
//  /* USER CODE BEGIN LTDC_MspDeInit 1 */
//
//  /* USER CODE END LTDC_MspDeInit 1 */
//  }
//
//}
//
///**
//* @brief QSPI MSP Initialization
//* This function configures the hardware resources used in this example
//* @param hqspi: QSPI handle pointer
//* @retval None
//*/
//void HAL_QSPI_MspInit(QSPI_HandleTypeDef* hqspi)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
//  if(hqspi->Instance==QUADSPI)
//  {
//  /* USER CODE BEGIN QUADSPI_MspInit 0 */
//////
//  /* USER CODE END QUADSPI_MspInit 0 */
//  /** Initializes the peripherals clock
//  */
//    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_QSPI;
//    PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_D1HCLK;
//    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//    {
//      Error_Handler();
//    }
//
//    /* Peripheral clock enable */
//    __HAL_RCC_QSPI_CLK_ENABLE();
//
//    __HAL_RCC_GPIOG_CLK_ENABLE();
//    __HAL_RCC_GPIOF_CLK_ENABLE();
//    __HAL_RCC_GPIOH_CLK_ENABLE();
//    __HAL_RCC_GPIOD_CLK_ENABLE();
//    /**QUADSPI GPIO Configuration
//    PG9     ------> QUADSPI_BK2_IO2
//    PG14     ------> QUADSPI_BK2_IO3
//    PG6     ------> QUADSPI_BK1_NCS
//    PF6     ------> QUADSPI_BK1_IO3
//    PF7     ------> QUADSPI_BK1_IO2
//    PF10     ------> QUADSPI_CLK
//    PF9     ------> QUADSPI_BK1_IO1
//    PH2     ------> QUADSPI_BK2_IO0
//    PH3     ------> QUADSPI_BK2_IO1
//    PD11     ------> QUADSPI_BK1_IO0
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_14;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
//    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
//
//    GPIO_InitStruct.Pin = GPIO_PIN_6;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
//    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
//
//    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
//    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
//
//    GPIO_InitStruct.Pin = GPIO_PIN_9;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
//    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
//
//    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
//    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
//
//    GPIO_InitStruct.Pin = GPIO_PIN_11;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
//    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//
//    /* QUADSPI interrupt Init */
//    HAL_NVIC_SetPriority(QUADSPI_IRQn, 15, 0);
//    HAL_NVIC_EnableIRQ(QUADSPI_IRQn);
//  /* USER CODE BEGIN QUADSPI_MspInit 1 */
//////
//  /* USER CODE END QUADSPI_MspInit 1 */
//  }
//
//}
//
///**
//* @brief QSPI MSP De-Initialization
//* This function freeze the hardware resources used in this example
//* @param hqspi: QSPI handle pointer
//* @retval None
//*/
//void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef* hqspi)
//{
//  if(hqspi->Instance==QUADSPI)
//  {
//  /* USER CODE BEGIN QUADSPI_MspDeInit 0 */
//////
//  /* USER CODE END QUADSPI_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_QSPI_CLK_DISABLE();
//
//    /**QUADSPI GPIO Configuration
//    PG9     ------> QUADSPI_BK2_IO2
//    PG14     ------> QUADSPI_BK2_IO3
//    PG6     ------> QUADSPI_BK1_NCS
//    PF6     ------> QUADSPI_BK1_IO3
//    PF7     ------> QUADSPI_BK1_IO2
//    PF10     ------> QUADSPI_CLK
//    PF9     ------> QUADSPI_BK1_IO1
//    PH2     ------> QUADSPI_BK2_IO0
//    PH3     ------> QUADSPI_BK2_IO1
//    PD11     ------> QUADSPI_BK1_IO0
//    */
//    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_9|GPIO_PIN_14|GPIO_PIN_6);
//
//    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_9);
//
//    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_2|GPIO_PIN_3);
//
//    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_11);
//
//    /* QUADSPI interrupt DeInit */
//    HAL_NVIC_DisableIRQ(QUADSPI_IRQn);
//  /* USER CODE BEGIN QUADSPI_MspDeInit 1 */
//////
//  /* USER CODE END QUADSPI_MspDeInit 1 */
//  }
//
//}
//
///**
//* @brief TIM_PWM MSP Initialization
//* This function configures the hardware resources used in this example
//* @param htim_pwm: TIM_PWM handle pointer
//* @retval None
//*/
//void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
//{
//  if(htim_pwm->Instance==TIM8)
//  {
//  /* USER CODE BEGIN TIM8_MspInit 0 */
//
//  /* USER CODE END TIM8_MspInit 0 */
//    /* Peripheral clock enable */
//    __HAL_RCC_TIM8_CLK_ENABLE();
//  /* USER CODE BEGIN TIM8_MspInit 1 */
//
//  /* USER CODE END TIM8_MspInit 1 */
//  }
//
//}
//
//void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  if(htim->Instance==TIM8)
//  {
//  /* USER CODE BEGIN TIM8_MspPostInit 0 */
//
//  /* USER CODE END TIM8_MspPostInit 0 */
//
//    __HAL_RCC_GPIOK_CLK_ENABLE();
//    /**TIM8 GPIO Configuration
//    PK0     ------> TIM8_CH3
//    */
//    GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
//    HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN TIM8_MspPostInit 1 */
//
//  /* USER CODE END TIM8_MspPostInit 1 */
//  }
//
//}
///**
//* @brief TIM_PWM MSP De-Initialization
//* This function freeze the hardware resources used in this example
//* @param htim_pwm: TIM_PWM handle pointer
//* @retval None
//*/
//void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
//{
//  if(htim_pwm->Instance==TIM8)
//  {
//  /* USER CODE BEGIN TIM8_MspDeInit 0 */
//
//  /* USER CODE END TIM8_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_TIM8_CLK_DISABLE();
//  /* USER CODE BEGIN TIM8_MspDeInit 1 */
//
//  /* USER CODE END TIM8_MspDeInit 1 */
//  }
//
//}
//
///**
//* @brief UART MSP Initialization
//* This function configures the hardware resources used in this example
//* @param huart: UART handle pointer
//* @retval None
//*/
//void HAL_UART_MspInit(UART_HandleTypeDef* huart)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
//  if(huart->Instance==USART3)
//  {
//  /* USER CODE BEGIN USART3_MspInit 0 */
//////
//  /* USER CODE END USART3_MspInit 0 */
//  /** Initializes the peripherals clock
//  */
//    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
//    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
//    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//    {
//      Error_Handler();
//    }
//
//    /* Peripheral clock enable */
//    __HAL_RCC_USART3_CLK_ENABLE();
//
//    __HAL_RCC_GPIOB_CLK_ENABLE();
//    /**USART3 GPIO Configuration
//    PB10     ------> USART3_TX
//    PB11     ------> USART3_RX
//    */
//    GPIO_InitStruct.Pin = VCP_TX_Pin|VCP_RX_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN USART3_MspInit 1 */
//////
//  /* USER CODE END USART3_MspInit 1 */
//  }
//
//}
//
///**
//* @brief UART MSP De-Initialization
//* This function freeze the hardware resources used in this example
//* @param huart: UART handle pointer
//* @retval None
//*/
//void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
//{
//  if(huart->Instance==USART3)
//  {
//  /* USER CODE BEGIN USART3_MspDeInit 0 */
//////
//  /* USER CODE END USART3_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_USART3_CLK_DISABLE();
//
//    /**USART3 GPIO Configuration
//    PB10     ------> USART3_TX
//    PB11     ------> USART3_RX
//    */
//    HAL_GPIO_DeInit(GPIOB, VCP_TX_Pin|VCP_RX_Pin);
//
//  /* USER CODE BEGIN USART3_MspDeInit 1 */
//////
//  /* USER CODE END USART3_MspDeInit 1 */
//  }
//
//}
//
//static uint32_t FMC_Initialized = 0;
//
//static void HAL_FMC_MspInit(void){
//  /* USER CODE BEGIN FMC_MspInit 0 */
//////
//  /* USER CODE END FMC_MspInit 0 */
//  GPIO_InitTypeDef GPIO_InitStruct ={0};
//  if (FMC_Initialized) {
//    return;
//  }
//  FMC_Initialized = 1;
//  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
//
//  /** Initializes the peripherals clock
//  */
//    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FMC;
//    PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
//    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//    {
//      Error_Handler();
//    }
//
//  /* Peripheral clock enable */
//  __HAL_RCC_FMC_CLK_ENABLE();
//
//  /** FMC GPIO Configuration
//  PE1   ------> FMC_NBL1
//  PE0   ------> FMC_NBL0
//  PG15   ------> FMC_SDNCAS
//  PD0   ------> FMC_D2
//  PD1   ------> FMC_D3
//  PG8   ------> FMC_SDCLK
//  PF2   ------> FMC_A2
//  PF1   ------> FMC_A1
//  PF0   ------> FMC_A0
//  PG5   ------> FMC_BA1
//  PF3   ------> FMC_A3
//  PG4   ------> FMC_BA0
//  PF5   ------> FMC_A5
//  PF4   ------> FMC_A4
//  PE10   ------> FMC_D7
//  PH5   ------> FMC_SDNWE
//  PF13   ------> FMC_A7
//  PF14   ------> FMC_A8
//  PE9   ------> FMC_D6
//  PE11   ------> FMC_D8
//  PD15   ------> FMC_D1
//  PD14   ------> FMC_D0
//  PF12   ------> FMC_A6
//  PF15   ------> FMC_A9
//  PE12   ------> FMC_D9
//  PE15   ------> FMC_D12
//  PF11   ------> FMC_SDNRAS
//  PG0   ------> FMC_A10
//  PE8   ------> FMC_D5
//  PE13   ------> FMC_D10
//  PH6   ------> FMC_SDNE1
//  PD10   ------> FMC_D15
//  PD9   ------> FMC_D14
//  PG1   ------> FMC_A11
//  PE7   ------> FMC_D4
//  PE14   ------> FMC_D11
//  PH7   ------> FMC_SDCKE1
//  PD8   ------> FMC_D13
//  */
//  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_9
//                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_8
//                          |GPIO_PIN_13|GPIO_PIN_7|GPIO_PIN_14;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
//  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_5|GPIO_PIN_4
//                          |GPIO_PIN_0|GPIO_PIN_1;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
//  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_15|GPIO_PIN_14
//                          |GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_8;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
//  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_3
//                          |GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_13|GPIO_PIN_14
//                          |GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_11;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
//  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
//  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN FMC_MspInit 1 */
//////
//  /* USER CODE END FMC_MspInit 1 */
//}
//
//void HAL_SDRAM_MspInit(SDRAM_HandleTypeDef* hsdram){
//  /* USER CODE BEGIN SDRAM_MspInit 0 */
//////
//  /* USER CODE END SDRAM_MspInit 0 */
//  HAL_FMC_MspInit();
//  /* USER CODE BEGIN SDRAM_MspInit 1 */
//////
//  /* USER CODE END SDRAM_MspInit 1 */
//}
//
//static uint32_t FMC_DeInitialized = 0;
//
//static void HAL_FMC_MspDeInit(void){
//  /* USER CODE BEGIN FMC_MspDeInit 0 */
//////
//  /* USER CODE END FMC_MspDeInit 0 */
//  if (FMC_DeInitialized) {
//    return;
//  }
//  FMC_DeInitialized = 1;
//  /* Peripheral clock enable */
//  __HAL_RCC_FMC_CLK_DISABLE();
//
//  /** FMC GPIO Configuration
//  PE1   ------> FMC_NBL1
//  PE0   ------> FMC_NBL0
//  PG15   ------> FMC_SDNCAS
//  PD0   ------> FMC_D2
//  PD1   ------> FMC_D3
//  PG8   ------> FMC_SDCLK
//  PF2   ------> FMC_A2
//  PF1   ------> FMC_A1
//  PF0   ------> FMC_A0
//  PG5   ------> FMC_BA1
//  PF3   ------> FMC_A3
//  PG4   ------> FMC_BA0
//  PF5   ------> FMC_A5
//  PF4   ------> FMC_A4
//  PE10   ------> FMC_D7
//  PH5   ------> FMC_SDNWE
//  PF13   ------> FMC_A7
//  PF14   ------> FMC_A8
//  PE9   ------> FMC_D6
//  PE11   ------> FMC_D8
//  PD15   ------> FMC_D1
//  PD14   ------> FMC_D0
//  PF12   ------> FMC_A6
//  PF15   ------> FMC_A9
//  PE12   ------> FMC_D9
//  PE15   ------> FMC_D12
//  PF11   ------> FMC_SDNRAS
//  PG0   ------> FMC_A10
//  PE8   ------> FMC_D5
//  PE13   ------> FMC_D10
//  PH6   ------> FMC_SDNE1
//  PD10   ------> FMC_D15
//  PD9   ------> FMC_D14
//  PG1   ------> FMC_A11
//  PE7   ------> FMC_D4
//  PE14   ------> FMC_D11
//  PH7   ------> FMC_SDCKE1
//  PD8   ------> FMC_D13
//  */
//  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_9
//                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_8
//                          |GPIO_PIN_13|GPIO_PIN_7|GPIO_PIN_14);
//
//  HAL_GPIO_DeInit(GPIOG, GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_5|GPIO_PIN_4
//                          |GPIO_PIN_0|GPIO_PIN_1);
//
//  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_15|GPIO_PIN_14
//                          |GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_8);
//
//  HAL_GPIO_DeInit(GPIOF, GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_3
//                          |GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_13|GPIO_PIN_14
//                          |GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_11);
//
//  HAL_GPIO_DeInit(GPIOH, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
//
//  /* USER CODE BEGIN FMC_MspDeInit 1 */
//////
//  /* USER CODE END FMC_MspDeInit 1 */
//}
//
//void HAL_SDRAM_MspDeInit(SDRAM_HandleTypeDef* hsdram){
//  /* USER CODE BEGIN SDRAM_MspDeInit 0 */
//////
//  /* USER CODE END SDRAM_MspDeInit 0 */
//  HAL_FMC_MspDeInit();
//  /* USER CODE BEGIN SDRAM_MspDeInit 1 */
//
//  /* USER CODE END SDRAM_MspDeInit 1 */
//}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

