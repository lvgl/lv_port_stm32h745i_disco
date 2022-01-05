/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h745i_discovery_qspi.h"
#include "lvgl/lvgl.h"
#include "lvgl_port_lcd.h"
#include "lvgl_port_touchpad.h"
#include "lvgl/demos/lv_demos.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

//DMA2D_HandleTypeDef hdma2d;
//
//I2C_HandleTypeDef hi2c4;
//
//LTDC_HandleTypeDef hltdc;
//
//QSPI_HandleTypeDef hqspi;
//
//TIM_HandleTypeDef htim8;
//
//UART_HandleTypeDef huart3;
//
//MDMA_HandleTypeDef hmdma_mdma_channel40_sw_0;
//SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_FMC_Init(void);
//static void MX_LTDC_Init(void);
//static void MX_QUADSPI_Init(void);
//static void MX_MDMA_Init(void);
//static void MX_I2C4_Init(void);
//static void MX_DMA2D_Init(void);
//static void MX_TIM8_Init(void);
//static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
	int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/* Wait until CPU2 boots and enters in stop mode or timeout*/
	timeout = 0xFFFF;
	while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0))
		;
	if (timeout < 0)
	{
		Error_Handler();
	}
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
	/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
	 HSEM notification */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/*Take HSEM */
	HAL_HSEM_FastTake(HSEM_ID_0);
	/*Release HSEM in order to notify the CPU2(CM4)*/
	HAL_HSEM_Release(HSEM_ID_0, 0);
	/* wait until CPU2 wakes up from stop mode */
	timeout = 0xFFFF;
	while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0))
		;
	if (timeout < 0)
	{
		Error_Handler();
	}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_FMC_Init();
//  MX_LTDC_Init();
//  MX_QUADSPI_Init();
//  MX_MDMA_Init();
//  MX_I2C4_Init();
//  MX_DMA2D_Init();
//  MX_TIM8_Init();
//  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	/* Configure LED1 and LED2 */
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);

	//Init QSPI Memory
	BSP_QSPI_Init_t qspi_init;
	qspi_init.InterfaceMode = MT25TL01G_QPI_MODE;
	qspi_init.TransferRate = MT25TL01G_DTR_TRANSFER;
	qspi_init.DualFlashMode = MT25TL01G_DUALFLASH_ENABLE;
	BSP_QSPI_Init(0, &qspi_init);
	BSP_QSPI_EnableMemoryMappedMode(0);

	lv_init();
	LCD_init();
	touchpad_init();
	lv_demo_widgets();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(5);
		lv_task_handler();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

///**
//  * @brief DMA2D Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_DMA2D_Init(void)
//{
//
//  /* USER CODE BEGIN DMA2D_Init 0 */
//
//  /* USER CODE END DMA2D_Init 0 */
//
//  /* USER CODE BEGIN DMA2D_Init 1 */
//
//  /* USER CODE END DMA2D_Init 1 */
//  hdma2d.Instance = DMA2D;
//  hdma2d.Init.Mode = DMA2D_M2M;
//  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
//  hdma2d.Init.OutputOffset = 0;
//  hdma2d.LayerCfg[1].InputOffset = 0;
//  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
//  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
//  hdma2d.LayerCfg[1].InputAlpha = 0;
//  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
//  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
//  hdma2d.LayerCfg[1].ChromaSubSampling = DMA2D_NO_CSS;
//  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN DMA2D_Init 2 */
//
//  /* USER CODE END DMA2D_Init 2 */
//
//}
//
///**
//  * @brief I2C4 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_I2C4_Init(void)
//{
//
//  /* USER CODE BEGIN I2C4_Init 0 */
//
//  /* USER CODE END I2C4_Init 0 */
//
//  /* USER CODE BEGIN I2C4_Init 1 */
//
//  /* USER CODE END I2C4_Init 1 */
//  hi2c4.Instance = I2C4;
//  hi2c4.Init.Timing = 0x307075B1;
//  hi2c4.Init.OwnAddress1 = 0;
//  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//  hi2c4.Init.OwnAddress2 = 0;
//  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
//  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Analogue filter
//  */
//  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Digital filter
//  */
//  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN I2C4_Init 2 */
//
//  /* USER CODE END I2C4_Init 2 */
//
//}
//
///**
//  * @brief LTDC Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_LTDC_Init(void)
//{
//
//  /* USER CODE BEGIN LTDC_Init 0 */
//
//  /* USER CODE END LTDC_Init 0 */
//
//  LTDC_LayerCfgTypeDef pLayerCfg = {0};
//  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};
//
//  /* USER CODE BEGIN LTDC_Init 1 */
//
//  /* USER CODE END LTDC_Init 1 */
//  hltdc.Instance = LTDC;
//  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
//  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
//  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
//  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
//  hltdc.Init.HorizontalSync = 40;
//  hltdc.Init.VerticalSync = 8;
//  hltdc.Init.AccumulatedHBP = 53;
//  hltdc.Init.AccumulatedVBP = 10;
//  hltdc.Init.AccumulatedActiveW = 533;
//  hltdc.Init.AccumulatedActiveH = 282;
//  hltdc.Init.TotalWidth = 565;
//  hltdc.Init.TotalHeigh = 284;
//  hltdc.Init.Backcolor.Blue = 255;
//  hltdc.Init.Backcolor.Green = 255;
//  hltdc.Init.Backcolor.Red = 255;
//  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  pLayerCfg.WindowX0 = 0;
//  pLayerCfg.WindowX1 = 480;
//  pLayerCfg.WindowY0 = 0;
//  pLayerCfg.WindowY1 = 272;
//  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
//  pLayerCfg.Alpha = 255;
//  pLayerCfg.Alpha0 = 0;
//  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
//  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
//  pLayerCfg.FBStartAdress = 0;
//  pLayerCfg.ImageWidth = 480;
//  pLayerCfg.ImageHeight = 272;
//  pLayerCfg.Backcolor.Blue = 0;
//  pLayerCfg.Backcolor.Green = 0;
//  pLayerCfg.Backcolor.Red = 0;
//  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  pLayerCfg1.WindowX0 = 0;
//  pLayerCfg1.WindowX1 = 0;
//  pLayerCfg1.WindowY0 = 0;
//  pLayerCfg1.WindowY1 = 0;
//  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
//  pLayerCfg1.Alpha = 0;
//  pLayerCfg1.Alpha0 = 0;
//  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
//  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
//  pLayerCfg1.FBStartAdress = 0;
//  pLayerCfg1.ImageWidth = 0;
//  pLayerCfg1.ImageHeight = 0;
//  pLayerCfg1.Backcolor.Blue = 0;
//  pLayerCfg1.Backcolor.Green = 0;
//  pLayerCfg1.Backcolor.Red = 0;
//  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN LTDC_Init 2 */
//
//  /* USER CODE END LTDC_Init 2 */
//
//}
//
///**
//  * @brief QUADSPI Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_QUADSPI_Init(void)
//{
//
//  /* USER CODE BEGIN QUADSPI_Init 0 */
//
//  /* USER CODE END QUADSPI_Init 0 */
//
//  /* USER CODE BEGIN QUADSPI_Init 1 */
//
//  /* USER CODE END QUADSPI_Init 1 */
//  /* QUADSPI parameter configuration*/
//  hqspi.Instance = QUADSPI;
//  hqspi.Init.ClockPrescaler = 3;
//  hqspi.Init.FifoThreshold = 1;
//  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
//  hqspi.Init.FlashSize = 26;
//  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_4_CYCLE;
//  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
//  hqspi.Init.DualFlash = QSPI_DUALFLASH_ENABLE;
//  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN QUADSPI_Init 2 */
//
//  /* USER CODE END QUADSPI_Init 2 */
//
//}
//
///**
//  * @brief TIM8 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM8_Init(void)
//{
//
//  /* USER CODE BEGIN TIM8_Init 0 */
//
//  /* USER CODE END TIM8_Init 0 */
//
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
//
//  /* USER CODE BEGIN TIM8_Init 1 */
//
//  /* USER CODE END TIM8_Init 1 */
//  htim8.Instance = TIM8;
//  htim8.Init.Prescaler = 4;
//  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim8.Init.Period = 50000;
//  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim8.Init.RepetitionCounter = 0;
//  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//  sBreakDeadTimeConfig.DeadTime = 0;
//  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//  sBreakDeadTimeConfig.BreakFilter = 0;
//  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
//  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
//  sBreakDeadTimeConfig.Break2Filter = 0;
//  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM8_Init 2 */
//
//  /* USER CODE END TIM8_Init 2 */
//  HAL_TIM_MspPostInit(&htim8);
//
//}
//
///**
//  * @brief USART3 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART3_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART3_Init 0 */
//
//  /* USER CODE END USART3_Init 0 */
//
//  /* USER CODE BEGIN USART3_Init 1 */
//
//  /* USER CODE END USART3_Init 1 */
//  huart3.Instance = USART3;
//  huart3.Init.BaudRate = 115200;
//  huart3.Init.WordLength = UART_WORDLENGTH_8B;
//  huart3.Init.StopBits = UART_STOPBITS_1;
//  huart3.Init.Parity = UART_PARITY_NONE;
//  huart3.Init.Mode = UART_MODE_TX_RX;
//  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
//  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART3_Init 2 */
//
//  /* USER CODE END USART3_Init 2 */
//
//}
//
///**
//  * Enable MDMA controller clock
//  * Configure MDMA for global transfers
//  *   hmdma_mdma_channel40_sw_0
//  */
//static void MX_MDMA_Init(void)
//{
//
//  /* MDMA controller clock enable */
//  __HAL_RCC_MDMA_CLK_ENABLE();
//  /* Local variables */
//
//  /* Configure MDMA channel MDMA_Channel0 */
//  /* Configure MDMA request hmdma_mdma_channel40_sw_0 on MDMA_Channel0 */
//  hmdma_mdma_channel40_sw_0.Instance = MDMA_Channel0;
//  hmdma_mdma_channel40_sw_0.Init.Request = MDMA_REQUEST_SW;
//  hmdma_mdma_channel40_sw_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
//  hmdma_mdma_channel40_sw_0.Init.Priority = MDMA_PRIORITY_HIGH;
//  hmdma_mdma_channel40_sw_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
//  hmdma_mdma_channel40_sw_0.Init.SourceInc = MDMA_SRC_INC_WORD;
//  hmdma_mdma_channel40_sw_0.Init.DestinationInc = MDMA_DEST_INC_WORD;
//  hmdma_mdma_channel40_sw_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_WORD;
//  hmdma_mdma_channel40_sw_0.Init.DestDataSize = MDMA_DEST_DATASIZE_WORD;
//  hmdma_mdma_channel40_sw_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
//  hmdma_mdma_channel40_sw_0.Init.BufferTransferLength = 128;
//  hmdma_mdma_channel40_sw_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
//  hmdma_mdma_channel40_sw_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
//  hmdma_mdma_channel40_sw_0.Init.SourceBlockAddressOffset = 0;
//  hmdma_mdma_channel40_sw_0.Init.DestBlockAddressOffset = 0;
//  if (HAL_MDMA_Init(&hmdma_mdma_channel40_sw_0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* MDMA interrupt initialization */
//  /* MDMA_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(MDMA_IRQn);
//
//}
//
///* FMC initialization function */
//void MX_FMC_Init(void)
//{
//
//  /* USER CODE BEGIN FMC_Init 0 */
//
//  /* USER CODE END FMC_Init 0 */
//
//  FMC_SDRAM_TimingTypeDef SdramTiming = {0};
//
//  /* USER CODE BEGIN FMC_Init 1 */
//
//  /* USER CODE END FMC_Init 1 */
//
//  /** Perform the SDRAM1 memory initialization sequence
//  */
//  hsdram1.Instance = FMC_SDRAM_DEVICE;
//  /* hsdram1.Init */
//  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
//  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
//  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
//  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
//  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
//  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
//  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
//  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
//  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
//  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
//  /* SdramTiming */
//  SdramTiming.LoadToActiveDelay = 2;
//  SdramTiming.ExitSelfRefreshDelay = 7;
//  SdramTiming.SelfRefreshTime = 4;
//  SdramTiming.RowCycleDelay = 7;
//  SdramTiming.WriteRecoveryTime = 3;
//  SdramTiming.RPDelay = 2;
//  SdramTiming.RCDDelay = 2;
//
//  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
//  {
//    Error_Handler( );
//  }
//
//  /* USER CODE BEGIN FMC_Init 2 */
//
//  /* USER CODE END FMC_Init 2 */
//}
//
///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOK_CLK_ENABLE();
//  __HAL_RCC_GPIOG_CLK_ENABLE();
//  __HAL_RCC_GPIOI_CLK_ENABLE();
//  __HAL_RCC_GPIOE_CLK_ENABLE();
//  __HAL_RCC_GPIOJ_CLK_ENABLE();
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOD_CLK_ENABLE();
//  __HAL_RCC_GPIOF_CLK_ENABLE();
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(LCD_DISPLAY_MODE_GPIO_Port, LCD_DISPLAY_MODE_Pin, GPIO_PIN_SET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : LCD_DISPLAY_MODE_Pin */
//  GPIO_InitStruct.Pin = LCD_DISPLAY_MODE_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LCD_DISPLAY_MODE_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : B1_Pin */
//  GPIO_InitStruct.Pin = B1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : LD2_Pin */
//  GPIO_InitStruct.Pin = LD2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : LCD_INT_Pin */
//  GPIO_InitStruct.Pin = LCD_INT_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : LD1_Pin */
//  GPIO_InitStruct.Pin = LD1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : LCD_RESET_Pin */
//  GPIO_InitStruct.Pin = LCD_RESET_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LCD_RESET_GPIO_Port, &GPIO_InitStruct);
//
//  /* EXTI interrupt init*/
//  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
//
//}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0xD0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x90000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128MB;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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

#ifdef  USE_FULL_ASSERT
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

