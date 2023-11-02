/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;

/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
  .name = "MainTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Comunication */
osThreadId_t ComunicationHandle;
const osThreadAttr_t Comunication_attributes = {
  .name = "Comunication",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal3,
};
/* Definitions for feed */
osThreadId_t feedHandle;
const osThreadAttr_t feed_attributes = {
  .name = "feed",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal2,
};
/* USER CODE BEGIN PV */
//--------my global variables----------//
volatile uint16_t dma[4];
uint8_t error_my=0, feed_motion=0, M1_motion=0, polyarity,first_intrrupt=1; // 0-off 1-rigt, 2-left, 3-Rocking, 4-brake active
uint16_t Time_b=0,Time_c=0, feed_speed_sp=0, I_M4=0;
uint16_t PWM_M1=0; //0-1000 
uint16_t PWM_M4=0;//1000 RPS
uint32_t Freq_TIM2=10000;  //10000 is one rpm/sec or 600000 is one rpm/min
//========================================//
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void StartMainTask(void *argument);
void StartComunication(void *argument);
void StartFeed(void *argument);

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
  MX_IWDG_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of MainTask */
  MainTaskHandle = osThreadNew(StartMainTask, NULL, &MainTask_attributes);

  /* creation of Comunication */
  ComunicationHandle = osThreadNew(StartComunication, NULL, &Comunication_attributes);

  /* creation of feed */
  feedHandle = osThreadNew(StartFeed, NULL, &feed_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 840-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 1;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 65534;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 840-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim8, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 840-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim9, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 840-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim11, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 840-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim13, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 840-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim14, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 57600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_2;
  huart4.Init.Parity = UART_PARITY_EVEN;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DO_305_Feed_y_Pin|Q7_Pin|Q8_Pin|SPI_CS_Pin
                          |INT_Pin|RST_Pin|DO_308_Clamp_x_Pin|DO_306_Feed_z_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, DO_304_Feed_x_Pin|RE_DE_Pin|DO_310_Clamp_y_Pin|DO_312_Clamp_z_Pin
                          |led_oil_Pin|led_Temper_Pin|DO_63_K7_Pin|DO_51_K6_Pin
                          |res_do24v1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Q2_Pin|Q3_Pin|Q4_Pin|Q5_Pin
                          |Q6_Pin|res_do24v2_Pin|led_reserve1_m_Pin|led_X_lock_m_Pin
                          |led_Z_lock_m_Pin|led_Y_lock_m_Pin|led_reserve3_m_Pin|led_2t_m_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, led_manual_Pin|led_error_m_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE7 PE8
                           PE10 PE12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTI_A_Pin */
  GPIO_InitStruct.Pin = EXTI_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXTI_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DO_305_Feed_y_Pin Q7_Pin Q8_Pin DO_308_Clamp_x_Pin
                           DO_306_Feed_z_Pin */
  GPIO_InitStruct.Pin = DO_305_Feed_y_Pin|Q7_Pin|Q8_Pin|DO_308_Clamp_x_Pin
                          |DO_306_Feed_z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : S2rocking_Pin S28Z_endswitch1_Pin S25Z_endswitch2_Pin S27Y_endswitch1_Pin
                           DI_82_S13_m_Pin DI_34_S8_m_Pin DI_34_S7_m_Pin S6_m1_on_m_Pin
                           ReserveDI2_m_Pin DI_26_S4_m_Pin DI_87_S15_m_Pin */
  GPIO_InitStruct.Pin = S2rocking_Pin|S28Z_endswitch1_Pin|S25Z_endswitch2_Pin|S27Y_endswitch1_Pin
                          |DI_82_S13_m_Pin|DI_34_S8_m_Pin|DI_34_S7_m_Pin|S6_m1_on_m_Pin
                          |ReserveDI2_m_Pin|DI_26_S4_m_Pin|DI_87_S15_m_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : EXTI_B_Pin EXTI_C_Pin */
  GPIO_InitStruct.Pin = EXTI_B_Pin|EXTI_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_277_S31_m_Pin DI_276_S30_m_Pin DI_275_S29_m_Pin DI_260_S22_m_Pin
                           DI_203_S35_m_Pin DI_280_S34_m_Pin DI_279_S33_m_Pin DI_278_S32_m_Pin */
  GPIO_InitStruct.Pin = DI_277_S31_m_Pin|DI_276_S30_m_Pin|DI_275_S29_m_Pin|DI_260_S22_m_Pin
                          |DI_203_S35_m_Pin|DI_280_S34_m_Pin|DI_279_S33_m_Pin|DI_278_S32_m_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CS_Pin INT_Pin RST_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin|INT_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DO_304_Feed_x_Pin DO_310_Clamp_y_Pin DO_312_Clamp_z_Pin led_oil_Pin
                           led_Temper_Pin DO_63_K7_Pin DO_51_K6_Pin res_do24v1_Pin */
  GPIO_InitStruct.Pin = DO_304_Feed_x_Pin|DO_310_Clamp_y_Pin|DO_312_Clamp_z_Pin|led_oil_Pin
                          |led_Temper_Pin|DO_63_K7_Pin|DO_51_K6_Pin|res_do24v1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PG1 PG2 PG3 PG4
                           PG5 PG6 PG7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : tacho_dir_right_Pin tacho_dir_left_Pin DI_63_S9_m_Pin */
  GPIO_InitStruct.Pin = tacho_dir_right_Pin|tacho_dir_left_Pin|DI_63_S9_m_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Q2_Pin Q3_Pin Q4_Pin Q5_Pin
                           Q6_Pin res_do24v2_Pin led_reserve1_m_Pin led_X_lock_m_Pin
                           led_Z_lock_m_Pin led_Y_lock_m_Pin led_reserve3_m_Pin led_2t_m_Pin */
  GPIO_InitStruct.Pin = Q2_Pin|Q3_Pin|Q4_Pin|Q5_Pin
                          |Q6_Pin|res_do24v2_Pin|led_reserve1_m_Pin|led_X_lock_m_Pin
                          |led_Z_lock_m_Pin|led_Y_lock_m_Pin|led_reserve3_m_Pin|led_2t_m_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_252_m_Pin DI_246_m_Pin DI_253_m_Pin DI_29_S5_m_Pin
                           DI_92_S20_m_Pin DI_91_S19_m_Pin DI_90_S18_m_Pin DI_88_S16_m_Pin
                           DI_89_S17_m_Pin ESD_m_Pin F1_DI_m_Pin F2_DI_m_Pin
                           F3_DI_m_Pin S6Gearbox_Pin S11Gripe_Pin S12ungrip_Pin */
  GPIO_InitStruct.Pin = DI_252_m_Pin|DI_246_m_Pin|DI_253_m_Pin|DI_29_S5_m_Pin
                          |DI_92_S20_m_Pin|DI_91_S19_m_Pin|DI_90_S18_m_Pin|DI_88_S16_m_Pin
                          |DI_89_S17_m_Pin|ESD_m_Pin|F1_DI_m_Pin|F2_DI_m_Pin
                          |F3_DI_m_Pin|S6Gearbox_Pin|S11Gripe_Pin|S12ungrip_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RE_DE_Pin */
  GPIO_InitStruct.Pin = RE_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RE_DE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : led_manual_Pin led_error_m_Pin */
  GPIO_InitStruct.Pin = led_manual_Pin|led_error_m_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//---------------------------exti--------------------//

void EXTI4_IRQHandler(void)
{
    
    HAL_GPIO_EXTI_IRQHandler(EXTI_A_Pin);
    if (HAL_GPIO_ReadPin(EXTI_A_GPIO_Port,EXTI_A_Pin)==GPIO_PIN_SET)   //polyarity hight
    {
      if(polyarity==0){
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
      }         
    __HAL_TIM_SET_COUNTER(&htim3,0);
    }else{
      if(polyarity==1){
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
      }       
    }
    //-----M1---//
    TIM11->CCR1=PWM_M1;
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
}
void EXTI2_IRQHandler(void)
{
    
    HAL_GPIO_EXTI_IRQHandler(EXTI_B_Pin);
    if (HAL_GPIO_ReadPin(EXTI_B_GPIO_Port,EXTI_B_Pin)==GPIO_PIN_SET)
    {
      //polyarity hight
    Time_b=TIM3->CNT;
      if(polyarity==0){
        HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
      } 
    }else{
       if(polyarity==1){
        HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
      } 
    }
    //-----M1---//
    TIM13->CCR1=PWM_M1;
    HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
}
void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(EXTI_C_Pin);
    if (HAL_GPIO_ReadPin(EXTI_C_GPIO_Port,EXTI_C_Pin)==GPIO_PIN_SET)
    {
      if(polyarity==0){
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
      } 
    Time_c=TIM3->CNT;
    }else{
      if(polyarity==1){
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
      }     
    }
    //-----M1---//
    TIM14->CCR1=PWM_M1;
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);    
}

//====================================================//
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the MainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  HAL_GPIO_WritePin(DO_51_K6_GPIO_Port,DO_51_K6_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DO_63_K7_GPIO_Port,DO_63_K7_Pin,GPIO_PIN_RESET);
  HAL_ADC_Stop(&hadc1);
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&dma,4);
  NVIC_EnableIRQ(EXTI2_IRQn);
  NVIC_EnableIRQ(EXTI3_IRQn);
  NVIC_EnableIRQ(EXTI4_IRQn); 
  NVIC_EnableIRQ(TIM2_IRQn); 
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
  osDelay(50);  
  static uint8_t oil_pump_state=0, limit_switches=0; // feed_motion 1- x left, 2-x right, 3-y left, 4-y right, 5-z up, 6-z down same for limit swiches but coded in bits
 
  uint16_t feed_sp_buf[3]={0}, I_M4_buf[3]={0},feed_sp_temp, I_M4_temp;
  struct button_without_fix {
     GPIO_PinState pos_current;
     GPIO_PinState pos_previous;
  };
  struct button_without_fix X_left= {GPIO_PIN_RESET,GPIO_PIN_RESET}, X_right= {GPIO_PIN_RESET,GPIO_PIN_RESET}, Y_left= {GPIO_PIN_RESET,GPIO_PIN_RESET}, 
  Y_right= {GPIO_PIN_RESET,GPIO_PIN_RESET}, Z_up= {GPIO_PIN_RESET,GPIO_PIN_RESET}, Z_down={GPIO_PIN_RESET,GPIO_PIN_RESET}, lock_All={GPIO_PIN_RESET,GPIO_PIN_RESET},
  Unlock_X={GPIO_PIN_RESET,GPIO_PIN_RESET}, Unlock_Y={GPIO_PIN_RESET,GPIO_PIN_RESET}, Unlock_Z={GPIO_PIN_RESET,GPIO_PIN_RESET}, Feed_Stop={GPIO_PIN_RESET,GPIO_PIN_RESET}, 
  M1_on={GPIO_PIN_RESET,GPIO_PIN_RESET}, M1_off={GPIO_PIN_RESET,GPIO_PIN_RESET}, Oil_pump={GPIO_PIN_RESET,GPIO_PIN_RESET}, auto_man={GPIO_PIN_RESET,GPIO_PIN_RESET}, Rocking_btn={GPIO_PIN_RESET,GPIO_PIN_RESET},
  Deblock_btn={GPIO_PIN_RESET,GPIO_PIN_RESET};
  uint8_t i;
  /* Infinite loop */
  for(;;)
  {
    HAL_IWDG_Refresh(&hiwdg);
    

    
    // phase error
    if (Time_b>Time_c){
      error_my=1;
    }
    // ESD error
    if (HAL_GPIO_ReadPin(ESD_m_GPIO_Port,ESD_m_Pin)==GPIO_PIN_SET)
    {
      error_my=2;
    }
    // Temperature error F1
    if (HAL_GPIO_ReadPin(F1_DI_m_GPIO_Port,F1_DI_m_Pin)==GPIO_PIN_SET)
    {
      error_my=3;
      HAL_GPIO_WritePin(led_Temper_GPIO_Port,led_Temper_Pin, GPIO_PIN_SET);
    }
    // Temperature error F2
    if (HAL_GPIO_ReadPin(F2_DI_m_GPIO_Port,F2_DI_m_Pin)==GPIO_PIN_SET)
    {
      error_my=4;
      HAL_GPIO_WritePin(led_Temper_GPIO_Port,led_Temper_Pin, GPIO_PIN_SET);
    }
    // Temperature error F3
    if (HAL_GPIO_ReadPin(F3_DI_m_GPIO_Port,F3_DI_m_Pin)==GPIO_PIN_SET)
    {
      error_my=5;
      HAL_GPIO_WritePin(led_Temper_GPIO_Port,led_Temper_Pin, GPIO_PIN_SET);
    }
      
     // oil pump on
    Oil_pump.pos_previous=Oil_pump.pos_current;
    Oil_pump.pos_current=HAL_GPIO_ReadPin(DI_26_S4_m_GPIO_Port,DI_26_S4_m_Pin);
    if ((Oil_pump.pos_previous==GPIO_PIN_SET)&(Oil_pump.pos_current==GPIO_PIN_RESET))
    {
      oil_pump_state=~oil_pump_state;
    }
    
    //auto man btn
    auto_man.pos_previous=auto_man.pos_current;
    auto_man.pos_current=HAL_GPIO_ReadPin(DI_203_S35_m_GPIO_Port,DI_203_S35_m_Pin);
    if ((auto_man.pos_previous==GPIO_PIN_SET)&(auto_man.pos_current==GPIO_PIN_RESET))
    {
      HAL_GPIO_TogglePin(led_manual_GPIO_Port,led_manual_Pin);
    }                          
    
      // lock xyz
    lock_All.pos_previous=lock_All.pos_current;
    lock_All.pos_current=HAL_GPIO_ReadPin(DI_87_S15_m_GPIO_Port,DI_87_S15_m_Pin);
    if ((lock_All.pos_previous==GPIO_PIN_SET) && (lock_All.pos_current==GPIO_PIN_RESET))
    {
      if ((feed_motion!=1)||(feed_motion!=2))
      {
        HAL_GPIO_WritePin(DO_308_Clamp_x_GPIO_Port,DO_308_Clamp_x_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(led_X_lock_m_GPIO_Port,led_X_lock_m_Pin,GPIO_PIN_SET);
      }
      if ((feed_motion!=3)||(feed_motion!=4))
      {
        HAL_GPIO_WritePin(DO_310_Clamp_y_GPIO_Port,DO_310_Clamp_y_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(led_Y_lock_m_GPIO_Port,led_Y_lock_m_Pin,GPIO_PIN_SET);
      }
      if ((feed_motion!=5)||(feed_motion!=6))
      {
        HAL_GPIO_WritePin(DO_312_Clamp_z_GPIO_Port,DO_312_Clamp_z_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(led_Z_lock_m_GPIO_Port,led_Z_lock_m_Pin,GPIO_PIN_SET);
      }
    } 
    //unclamp X
    Unlock_X.pos_previous=Unlock_X.pos_current;
    Unlock_X.pos_current=HAL_GPIO_ReadPin(DI_88_S16_m_GPIO_Port,DI_88_S16_m_Pin);
    if ((Unlock_X.pos_previous==GPIO_PIN_SET)&&(Unlock_X.pos_current==GPIO_PIN_RESET))  
    {
      HAL_GPIO_WritePin(led_X_lock_m_GPIO_Port,led_X_lock_m_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DO_308_Clamp_x_GPIO_Port,DO_308_Clamp_x_Pin,GPIO_PIN_RESET);
    }
    //unclamp Y
    Unlock_Y.pos_previous=Unlock_Y.pos_current;
    Unlock_Y.pos_current=HAL_GPIO_ReadPin(DI_89_S17_m_GPIO_Port,DI_89_S17_m_Pin);
    if ((Unlock_Y.pos_previous==GPIO_PIN_SET)&&(Unlock_Y.pos_current==GPIO_PIN_RESET))  
    {    
      HAL_GPIO_WritePin(led_Y_lock_m_GPIO_Port,led_Y_lock_m_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DO_310_Clamp_y_GPIO_Port,DO_310_Clamp_y_Pin,GPIO_PIN_RESET);
    }    
    //unclamp Z
    Unlock_Z.pos_previous=Unlock_Z.pos_current;
    Unlock_Z.pos_current=HAL_GPIO_ReadPin(DI_90_S18_m_GPIO_Port,DI_90_S18_m_Pin);
    if ((Unlock_Z.pos_previous==GPIO_PIN_SET)&&(Unlock_Z.pos_current==GPIO_PIN_RESET))      
    {
      HAL_GPIO_WritePin(led_Z_lock_m_GPIO_Port,led_Z_lock_m_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DO_312_Clamp_z_GPIO_Port,DO_312_Clamp_z_Pin,GPIO_PIN_RESET);   
    }      
    
    //motion defenition 2 t 4 t  man/auto end swich
    X_left.pos_previous=X_left.pos_current;
    X_left.pos_current=HAL_GPIO_ReadPin(DI_276_S30_m_GPIO_Port,DI_276_S30_m_Pin);

    X_right.pos_previous=X_right.pos_current;
    X_right.pos_current=HAL_GPIO_ReadPin(DI_275_S29_m_GPIO_Port,DI_275_S29_m_Pin);

    Y_left.pos_previous=Y_left.pos_current;
    Y_left.pos_current=HAL_GPIO_ReadPin(DI_278_S32_m_GPIO_Port,DI_278_S32_m_Pin);

    Y_right.pos_previous=Y_right.pos_current;
    Y_right.pos_current=HAL_GPIO_ReadPin(DI_277_S31_m_GPIO_Port,DI_277_S31_m_Pin);

    Z_up.pos_previous=Z_up.pos_current;
    Z_up.pos_current=HAL_GPIO_ReadPin(DI_280_S34_m_GPIO_Port,DI_280_S34_m_Pin);

    Z_down.pos_previous=Z_down.pos_current;
    Z_down.pos_current=HAL_GPIO_ReadPin(DI_279_S33_m_GPIO_Port,DI_279_S33_m_Pin);
    
    limit_switches= HAL_GPIO_ReadPin(DI_246_m_GPIO_Port,DI_246_m_Pin)+HAL_GPIO_ReadPin(DI_253_m_GPIO_Port,DI_253_m_Pin)*2+HAL_GPIO_ReadPin(DI_252_m_GPIO_Port,DI_252_m_Pin)*4+HAL_GPIO_ReadPin(S27Y_endswitch1_GPIO_Port,S27Y_endswitch1_Pin)*8+HAL_GPIO_ReadPin(S25Z_endswitch2_GPIO_Port,S25Z_endswitch2_Pin)*16+HAL_GPIO_ReadPin(S28Z_endswitch1_GPIO_Port,S28Z_endswitch1_Pin)*32;
    
    
    
    
    if (HAL_GPIO_ReadPin(DI_82_S13_m_GPIO_Port,DI_82_S13_m_Pin)==GPIO_PIN_SET) //2t 4t feed control mode
    { 
      if ((X_left.pos_previous==GPIO_PIN_SET)&&(X_left.pos_current==GPIO_PIN_RESET))       
      {
        feed_motion=1;
        polyarity=1;
      }    
      if ((X_right.pos_previous==GPIO_PIN_SET)&&(X_right.pos_current==GPIO_PIN_RESET))  
      {
        feed_motion=2;
        polyarity=0;
      } 
      if ((Y_left.pos_previous==GPIO_PIN_SET)&&(Y_left.pos_current==GPIO_PIN_RESET))      
      {
        feed_motion=3;
        polyarity=1;
      }   
      if ((Y_right.pos_previous==GPIO_PIN_SET)&&(Y_right.pos_current==GPIO_PIN_RESET))     
      {
        feed_motion=4;
        polyarity=0;
      }     
      if ((Z_up.pos_previous==GPIO_PIN_SET)&&(Z_up.pos_current==GPIO_PIN_RESET))       
      {
        feed_motion=5;
        polyarity=1;
      } 
      if ((Z_down.pos_previous==GPIO_PIN_SET)&&(Z_down.pos_current==GPIO_PIN_RESET))     
      {
        feed_motion=6;
        polyarity=0;
      } 
    } else {
      if (X_left.pos_current==GPIO_PIN_SET)       
      {
        feed_motion=1;
        polyarity=1;
      }    
      if (X_right.pos_current==GPIO_PIN_SET)  
      {
        feed_motion=2;
        polyarity=0;
      } 
      if (Y_left.pos_current==GPIO_PIN_SET)      
      {
        feed_motion=3;
        polyarity=1;
      }   
      if (Y_right.pos_current==GPIO_PIN_SET)     
      {
        feed_motion=4;
        polyarity=0;
      }     
      if (Z_up.pos_current==GPIO_PIN_SET)       
      {
        feed_motion=5;
        polyarity=1;
      } 
      if (Z_down.pos_current==GPIO_PIN_SET)     
      {
        feed_motion=6;
        polyarity=0;
      }
      if (!(X_left.pos_current&X_right.pos_current&Y_left.pos_current&Y_right.pos_current&Z_up.pos_current&Z_down.pos_current))
      {
        feed_motion=0;
      }
    }
    

    Feed_Stop.pos_previous=Feed_Stop.pos_current;
    Feed_Stop.pos_current=HAL_GPIO_ReadPin(DI_260_S22_m_GPIO_Port,DI_260_S22_m_Pin);
    if (((Feed_Stop.pos_previous==GPIO_PIN_SET)&&(Feed_Stop.pos_current==GPIO_PIN_RESET))||(error_my!=0))         
    {
      feed_motion=0;
    }   
    
    
    switch (feed_motion) //check endswiches
    {
    case 0:
      HAL_GPIO_WritePin(DO_304_Feed_x_GPIO_Port,DO_304_Feed_x_Pin,GPIO_PIN_RESET);  //cluch engadement
      HAL_GPIO_WritePin(DO_306_Feed_z_GPIO_Port,DO_306_Feed_z_Pin,GPIO_PIN_RESET);  //cluch engadement
      HAL_GPIO_WritePin(DO_305_Feed_y_GPIO_Port,DO_305_Feed_y_Pin,GPIO_PIN_RESET);  //cluch engadement
      
      
      break;
    case 1:
      if (limit_switches&1){
        feed_motion=0;
      } else{
        HAL_GPIO_WritePin(led_X_lock_m_GPIO_Port,led_X_lock_m_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DO_308_Clamp_x_GPIO_Port,DO_308_Clamp_x_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DO_304_Feed_x_GPIO_Port,DO_304_Feed_x_Pin,GPIO_PIN_SET);  //cluch engadement
      }
      break;
    case 2:
      if (limit_switches&2){
        feed_motion=0;
      } else{
        HAL_GPIO_WritePin(led_X_lock_m_GPIO_Port,led_X_lock_m_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DO_308_Clamp_x_GPIO_Port,DO_308_Clamp_x_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DO_304_Feed_x_GPIO_Port,DO_304_Feed_x_Pin,GPIO_PIN_SET);  //cluch engadement
      }
      break;
    case 3:
      if (limit_switches&4){
        feed_motion=0;
      } else {        
        HAL_GPIO_WritePin(led_Y_lock_m_GPIO_Port,led_Y_lock_m_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DO_310_Clamp_y_GPIO_Port,DO_310_Clamp_y_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DO_305_Feed_y_GPIO_Port,DO_305_Feed_y_Pin,GPIO_PIN_SET);  //cluch engadement        
      }
      break;   
    case 4:
      if (limit_switches&8){
        feed_motion=0;
      } else {                
        HAL_GPIO_WritePin(led_Y_lock_m_GPIO_Port,led_Y_lock_m_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DO_310_Clamp_y_GPIO_Port,DO_310_Clamp_y_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DO_305_Feed_y_GPIO_Port,DO_305_Feed_y_Pin,GPIO_PIN_SET);  //cluch engadement         
      }
      break;
    case 5:
      if (limit_switches&16){
        feed_motion=0;
      } else {                
        HAL_GPIO_WritePin(led_Z_lock_m_GPIO_Port,led_Z_lock_m_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DO_312_Clamp_z_GPIO_Port,DO_312_Clamp_z_Pin,GPIO_PIN_RESET);  
        HAL_GPIO_WritePin(DO_306_Feed_z_GPIO_Port,DO_306_Feed_z_Pin,GPIO_PIN_SET); //cluch engadement            
      }  
      break;   
    case 6:
      if (limit_switches&32){
        feed_motion=0;
      } else {             
        HAL_GPIO_WritePin(led_Z_lock_m_GPIO_Port,led_Z_lock_m_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DO_312_Clamp_z_GPIO_Port,DO_312_Clamp_z_Pin,GPIO_PIN_RESET);  
        HAL_GPIO_WritePin(DO_306_Feed_z_GPIO_Port,DO_306_Feed_z_Pin,GPIO_PIN_SET);  //cluch engadement          
      }
      break;        
    }
    // feed speed I_M4
    
    for (i=0; i>1; i++){
      feed_sp_buf[i]=feed_sp_buf[i+1];
      I_M4_buf[i]=I_M4_buf[i+1];
    }
    feed_sp_buf[2]=dma[1]>4096?0:dma[1];
    feed_sp_temp=0;
    I_M4_buf[2]=dma[3]>4096?0:dma[3];
    I_M4_temp=0;
    for (i=0; i>2; i++){
      feed_sp_temp+=feed_sp_buf[i];
      I_M4_temp+=I_M4_buf[i];
    }
    I_M4=I_M4_temp/12; //0-1024 
    if (HAL_GPIO_ReadPin(DI_91_S19_m_GPIO_Port,DI_91_S19_m_Pin)==GPIO_PIN_SET)
    {
      feed_speed_sp=feed_sp_temp/120;    //0-1000 /3-buf /4-1024/ 10- 100
      HAL_GPIO_WritePin(led_2t_m_GPIO_Port,led_2t_m_Pin,GPIO_PIN_SET); 
    } else{
      feed_speed_sp=feed_sp_temp/12;    //0-1000 /3-buf /4-1024
      HAL_GPIO_WritePin(led_2t_m_GPIO_Port,led_2t_m_Pin,GPIO_PIN_RESET); 
    }
    if (HAL_GPIO_ReadPin(DI_92_S20_m_GPIO_Port,DI_92_S20_m_Pin)==GPIO_PIN_SET)
    {
      feed_speed_sp=1000;    //0-1000 /3-buf /4-1024/ 10- 100
    }
    //----------------------------------------//
    
    // gripe instrument
    if ((HAL_GPIO_ReadPin(S11Gripe_GPIO_Port,S11Gripe_Pin)==GPIO_PIN_SET)&&(M1_motion==0)&&(feed_motion==0))
    {
      HAL_GPIO_WritePin(DO_51_K6_GPIO_Port,DO_51_K6_Pin,GPIO_PIN_SET);
    } else{
      HAL_GPIO_WritePin(DO_51_K6_GPIO_Port,DO_51_K6_Pin,GPIO_PIN_RESET);
    }
    if ((HAL_GPIO_ReadPin(S12ungrip_GPIO_Port,S12ungrip_Pin)==GPIO_PIN_SET)&&(M1_motion==0)&&(feed_motion==0))
    {
      HAL_GPIO_WritePin(DO_63_K7_GPIO_Port,DO_63_K7_Pin,GPIO_PIN_SET);
    } else{
      HAL_GPIO_WritePin(DO_63_K7_GPIO_Port,DO_63_K7_Pin,GPIO_PIN_RESET);
    }        
    //----------------------------------------//
    
    
    
    // M1 on off, rocking motion
    M1_on.pos_previous=M1_on.pos_current;
    M1_on.pos_current=HAL_GPIO_ReadPin(DI_34_S7_m_GPIO_Port,DI_34_S7_m_Pin);
    if ((HAL_GPIO_ReadPin(S6Gearbox_GPIO_Port,S6Gearbox_Pin)==GPIO_PIN_SET)&&error_my==0)
    {
      
      if ((M1_on.pos_previous==GPIO_PIN_SET)&&(M1_on.pos_current==GPIO_PIN_RESET)&&(M1_motion==0))  
      { 
        if (HAL_GPIO_ReadPin(DI_34_S8_m_GPIO_Port,DI_34_S8_m_Pin)==GPIO_PIN_SET){
          M1_motion=1;
        } else{
          M1_motion=2;
        }
      }   
    } else {
      M1_motion=0;
    }
    Rocking_btn.pos_previous=Rocking_btn.pos_current;
    Rocking_btn.pos_current=HAL_GPIO_ReadPin(S2rocking_GPIO_Port,S2rocking_Pin);
    if ((Rocking_btn.pos_previous==GPIO_PIN_SET)&&(Rocking_btn.pos_current==GPIO_PIN_RESET)&&(error_my==0)&&(M1_motion==0))
    {
      M1_motion=3;  // zero when over (time based)
    }
    M1_off.pos_previous=M1_off.pos_current;
    M1_off.pos_current=HAL_GPIO_ReadPin(DI_29_S5_m_GPIO_Port,DI_29_S5_m_Pin);
    if ((M1_off.pos_previous==GPIO_PIN_SET)&&(M1_off.pos_current==GPIO_PIN_RESET))  
    {
      M1_motion=4;
    }
    //----------------------------------------//
    
    
    // pumps xyiamps error blocks everithyng    
    if (error_my==0){
      
      if (oil_pump_state!=0)
      {
        HAL_GPIO_WritePin(Q2_GPIO_Port,Q2_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(led_oil_GPIO_Port,led_oil_Pin,GPIO_PIN_SET);
      }else{
        oil_pump_state=0;
        HAL_GPIO_WritePin(Q2_GPIO_Port,Q2_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(led_oil_GPIO_Port,led_oil_Pin,GPIO_PIN_RESET);
      }
      //----------------------------------------//
      // coolant pump on
      if (HAL_GPIO_ReadPin(DI_63_S9_m_GPIO_Port,DI_63_S9_m_Pin)==GPIO_PIN_SET)
      {
        HAL_GPIO_WritePin(Q5_GPIO_Port,Q5_Pin,GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(Q5_GPIO_Port,Q5_Pin,GPIO_PIN_RESET);
      }
      //----------------------------------------//
      HAL_GPIO_WritePin(led_error_m_GPIO_Port,led_error_m_Pin, GPIO_PIN_RESET);
      
      
    }else{
      M1_motion=0;
      feed_speed_sp=0;
      HAL_GPIO_WritePin(Q5_GPIO_Port,Q5_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(Q2_GPIO_Port,Q2_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(led_oil_GPIO_Port,led_oil_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(led_Temper_GPIO_Port,led_Temper_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(led_error_m_GPIO_Port,led_error_m_Pin, GPIO_PIN_SET);
    }
    
    
    
    osDelay(50);  //20hz
    
    Deblock_btn.pos_previous=Deblock_btn.pos_current;
    Deblock_btn.pos_current=HAL_GPIO_ReadPin(DI_88_S16_m_GPIO_Port,DI_88_S16_m_Pin);
    if ((Deblock_btn.pos_previous==GPIO_PIN_SET)&&(Deblock_btn.pos_current==GPIO_PIN_RESET))  
    {
      error_my=0;
    }    

    //frequency check if interrupts not work and RPS less then 1 Reverse per/SEC motor max RPM 4000/60= 66 rps
    if ((TIM2->CNT)>10000)
    {
      TIM2->CNT=0;
      first_intrrupt=1;
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartComunication */
/**
* @brief Function implementing the Comunication thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartComunication */
void StartComunication(void *argument)
{
  /* USER CODE BEGIN StartComunication */
  uint8_t blynk=0,error_counter=0; 
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
    blynk++;
    if ((error_my)&&(blynk>29))
    {
      if (blynk==30+5*error_counter)
      {
        HAL_GPIO_TogglePin(led_manual_GPIO_Port,led_manual_Pin);
        error_counter++;
      }
      if (error_my==error_counter/2)
      {
        HAL_GPIO_WritePin(led_manual_GPIO_Port,led_manual_Pin,GPIO_PIN_RESET);
        error_counter=0;
      }
    }
    else {
      HAL_GPIO_WritePin(led_manual_GPIO_Port,led_manual_Pin,GPIO_PIN_RESET);
    }
  }
  /* USER CODE END StartComunication */
}

/* USER CODE BEGIN Header_StartFeed */
/**
* @brief Function implementing the feed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFeed */
void StartFeed(void *argument)
{
  /* USER CODE BEGIN StartFeed */
  uint8_t startuem=0, M1_motion_prev,iter; 
  uint16_t i=0, Freqency;  //frequency 0-1000 -0 RPM 1000-4000 RPM
  int16_t errorsold1[256]={0};
  int16_t  Error1;
  int32_t regD1=0,regP1=0,regI1=0,PID1;
  /* Infinite loop */
  for(;;)
  {
    //--rocking motion actually not rocking---//
    if ((M1_motion==3)&&(i<Roking_time)){                        
      HAL_GPIO_WritePin(Q4_GPIO_Port,Q4_Pin, GPIO_PIN_RESET);  
      HAL_GPIO_WritePin(Q3_GPIO_Port,Q3_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(Q6_GPIO_Port,Q6_Pin, GPIO_PIN_RESET);
      PWM_M1=1000-M1_start_PWM;  
      i++;
    }else if((M1_motion==3)&&(i>=Roking_time))
    {
      HAL_GPIO_WritePin(Q4_GPIO_Port,Q4_Pin, GPIO_PIN_RESET);  
      HAL_GPIO_WritePin(Q3_GPIO_Port,Q3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(Q6_GPIO_Port,Q6_Pin, GPIO_PIN_RESET);
      PWM_M1=1000;      
      M1_motion=0;
      i=0;
    }
    //----------------------------------------//   
    
    //---M1 left right------------------------//
    if (M1_motion==1&&M1_motion_prev==0) //right
    {
      i=0;
      PWM_M1=1000;  
      HAL_GPIO_WritePin(Q4_GPIO_Port,Q4_Pin, GPIO_PIN_RESET);  
      HAL_GPIO_WritePin(Q3_GPIO_Port,Q3_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(Q6_GPIO_Port,Q6_Pin, GPIO_PIN_RESET);
      startuem=1;
    }    
    if (M1_motion==2&&M1_motion_prev==0)  //left
    {
      i=0;
      PWM_M1=1000;  
      HAL_GPIO_WritePin(Q4_GPIO_Port,Q4_Pin, GPIO_PIN_SET);  
      HAL_GPIO_WritePin(Q3_GPIO_Port,Q3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(Q6_GPIO_Port,Q6_Pin, GPIO_PIN_RESET);
      startuem=1;
    }    
    if(startuem==1&&i<Acceleration_time)
    {
      
      PWM_M1=1000-M1_start_PWM+(600/Acceleration_time)*i;  //max 90%
      i++;
    }else if(startuem==1&&i>=Acceleration_time) {
      i=0;
      PWM_M1=0;
    }
    //----------------------------------------//
    
    //---M1 brake------------------------//
    if ((M1_motion_prev==1||M1_motion_prev==2)&&M1_motion==4)
    {
      i=0;
      PWM_M1=1000;  
      HAL_GPIO_WritePin(Q4_GPIO_Port,Q4_Pin, GPIO_PIN_RESET);  
      HAL_GPIO_WritePin(Q3_GPIO_Port,Q3_Pin, GPIO_PIN_RESET);
    }
    if (M1_motion==4&&i<Brake_time)
    {
      HAL_GPIO_WritePin(Q6_GPIO_Port,Q6_Pin, GPIO_PIN_SET);
      i++;
    }else if((M1_motion==4)&&(i>=Brake_time))
    {
      HAL_GPIO_WritePin(Q6_GPIO_Port,Q6_Pin, GPIO_PIN_RESET);
      i=0;
      M1_motion=0;
    }
    //----------------------------------------//    
    
    //----------NA VSYAKII SLYCHAI---------//
    if (M1_motion==0)
    {
      i=0;
      PWM_M1=1000;  
      HAL_GPIO_WritePin(Q4_GPIO_Port,Q4_Pin, GPIO_PIN_RESET);  
      HAL_GPIO_WritePin(Q3_GPIO_Port,Q3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(Q6_GPIO_Port,Q6_Pin, GPIO_PIN_RESET);
      startuem=0;
    }
    //-------------------------------------//


    //-------------------------FEED-----------------------------//
    if (Freq_TIM2>10000){
      Freq_TIM2=10000;
    }else if (Freq_TIM2<130){
      error_my=6; //very hight frequency 
    }
    Freqency=(uint16_t)(1016-Freq_TIM2/10); //os    
    //enable
    if (feed_motion==0){
      HAL_GPIO_WritePin(Q7_GPIO_Port,Q7_Pin, GPIO_PIN_RESET); 
    }else{
      HAL_GPIO_WritePin(Q7_GPIO_Port,Q7_Pin, GPIO_PIN_SET); 
      //-------------------pid1------------------------------------------//   
      Error1=feed_speed_sp-Freqency;    //0-1000 zadanie 16-1003 feedback
      regI1=0;
      for (iter=0; iter<I1; iter++)  
      {
        regI1=regI1+errorsold1[iter];
      } 
      regP1=Error1*P1;  
      regI1 =(I1>0)? (regI1 + Error1):0; 
      regD1=(Error1-errorsold1[0])*D1; 
      if (regD1<(-10000))
      {
        regD1=(-10000);
      }
      if (regD1>(10000))
      {
        regD1=(10000);
      }
      if (regI1<(-10000))
      {
        regI1=(-10000);
      }
      if (regI1>(10000))
      {
        regI1=(10000);
      }
      for (iter=254; iter>0; iter--)
      {
        errorsold1[iter+1]=errorsold1[iter];
      } 
      errorsold1[0]=Error1;
      if ((Error1>=dead_zone1)||(dead_zone1==0)||(Error1<=(dead_zone1*(-1))) )  
      {
        
        if (direct_pid==0){
          PID1=regP1+regI1+regD1;
        }else{
          PID1=(regP1+regI1+regD1)*(-1);
        }
        if (PID1<0)
        {
          PID1=0;
        }
        if (PID1>=10000)
        {
          PID1=10000;
        }
        if((PID1>=0)&&(PID1<10000)&&(I_M4<Hi_Current_lim))  //current limit
        {
          PWM_M4=1999-PID1/10; //output
        }else if((I_M4>=Hi_Current_lim)&&(PWM_M4<1990))
        {
          PWM_M4=PWM_M4+10;
        }
        
      }
      
      if (I_M4>Hi_Current_error){
        error_my=7; //very hight currnet 
      }    
      //---------------------------------------------------------//          
      
    }

    if ((polyarity==1)&&(HAL_GPIO_ReadPin(tacho_dir_right_GPIO_Port,tacho_dir_right_Pin)==GPIO_PIN_SET)&&(I_M4>Rotation_lim))//left and actual right
    {
      error_my=8; //something wrong with direction left 
    }
    if ((polyarity==0)&&(HAL_GPIO_ReadPin(tacho_dir_left_GPIO_Port,tacho_dir_left_Pin)==GPIO_PIN_SET)&&(I_M4>Rotation_lim))
    {
      error_my=9; //something wrong with direction right
    }
    
    //----------------------------------------------------------//
    osDelay(5);  //200 hz
    M1_motion_prev=M1_motion;
    
  }  

  /* USER CODE END StartFeed */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
