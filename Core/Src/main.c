/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
//#include <stdio.h>
#include <math.h>

#include <ledImplementation.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#pragma GCC push_options
#pragma GCC optimize ("O3")
void delayUS(uint32_t us) {
	volatile uint32_t cycles = (SystemCoreClock/1000000L)*us;
	volatile uint32_t start = DWT->CYCCNT;

	do {
	} while(DWT->CYCCNT - start < cycles);
}
#pragma GCC pop_options

int map(int st1, int fn1, int st2, int fn2, int value)
{
    return (1.0*(value-st1))/((fn1-st1)*1.0) * (fn2-st2)+st2;
}


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId defaultTaskHandle;
osThreadId uartSendTaskHandle;
osThreadId uartReceiveTaskHandle;
osThreadId mainCalculationHandle;
osSemaphoreId uartReceiveDoneSemHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);
void uartSendTaskFun(void const * argument);
void uartReceiveTaskFun(void const * argument);
void mainCalculationFun(void const * argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char trans_str[64] = {0,};

static uint16_t rxLen = 16;
static uint8_t rxBuff[16];

static uint16_t txLen = 16;
static uint8_t txBuff[16];

uint16_t angle = 0;
uint16_t radius = 0;
static int pan = 90;
static int tilt = 20;

uint16_t distance = 0;
uint16_t wall = 15;

static int X = 0;
static int Y = 0;
static int L = 0;

static int FB_PWM = 0;
static int TURN_PWM = 0;

uint16_t FR = 0;
uint16_t FL = 0;
uint16_t BR = 0;
uint16_t BL = 0;

uint16_t TIM3_CNT = 0;
uint8_t TAHO_HOLES = 20;
uint16_t FR_TAHO_PULSES = 0;
uint16_t FL_TAHO_PULSES = 0;
uint16_t FR_RPM = 0;
uint16_t FL_RPM = 0;
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
  SEGGER_SYSVIEW_Conf();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  SEGGER_RTT_printf(0, "Board Init\n");

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_UART_Receive_IT(&huart2, rxBuff, rxLen);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of uartReceiveDoneSem */
  osSemaphoreDef(uartReceiveDoneSem);
  uartReceiveDoneSemHandle = osSemaphoreCreate(osSemaphore(uartReceiveDoneSem), 1);

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of uartSendTask */
  osThreadDef(uartSendTask, uartSendTaskFun, osPriorityAboveNormal, 0, 128);
  uartSendTaskHandle = osThreadCreate(osThread(uartSendTask), NULL);

  /* definition and creation of uartReceiveTask */
  osThreadDef(uartReceiveTask, uartReceiveTaskFun, osPriorityNormal, 0, 128);
  uartReceiveTaskHandle = osThreadCreate(osThread(uartReceiveTask), NULL);

  /* definition and creation of mainCalculation */
  osThreadDef(mainCalculation, mainCalculationFun, osPriorityNormal, 0, 128);
  mainCalculationHandle = osThreadCreate(osThread(mainCalculation), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  /* EXTI4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 719;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_HCSR04_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 GPIO_HCSR04_TRIG_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_HCSR04_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_FR_TAHO_Pin GPIO_FL_TAHO_Pin */
  GPIO_InitStruct.Pin = GPIO_FR_TAHO_Pin|GPIO_FL_TAHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Функция формирует импульс Trig для HCSR04
void GetDistance()
{
	HAL_TIM_IC_Start_IT(&HTIM_HCSR04, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&HTIM_HCSR04, TIM_CHANNEL_2);

	//HAL_TIM_Base_Start(&htim4);
	HAL_GPIO_WritePin(GPIO_HCSR04_TRIG_GPIO_Port, GPIO_HCSR04_TRIG_Pin, GPIO_PIN_SET);
	delayUS(10); //measure upper in 5us
	//HAL_Delay(30); //measure upper in 1ms
	HAL_GPIO_WritePin(GPIO_HCSR04_TRIG_GPIO_Port, GPIO_HCSR04_TRIG_Pin, GPIO_PIN_RESET);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) // колбек по захвату
{
	if(htim->Instance == TIM4)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // RISING с LOW на HIGH
		{
			__HAL_TIM_SET_COUNTER(&HTIM_HCSR04, 0x0000); // обнуление счётчика
		}

		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // FALLING с HIGH на LOW
		{
			uint32_t cnt = HAL_TIM_ReadCapturedValue(&HTIM_HCSR04, TIM_CHANNEL_2); // чтение значения в регистре захвата/сравнения
			distance = cnt*17/1000;
			//snprintf(trans_str, 63, "Pulse %lu cm\n", cnt);
			//HAL_UART_Transmit(&huart2, (uint8_t*)trans_str, strlen(trans_str), 1000);
			HAL_TIM_IC_Stop_IT(&HTIM_HCSR04, TIM_CHANNEL_1);
			HAL_TIM_IC_Stop_IT(&HTIM_HCSR04, TIM_CHANNEL_2);
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // колбек по захвату
{
	if(htim->Instance == TIM3)
	{
		TIM3_CNT++;

		if (TIM3_CNT >= 1000){
			FR_RPM = (FR_TAHO_PULSES * 60) / (TAHO_HOLES);
			FL_RPM = (FL_TAHO_PULSES * 60) / (TAHO_HOLES);
			FR_TAHO_PULSES = 0;
			FL_TAHO_PULSES = 0;
			TIM3_CNT = 0;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin== GPIO_FR_TAHO_Pin) {
		FR_TAHO_PULSES++;
	}
	if(GPIO_Pin== GPIO_FL_TAHO_Pin) {
		FL_TAHO_PULSES++;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	SEGGER_SYSVIEW_RecordEnterISR();
	if (huart->Instance == USART2)
	{
		xSemaphoreGiveFromISR(uartReceiveDoneSemHandle, &xHigherPriorityTaskWoken);
	}
	SEGGER_SYSVIEW_RecordExitISR();
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

	  /* Infinite loop */
	  for(;;)
	  {
		BlueLed.Toggle();

	    SEGGER_RTT_printf(0, "Blink!!!\n\r");

	    osDelay(500);
	  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_uartSendTaskFun */
/**
* @brief Function implementing the uartSendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uartSendTaskFun */
void uartSendTaskFun(void const * argument)
{
  /* USER CODE BEGIN uartSendTaskFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END uartSendTaskFun */
}

/* USER CODE BEGIN Header_uartReceiveTaskFun */
/**
* @brief Function implementing the uartReceiveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uartReceiveTaskFun */
void uartReceiveTaskFun(void const * argument)
{
  /* USER CODE BEGIN uartReceiveTaskFun */


  /* Infinite loop */
  for(;;)
  {
	if(xSemaphoreTake(uartReceiveDoneSemHandle, 300) == pdPASS)
	{
		//SEGGER_SYSVIEW_PrintfHost("uartReceive: ");
		HAL_StatusTypeDef ret = HAL_UART_Receive_DMA(&huart2, rxBuff, rxLen);
		if (ret == HAL_OK)
		{
			//SEGGER_SYSVIEW_Print(rxBuff);
			uint32_t receivedCRC = 0;
			for(int i=0; i<rxLen-3; i++){
				receivedCRC = receivedCRC + rxBuff[i];
			}

		    if ((rxBuff[0] == 0x02) & (rxBuff[rxLen-1] == 0x03) & (((receivedCRC & 0xFF00) >> 8) == rxBuff[rxLen-3]) & ((receivedCRC & 0xFF) == rxBuff[rxLen-2]))
		    {
		    	angle = ((rxBuff[1] - 0x30) * 100 + (rxBuff[2] - 0x30) * 10 + rxBuff[3] - 0x30);
		    	radius = ((rxBuff[4] - 0x30) * 100 + (rxBuff[5] - 0x30) * 10 + rxBuff[6] - 0x30);
		    	pan = ((rxBuff[7] - 0x30) * 100 + (rxBuff[8] - 0x30) * 10 + rxBuff[9] - 0x30);
		    	tilt = ((rxBuff[10] - 0x30) * 100 + (rxBuff[11] - 0x30) * 10 + rxBuff[12] - 0x30);
		    } else {
		    	SEGGER_SYSVIEW_PrintfHost("receivedCRC wrong");
		    }
		    //SEGGER_SYSVIEW_PrintfHost("%u", receivedCRC);

			txBuff[0] = 0x02;
			for(int i=1; i<txLen-3; i++){
				txBuff[i] = rxBuff[i];
			}
			uint32_t transmitCRC = 0;
			for(int i=0; i<txLen-3; i++){
				transmitCRC = transmitCRC + txBuff[i];
			}
			txBuff[txLen-3] = (transmitCRC & 0xFF00) >> 8;
			txBuff[txLen-2] = (transmitCRC & 0xFF);
			txBuff[txLen-1] = 0x03;

		    HAL_UART_Transmit_DMA(&huart2, txBuff, txLen);
		    //SEGGER_SYSVIEW_PrintfHost("uartTransmit: ");
		    //SEGGER_SYSVIEW_PrintfHost((char*)txBuff);
		}
	}
	//else
	//{
	//	SEGGER_SYSVIEW_PrintfHost("uartReceive timeout");
	//}

	osDelay(1);
  }
  /* USER CODE END uartReceiveTaskFun */
}

/* USER CODE BEGIN Header_mainCalculationFun */
/**
* @brief Function implementing the mainCalculation thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mainCalculationFun */
void mainCalculationFun(void const * argument)
{
  /* USER CODE BEGIN mainCalculationFun */
  /* Infinite loop */
  for(;;)
  {
	GetDistance();

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, map(0, 180, 250, 50, pan));
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, map(0, 100, 55, 145, tilt));

	X = (int)(radius*cos(angle*2*3.14/360));
	Y = (int)(radius*sin(angle*2*3.14/360));
	L = radius;

	/*
	if (Y > 70){
		wall = 30;
	}

	if (distance <= wall){ // in cm
		if (Y > 0) {
		   Y = 0;
		   L = X;
		}
	}
	*/

	float proportion_koef = (__HAL_TIM_GET_AUTORELOAD(&htim3) + 1)/100;

	if (Y == 0) {
		FB_PWM = 0;
	} else {
		FB_PWM = map(0, 100, 50, 100, abs(Y))*proportion_koef;
	}
	if (L == 0) {
		TURN_PWM = 0;
	} else {
		TURN_PWM = map(0, 100, 50, 100, abs(L))*proportion_koef;
	}

	SEGGER_SYSVIEW_PrintfHost("angle %i,  radius %i,  pan %d,  tilt %d,  distance %i,  X %i,  Y %i,  L %i,  FB_PWM %i,  TURN_PWM %i,  FR_RPM %i,  FL_RPM %i",
								angle, radius, pan, tilt, distance, X, Y, L, FB_PWM, TURN_PWM, FR_RPM, FL_RPM);

	// ForwardRight BackwardRight ForwardLeft BackwardLeft
	if (Y >= 0){
		if (X >= 0){
			FR = FB_PWM;
			BR = 0;
			FL = TURN_PWM;
			BL = 0;
		}
		else {
			FR = TURN_PWM;
			BR = 0;
			FL = FB_PWM;
			BL = 0;
		}
	} else {
		if (X >= 0){
			FR = 0;
			BR = FB_PWM;
			FL = 0;
			BL = TURN_PWM;
		}
		else {
			FR = 0;
			BR = TURN_PWM;
			FL = 0;
			BL = FB_PWM;
		}
	}

	__HAL_TIM_SET_COMPARE(&HTIM_PWM, FL_TIM_CHANNEL, FL); //FR
	__HAL_TIM_SET_COMPARE(&HTIM_PWM, BL_TIM_CHANNEL, BL); //BR
	__HAL_TIM_SET_COMPARE(&HTIM_PWM, FR_TIM_CHANNEL, FR); //FL
	__HAL_TIM_SET_COMPARE(&HTIM_PWM, BR_TIM_CHANNEL, BR); //BL

	osDelay(1000);
  }
  /* USER CODE END mainCalculationFun */
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
