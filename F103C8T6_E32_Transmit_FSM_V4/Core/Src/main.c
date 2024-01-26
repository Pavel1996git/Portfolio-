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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "FLASH_PAGE_F1.h"
#include "string.h"
#include "fsm.h"
#include "ssd1306.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t rxByteSize;

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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
// Переменные типа uint8_t
uint8_t rxByteSize; // Размер буфера приема (не инициализирован)
uint8_t rxByteSize = RX_BUF_SIZE0; // Размер буфера приема (инициализирован)
uint8_t receiveBuffer[RX_BUF_SIZE1]; // Буфер приема данных

uint8_t txByteDemolition[] = BYTE_PULT_RESIV_DEMOLITION; // Байты для передачи сигнала "demolition"
const uint8_t txByteTest[] = BYTE_PULT_RESIV_TEST; // Байты для передачи сигнала "test"
const uint8_t txByteSetup0[] = {0xC0, 0xFF, 0xFF, 0x18, 0x17, 0x44}; // Байты настройки 0
const uint8_t txByteSetup1[] = {0xC1, 0xC1, 0xC1}; // Байты настройки 1
const uint8_t rxByteLog[] = BYTE_PULT_RESIV_LOG; // Байты для приема лога
const uint8_t rxByteTest[] = BYTE_PULT_RESIV_TEST; // Байты для приема сигнала "test"

uint8_t flagInterrupt = RESET; // Флаг прерывания
uint8_t flagButtonDemolition = RESET; // Флаг кнопки "demolition"
uint8_t flagButtonTest = RESET; // Флаг кнопки "test"
uint8_t flagButtonDemolitionUpdateDisplay = RESET; // Флаг обновления дисплея при нажатии "demolition"
uint8_t countButtonDemolitionClock; // Счетчик времени при нажатии "demolition"
uint8_t flagButtonTestUpdateDisplay = RESET; // Флаг обновления дисплея при нажатии "test"
uint8_t countButtonTestClock; // Счетчик времени при нажатии "test"
uint8_t flagPeriodic1 = RESET; // Флаг периодического события 1
uint8_t flagPeriodic2 = RESET; // Флаг периодического события 2

// Переменные типа uint16_t
uint16_t adcData[ADC_CHANNELS_NUM]; // Массив данных от АЦП

// Переменные типа перечисления
Tx txStatus = txNoTx; // Состояние передачи

// Переменные указателя на функцию void
void (*outDispl)(void); // Указатель на функцию обновления дисплея

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void (*outDispl)(void);


char* intToChar(uint8_t int_, uint8_t n);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Массив структур, представляющих точки переходов конечного автомата
tag_point FSM[NUMBER_STATE]={
		{2, 0, startFsm, readInterrupt, waitShort, {STATE_ERROR_FOREWER, STATE_BUTTON_CHOESE_DETONATOR, STATE_RX_MESSAGE, STATE_TX_MESSAGE, STATE_BUTTON_TEST, STATE_BUTTON_DEMOLITION, STATE_BUTTON_MENU, STATE_WAIT_FOREVER, STATE_WAIT_MENU, STATE_WAIT_TX, STATE_START, STATE_WAIT}},// STATE_START
		{0, 3, outError, ForewerHandlerError, waitShort, {STATE_START, STATE_ERROR_FOREWER, STATE_ERROR_FOREWER, STATE_ERROR_FOREWER, STATE_ERROR_FOREWER, STATE_ERROR_FOREWER, STATE_ERROR_FOREWER, STATE_ERROR_FOREWER, STATE_ERROR_FOREWER, STATE_ERROR_FOREWER, STATE_ERROR_FOREWER, STATE_ERROR_FOREWER}},// STATE_ERROR_FOREWER
		{3, 0, outDisplay, readInterrupt, waitLong, {STATE_ERROR_FOREWER, STATE_BUTTON_CHOESE_DETONATOR, STATE_RX_MESSAGE, STATE_TX_MESSAGE, STATE_BUTTON_TEST, STATE_BUTTON_DEMOLITION, STATE_BUTTON_MENU, STATE_WAIT_FOREVER, STATE_WAIT_MENU, STATE_WAIT_TX, STATE_START, STATE_WAIT}},// STATE_WAIT
		{0, 0, outBufferDisplay, readInterrupt, waitShort, {STATE_ERROR_FOREWER, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT_FOREVER, STATE_WAIT}},// STATE_BUTTON CHOESE_DETONATOR
		{0, 0, outBufferDisplay, readInterrupt, waitShort, {STATE_ERROR_FOREWER, STATE_WAIT_MENU, STATE_WAIT_MENU, STATE_WAIT_MENU, STATE_WAIT_MENU, STATE_WAIT_MENU, STATE_WAIT_MENU, STATE_WAIT_MENU, STATE_WAIT_MENU, STATE_WAIT_MENU, STATE_WAIT_MENU, STATE_WAIT_MENU}},// STATE_BUTTON_MENU
		{2, 0, outRxMessage, handlerRxMessage, waitShort, {STATE_ERROR_FOREWER, STATE_WAIT, STATE_RX_MESSAGE, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT_TX, STATE_WAIT_FOREVER, STATE_WAIT}},// STATE_RX_MESSAGE
		{0, 0, outTxMessage, readInterrupt, waitShort, {STATE_ERROR_FOREWER, STATE_WAIT,  STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT_TX, STATE_WAIT_TX, STATE_WAIT_TX, STATE_WAIT_FOREVER, STATE_WAIT}},// STATE_TX_MESSAGE
		{3, 0, outBlank, readInterrupt, waitForever, {STATE_ERROR_FOREWER, STATE_BUTTON_CHOESE_DETONATOR, STATE_RX_MESSAGE, STATE_TX_MESSAGE, STATE_BUTTON_TEST, STATE_BUTTON_DEMOLITION, STATE_BUTTON_MENU, STATE_WAIT_FOREVER, STATE_WAIT_MENU, STATE_WAIT_TX, STATE_START, STATE_WAIT}},// STATE_WAIT_FOREVER
		{1, 0, outDisplay, readInterrupt, waitLongMenu, {STATE_ERROR_FOREWER, STATE_TX_MESSAGE, STATE_WAIT_FOREVER, STATE_WAIT_MENU, STATE_START, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT}},// STATE_WAIT_MENU
		{0, 0, outBufferDisplay, readInterrupt, waitShort, {STATE_ERROR_FOREWER, STATE_WAIT, STATE_WAIT_FOREVER, STATE_WAIT_MENU, STATE_TX_MESSAGE, STATE_TX_MESSAGE, STATE_TX_MESSAGE, STATE_TX_MESSAGE, STATE_TX_MESSAGE, STATE_TX_MESSAGE, STATE_TX_MESSAGE, STATE_TX_MESSAGE}},// STATE_BUTTON_TEST
		{0, 0, outBufferDisplay, readInterrupt, waitShort, {STATE_ERROR_FOREWER, STATE_WAIT, STATE_WAIT_FOREVER, STATE_WAIT_MENU, STATE_TX_MESSAGE, STATE_TX_MESSAGE, STATE_TX_MESSAGE, STATE_TX_MESSAGE, STATE_TX_MESSAGE, STATE_TX_MESSAGE, STATE_TX_MESSAGE, STATE_TX_MESSAGE}},// STATE_BUTTON_DEMOLITION
		{2, 0, outDisplay, readInterrupt, waitLongTx, {STATE_ERROR_FOREWER, STATE_BUTTON_CHOESE_DETONATOR, STATE_RX_MESSAGE, STATE_TX_MESSAGE, STATE_BUTTON_TEST, STATE_BUTTON_DEMOLITION, STATE_BUTTON_MENU, STATE_WAIT_FOREVER, STATE_WAIT_MENU, STATE_WAIT_TX, STATE_START, STATE_WAIT}}// STATE_WAIT_TX
};
// Текущее состояние конечного автомата
uint32_t currentState = STATE_START;
// Входные данные
uint32_t input;

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  FSM[currentState].out(); //Вызов функции out() для текущего состояния конечного автомата
	  setInterruptBeginState(FSM[currentState].interruptBeginState); //Установка начального состояния для прерывания
	  FSM[currentState].time(); //Вызов функции time() для управления временем ожидания
	  setInterruptEndState(FSM[currentState].interruptEndState); //Установка конечного состояния для прерывания
	  input = FSM[currentState].in(); //Получение входных данных с помощью функции in()
	  currentState = FSM[currentState].next[input]; //переход в следующее состояние
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 15000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PIN_LED_Pin|PIN_POWER_DISPLAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PIN_M0_Pin|PIN_M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PIN_LED_Pin PIN_POWER_DISPLAY_Pin */
  GPIO_InitStruct.Pin = PIN_LED_Pin|PIN_POWER_DISPLAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN_M0_Pin PIN_M1_Pin */
  GPIO_InitStruct.Pin = PIN_M0_Pin|PIN_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN_AUX_Pin */
  GPIO_InitStruct.Pin = PIN_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PIN_AUX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN_BUTTON_10_Pin PIN_BUTTON_11_Pin PIN_BUTTON_12_Pin PIN_BUTTON_13_Pin
                           PIN_BUTTON_14_Pin */
  GPIO_InitStruct.Pin = PIN_BUTTON_10_Pin|PIN_BUTTON_11_Pin|PIN_BUTTON_12_Pin|PIN_BUTTON_13_Pin
                          |PIN_BUTTON_14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN_BUTTON_DEMOLITION_Pin PIN_BUTTON_TEST_Pin */
  GPIO_InitStruct.Pin = PIN_BUTTON_DEMOLITION_Pin|PIN_BUTTON_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN_BUTTON_ARM_Pin */
  GPIO_InitStruct.Pin = PIN_BUTTON_ARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PIN_BUTTON_ARM_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//=========================================================================================================================================//
//=================================================startFsm=================================================================//
//=========================================================================================================================================//

void startFsm(void) {
#ifdef Debug
    printf("stnvartstart: \n"); // Вывод отладочной информации в режиме отладки
#endif
    setInterruptState(UsaNoExtern); // Установка состояния прерывания
    HAL_GPIO_WritePin(PIN_LED_GPIO_Port, PIN_LED_Pin, GPIO_PIN_RESET); // Сброс состояния пина LED
    if(e32PinSetup(sleepMode)) // Инициализация E32 в режиме сна
        handlerError(e32FunctionInit); // Обработка ошибки инициализации E32 в режиме сна
    HAL_Delay(1000); // Задержка 1000 миллисекунд
    HAL_UART_Transmit_DMA(&huart1, &txByteSetup0, sizeof(txByteSetup0)); // Отправка установочных байтов по UART в режиме DMA (Direct Memory Access)
    HAL_Delay(500); // Задержка 500 миллисекунд
    HAL_UART_Transmit_DMA(&huart1, &txByteSetup1, sizeof(txByteSetup1)); // Отправка проверки исправности e32 по UART в режиме DMA
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveBuffer, rxByteSize); // Перевод UART в режим приема с использованием DMA

    if(checkRxStartMessage(txByteSetup0, rxByteSize)) // Проверка корректности работы е32
        handlerError(startE32Error); // Обработка ошибки стартового сообщения E32
    rxByteSize = sizeof(txByteDemolition); // Установка нового размера буфера приема для E32
    HAL_GPIO_WritePin(PIN_POWER_DISPLAY_GPIO_Port, PIN_POWER_DISPLAY_Pin, SET); // Установка состояния пина питания дисплея
    HAL_Delay(1000); // Задержка 1000 миллисекунд
    SSD1306_Init(); // Инициализация дисплея SSD1306
    if (HAL_I2C_IsDeviceReady(&hi2c1, SSD1306_I2C_ADDR, 1, 20000) != HAL_OK) { // Проверка доступности дисплея по I2C
        /* Return false */
        handlerError(startDisplayError); // Обработка ошибки инициализации дисплея
        return 0;
    }
    outDispl = outDisplayUser; // Установка функции вывода на дисплей
    outBufferDisplay(); // Обработка буфера дисплея
    e32PinSetup(powerSavingMode); // Инициализация пинов E32 в режим энергосбережения
    setInterruptState(UsaExtern); // Установка состояния прерывания
    nextState = wait; // следующее состояние автомата

    HAL_Delay(1); // Задержка 1 миллисекунда
}



//============================================================================================================================//
//================================================outTxMessage================================================================//
//============================================================================================================================//

/**
 * @brief Обрабатывает передачу сообщений в модуль передатчика.
 *
 * Эта функция управляет передачей сообщений в модуль передатчика.
 * Она настраивает модуль E32 для передачи и отправляет соответствующее сообщение.
 * Функция также обновляет дисплей и обрабатывает ошибки.
 */
void outTxMessage(void) {
    //HAL_Delay(NUMBER_NOD * 2000);
#ifdef Debug
    printf(" outTxMessage.\n"); // Отладочный вывод
#endif
    switch (txStatus) {
    case txNoTx: // Ошибка логики программы
#ifdef Debug
        printf("ERROR logic outTxMessage.\n"); // Отладочный вывод
#endif
        handlerError(txLogicError); // Обработчик ошибок
        break;
    case txTest: // Было прерывание кнопки тест
        if (flagButtonTest == RESET) {
#ifdef Debug
            printf("ERROR outTxMessage.\n"); // Ошибка вызова функции
#endif
            handlerError(txIntruptError); // Обработчик ошибок
        }
        e32PinSetup(wakeUpMode); // Настройка модуля E32 в режим пробуждения приемников
        HAL_Delay(10);
        HAL_UART_Transmit_DMA(&huart1, &txByteTest, sizeof(txByteTest)); // Отправка тестового сообщения
        HAL_Delay(100);
        bufferDisplay.timeTxWake = (TIME_TX_DEMOLITION_WEAKE) + ((TIME_TX_TEST_1_WEAKE) * (NUMBER_EXPLODER)); // Установка времени ожидания для дисплея режима тест
        txStatus = txNoTx; // Сброс статуса передачи
        break;
    case txDemolition: // Было прерывание кнопки демолитион
        if (flagButtonDemolition == RESET) {
#ifdef Debug
            printf("ERROR outTxMessage.\n"); // Отладочный вывод
#endif
            handlerError(txIntruptError); // Обработчик ошибок
        }

        if (bufferDisplay.bfArm == 0) { // Если не нажата кнопка АРМ - выход из функции
            return;
        }

        e32PinSetup(wakeUpMode); // Настройка модуля E32 в режим пробуждения приемников
        HAL_Delay(10);
        txByteDemolition[ADRES_NUMBER_BYTE_DEMOLITION_ON_EXPLODER] = 0; // Сбрасываем байты адресов выбранных приемников.
        for (uint8_t i = 0; i < NUMBER_EXPLODER; i++) {
            int8_t x = 1;
            if (bufferDisplay.bfDetonator[i] == 1) {
                x = x << (i + 1); // Битовыми сдвигами формируем байт из адресов приемников
                txByteDemolition[ADRES_NUMBER_BYTE_DEMOLITION_ON_EXPLODER] = txByteDemolition[ADRES_NUMBER_BYTE_DEMOLITION_ON_EXPLODER] | x; // Формирование байта, указывающего на подключенные взрыватели
            }
        }
        HAL_UART_Transmit_DMA(&huart1, &txByteDemolition, sizeof(txByteDemolition)); // Отправка сообщения о взрыве
        HAL_Delay(100);
        bufferDisplay.timeTxWake = TIME_TX_DEMOLITION_WEAKE; // Установка времени ожидания для дисплея режима взрыва
        txStatus = txNoTx; // Сброс статуса передачи
        break;
    default: // Ошибка логики программы
        handlerError(txLogicError); // Обработчик ошибок
#ifdef Debug
        printf("ERROR outTxMessage.\n"); // Отладочный вывод
#endif
        break;
    }

    outDispl = outDisplayWakeTx; // Установка указателя на функцию для работы дисплея в режиме ожидания
    nextState = waitTx; // Следующее состояние конечного автомата
    HAL_Delay(1);
}

//=============================================================================================================================//
//=====================================================outDisplay==============================================================//
//=============================================================================================================================//
//В этой инструкции по указателю вызывается нужная функция обновление дисплея.
void outDisplay(void){
	HAL_Delay(1);
	outDispl();
	}
//===============================================================================================================================//
//=================================================readPower()===============================================================//
//===============================================================================================================================//
/**
 * @brief Считывает данные о напряжении питания.
 *
 * Эта функция запускает измерение напряжения питания с помощью АЦП
 * и обрабатывает результаты. В случае обнаружения ошибки в напряжении,
 * функция выводит сообщение об ошибке.
 */
void readPower() {
#ifdef Debug
    printf("readPower().\n"); // Отладочный вывод
#endif
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcData, ADC_CHANNELS_NUM); // Запуск измерения напряжения
    HAL_Delay(50);  // Задержка перед возвратом результата
    if (adcVoltage[0] > 50 || adcVoltage[0] < 10) {
#ifdef Debug
        printf("ErrorReadVolt.\n"); // Отладочный вывод
#endif
        handlerError(readVoltageError);
    }
}
//============================================================================================================================//
//=================================================waitLong====================================================//
//============================================================================================================================//
/**
 * @brief Ожидание с длинной задержкой.
 *
 * Эта функция устанавливает длинную задержку с использованием таймера
 * и переводит микроконтроллер в режим сна. Микроконтроллер просыпается
 * по прерыванием или по истечению таймера
 */
void waitLong(void) {
#ifdef Debug
    printf("waitLong.\n"); // Отладочный вывод
#endif
    HAL_TIM_Base_Start_IT(&htim1); // Запуск таймера для задержки
    HAL_SuspendTick(); // Остановка системного таймера
    HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI); // Перевод в режим сна
    HAL_ResumeTick(); // Возобновление системного таймера после выхода из сна
    HAL_TIM_Base_Stop_IT(&htim1); // Остановка таймера
    HAL_Delay(1); // Задержка для стабилизации
}
//============================================================================================================================//
//================================================readInterrupt===============================================================//
//============================================================================================================================//
//Функция читает прерывания и возвращает следующую стадию конечного автомата
uint32_t readInterrupt(void){

	HAL_Delay(1);
	return nextState;
}
//============================================================================================================================//
//================================================waitForever====================================================//
//============================================================================================================================//
/**
 * @brief Ожидание вечное.
 *
 * Эта функция устанавливает микроконтроллер в вечный режим ожидания.
 * В это состояние входит выключение дисплея, остановка системного таймера
 * и перевод в режим Stop Mode для минимизации энергопотребления.
 * При прерываниях микроконтроллер возвращается из сна.
 */
void waitForever(void) {
#ifdef Debug
    printf("waitForever.\n"); // Отладочный вывод
#endif
    SSD1306_Clear(); // Очистка дисплея
    //ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xAE); // Выключение дисплея (отладка, разкоментрировать)
    HAL_GPIO_WritePin(PIN_POWER_DISPLAY_GPIO_Port, PIN_POWER_DISPLAY_Pin, RESET);// Выключение дисплея
    HAL_SuspendTick(); // Остановка системного таймера
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI); // Перевод в режим Stop Mode
    HAL_ResumeTick(); // Возобновление системного таймера
    HAL_GPIO_WritePin(PIN_POWER_DISPLAY_GPIO_Port, PIN_POWER_DISPLAY_Pin, SET); // Включение дисплея
    ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xAF); // Включение дисплея
    SSD1306_Init();
    HAL_Delay(1000); // Задержка для стабилизации (отладачная, поставить меньше)
}
//============================================================================================================================//
//======================================================waitLongTx====================================================//
//============================================================================================================================//
/**
 * @brief Ожидание в режиме передачи.
 *
 * Эта функция реализует ожидание в режиме передачи, учитывая время передачи и ожидания.
 * В зависимости от времени ожидания управляется режим работы модуля E32 и системного таймера.
 * По завершении ожидания микроконтроллер переходит в нормальному режиму работы.
 */
void waitLongTx(void) {

#ifdef Debug
    printf("waitLongTx\n"); // Отладочный вывод
#endif

    if (countButtonDemolitionClock < 6) { // Проверка времени передачи и ожидания, первые 6 секунд, ме должен не спать
        HAL_Delay(1);
        HAL_Delay(1000);
        bufferDisplay.timeTxWake--; // Уменьшение времени ожидания
        countButtonDemolitionClock++;
    } else {
        e32PinSetup(normalMode); // Перевод модуля E32 в нормальный режим
        HAL_TIM_Base_Start_IT(&htim2); // Запуск таймера 2 (на одну секунду)
        HAL_SuspendTick(); // Остановка системного таймера
        HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI); // Вход в режим SLEEP
        HAL_ResumeTick(); // Возобновление системного таймера
#ifdef Debug
        printf("HAL_PWR_EnterSLEEPMode\n"); // Отладочный вывод
#endif
        if (flagPeriodic2 == SET) {
            HAL_TIM_Base_Stop_IT(&htim2); // Остановка таймера
            bufferDisplay.timeTxWake--;
            countButtonDemolitionClock++;
            flagPeriodic2 = RESET;
        }
    }
    if (bufferDisplay.timeTxWake < 1) {
        e32PinSetup(sleepMode); // Перевод модуля E32 в режим сна
        nextState = wait; // Установка следующего состояния
        outDispl = outDisplayUser; // Установка указателя на функцию вывода на дисплей пользователя
    }

}
//============================================================================================================================//
//=================================================handlerRxMessage===========================================================//
//============================================================================================================================//
//Проверяет сообщение, возвращает следующее состояние
uint32_t handlerRxMessage(void){
	return cheсkRxMessage(rxByteTest, rxByteLog, ADRES_NUMBER_BYTE_EXPLODER);

	HAL_Delay(1);
}
//============================================================================================================================//
//=================================================HAL_UART_RxCpltCallback====================================================//
//============================================================================================================================//
/**
 * @brief Обработчик события приема данных по UART.
 *
 * Эта функция вызывается при завершении приема данных по UART.
 * При вызове функции устанавливается флаг прерывания и устанавливается следующее состояние для конечного автомата.
 * Также запускается новый прием данных по DMA.
 *
 * @param huart Указатель на структуру UART_HandleTypeDef, представляющую конфигурацию UART.
 * @param Size Количество принятых байт.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

#ifdef Debug
    printf("HAL_UARTEx_RxEventCallback.\n"); // Отладочный вывод
#endif

    if (huart->Instance == USART1) { // Проверка, что событие произошло для UART1
        flagInterrupt = SET; // Установка флага прерывания
        nextState = rxMessage; // Установка следующего состояния для конечного автомата
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveBuffer, rxByteSize); // Запуск нового приема данных по DMA
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); // Отключение прерывания при половине буфера
    }
}

//=============================================================================================================================//
//=================================================HAL_GPIO_EXTI_Callback======================================================//
//=============================================================================================================================//
/**
 * @brief Обработчик прерывания по изменению состояния входов GPIO.
 *
 * Эта функция вызывается при прерывании от входов GPIO, отвечающих за кнопки.
 * В зависимости от нажатой кнопки устанавливает соответствующее состояние и флаги для последующего обработчика.
 * Каждый блок условия относится к конкретной кнопке (взрыватель, демонтаж, тест), и в зависимости от нее устанавливаются определенные параметры.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    #ifdef Debug
        printf("HAL_GPIO_EXTI_Callback.\n"); // Отладочный вывод
    #endif
    if (GPIO_Pin == PIN_BUTTON_10_Pin || GPIO_Pin == PIN_BUTTON_11_Pin ||
        GPIO_Pin == PIN_BUTTON_12_Pin || GPIO_Pin == PIN_BUTTON_13_Pin ||
        GPIO_Pin == PIN_BUTTON_14_Pin || GPIO_Pin == PIN_BUTTON_ARM_Pin) {
        nextState = buttonDetonator; // Устанавливаем следующее состояние в режим кнопки выбора взрывателя
        flagInterrupt = SET; // Устанавливаем флаг прерывания
    } else if (GPIO_Pin == PIN_BUTTON_DEMOLITION_Pin) {
        nextState = buttonDemolition; // Устанавливаем следующее состояние в режим кнопки Демонтаж
        flagInterrupt = SET; // Устанавливаем флаг прерывания
        flagButtonDemolition = SET; // Устанавливаем флаг кнопки Демонтаж
        flagButtonDemolitionUpdateDisplay = SET; // Устанавливаем флаг обновления дисплея для кнопки Демонтаж
        countButtonDemolitionClock = 0; // Сбрасываем счетчик времени кнопки Демонтаж
        txStatus = txDemolition; // Устанавливаем статус передачи в режим Демонтаж
    } else if (GPIO_Pin == PIN_BUTTON_TEST_Pin) {
        nextState = buttonTest; // Устанавливаем следующее состояние в режим кнопки Тест
        flagInterrupt = SET; // Устанавливаем флаг прерывания
        flagButtonTest = SET; // Устанавливаем флаг кнопки Тест
        txStatus = txTest; // Устанавливаем статус передачи в режим Тест
        flagButtonTestUpdateDisplay = SET; // Устанавливаем флаг обновления дисплея для кнопки Тест
        countButtonTestClock = 0; // Сбрасываем счетчик времени кнопки Тест
    }

}

//=============================================================================================================================//
//=================================================HAL_ADC_ConvCpltCallback======================================================//
//=============================================================================================================================//
/**
 * @brief Обработчик прерывания по завершению преобразования ADC.
 *
 * Эта функция вызывается по завершению преобразования ADC.
 * При условии, что прерывание было вызвано ADC1, останавливает DMA, присваивает переменной adcVoltage[0] значение,
 * преобразованное из данных ADC.
 *
 * @param hadc Указатель на структуру ADC_HandleTypeDef.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) { // Проверяем, вызвано ли прерывание от ADC1
        adcVoltage[0] = adcData[0] / 88; // Преобразуем данные ADC в напряжение и сохраняем в массиве adcVoltage
    }
}

//=============================================================================================================================//
//===========================================HAL_TIM_PeriodElapsedCallback=====================================================//
//=============================================================================================================================//
/**
 * @brief Обработчик прерывания по истечении периода таймера.
 *
 * Эта функция вызывается по истечении периода таймера.
 * В зависимости от таймера, устанавливает соответствующие флаги прерывания и переключает состояние конечного автомата.
 *
 * @param htim Указатель на структуру TIM_HandleTypeDef, представляющую таймер.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) { // Проверяем, вызвано ли прерывание от TIM1
        flagInterrupt = SET; // Устанавливаем флаг прерывания
        flagPeriodic1 = SET; // Устанавливаем флаг периодического прерывания 1
         nextState = waitForev; // Указываем следующее состояние конечного автомата
    }
    if (htim->Instance == TIM2) { // Проверяем, вызвано ли прерывание от TIM2
        flagInterrupt = SET; // Устанавливаем флаг прерывания
        flagPeriodic2 = SET; // Устанавливаем флаг периодического прерывания 2
        nextState = waitTx; // Указываем следующее состояние конечного автомата
        if (bufferDisplay.timeTxWake < 1) { // Если время ожидания для передачи истекло
            nextState = wait; // Переключаемся в состояние ожидания
        }
    }
}
//=============================================================================================================================//
//===========================================togleLed()=====================================================//
//=============================================================================================================================//
extern void togleLed() {
    HAL_GPIO_TogglePin(PIN_LED_GPIO_Port, PIN_LED_Pin); // Переключает состояние пина светодиода
}
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
