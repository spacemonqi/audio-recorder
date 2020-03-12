/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "variables.h"
#include "math.h"
#include "sinewave.h"
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
DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile State state;
volatile int buttOne,buttTwo, buttThree, buttRec, buttStop, exti_start, exti, state_start, wave;
volatile int ticky, Ri, Rf;
uint16_t dac_buffer[1024];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
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


	state = Idle;
	state_start = off;
	exti = off;
	exti_start = off;
	buttOne = off;
	buttTwo	= off;
	buttThree = off;
	buttRec = off;
	buttStop = off;

	wave_init();

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
  MX_USART2_UART_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  uint8_t msg[10] = {127, 128, '2','1','7','8','5','1','5','5'};
  HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
  HAL_TIM_Base_Start(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /////////////////////////////////////////////////////////////////////
	  if (exti){

		  if (!exti_start) Ri = HAL_GetTick();
		  exti_start = on;
		  Rf = HAL_GetTick();

		  	if (Rf - Ri > 10){
		  		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)) buttOne = on;
		  		else buttOne = off;

			  	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)) buttTwo = on;
			  	else buttTwo = off;

		  		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)) buttThree = on;
		  		else buttThree = off;

		  		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)) buttRec = on;
		  		else buttRec = off;

		  		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)) buttStop = on;
		  		else buttStop = off;

		  		exti = off;
		  		state_start = off;
		  		exti_start = off;
		  	}
	  }
	  /////////////////////////////////////////////////////////////////////

	  if (!state_start && !(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))){
			  if (buttOne || buttTwo || buttThree || buttStop) state_start = on;
			  if (!buttRec && buttOne){
				  state = PlayOne;
				  wave = 1;
				  uint8_t msg[10] = {127, 128,'P','l','a','y','_','_','_','1'};
				  HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
		  		  wave_fillbuffer(dac_buffer, 1, 1024);
		  		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dac_buffer, 1024, DAC_ALIGN_12B_R);
			  }
			  else if (!buttRec && buttTwo){
				  state = PlayTwo;
				  wave = 2;
				  uint8_t msg[10] = {127, 128,'P','l','a','y','_','_','_','2'};
				  HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
		  		  wave_fillbuffer(dac_buffer, 2, 1024);
		  		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dac_buffer, 1024, DAC_ALIGN_12B_R);
			  }
			  else if (!buttRec && buttThree){
				  state = PlayThree;
				  wave = 3;
				  uint8_t msg[10] = {127, 128,'P','l','a','y','_','_','_','3'};
				  HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
		  		  wave_fillbuffer(dac_buffer, 3, 1024);
		  		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dac_buffer, 1024, DAC_ALIGN_12B_R);
			  }
			  else if (buttRec && buttOne){
				  state = RecOne;
				  uint8_t msg[10] = {127, 128,'R','e','c','o','r','d','_','1'};
				  HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
		  		  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
			  }
			  else if (buttRec && buttTwo){
				  state = RecTwo;
				  uint8_t msg[10] = {127, 128,'R','e','c','o','r','d','_','2'};
				  HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
		  		  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
			  }
			  else if (buttRec && buttThree){
				  state = RecThree;
				  uint8_t msg[10] = {127, 128,'R','e','c','o','r','d','_','3'};
				  HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
		  		  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
			  }
			  else if (buttStop){
				  state = Idle;
				  uint8_t msg[10] = {127, 128,'S','t','o','p','_','_','_','_'};
				  HAL_UART_Transmit(&huart2, msg, sizeof(msg), 1000);
		  		  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
			  }
	  }

	  ticky = HAL_GetTick();
	  ///////////////////////////////////////////////////////////////////////////////////
	  if (state == PlayOne || state == PlayTwo || state == PlayThree || state == Idle){
	  	  if (state == PlayOne){
	  		  if (ticky % 500 < 250) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, on);
	  		  else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, off);
	  	  }
	  	  else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, off);

	  	  if (state == PlayTwo){
	  		  if (ticky % 500 < 250) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, on);
	  		  else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, off);
	  	  }
	  	  else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, off);

	  	  if (state == PlayThree){
	  		  if (ticky % 500 < 250) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, on);
	  		  else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, off);
	  	  }
	  	  else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, off);
	  }
	  ///////////////////////////////////////////////////////////////////////////////////
  	  if (state == RecOne || state == RecTwo || state == RecThree){
  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, on);

  	  	  if (state == RecOne){
  	  		  if (ticky % 500 < 250) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, on);
  	  		  else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, off);
  	  	  }
  	  	  else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, off);

  	  	  if (state == RecTwo){
  	  		  if (ticky % 500 < 250) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, on);
  	  		  else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, off);
  	  	  }
  	  	  else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, off);

  	  	  if (state == RecThree){
  	  		  if (ticky % 500 < 250) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, on);
  	  		  else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, off);
  	  	  }
  	  	  else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, off);
  	  }
  	  else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, off);
	  ///////////////////////////////////////////////////////////////////////////////////
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1905;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.BaudRate = 500000;
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

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
	  HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t*)dac_buffer, 1024, DAC_ALIGN_12B_R);
	  wave_fillbuffer(dac_buffer+512, wave, 512);
}

/**
  * @brief  Conversion half DMA transfer callback in non blocking mode for Channel1
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
	  wave_fillbuffer(dac_buffer, wave, 512);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
