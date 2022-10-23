/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PIN_IN	GPIO_PIN_1
#define PORT_IN GPIOB
#define BAUDRATE 10   //bits per second
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint32_t delayTime = 1000/BAUDRATE;
uint8_t numSamples = 0;
uint8_t buffer[50];
bool buttonFlag = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
void LOT_Receive(uint8_t *buffer);
void ReceiveSample(uint8_t *buffer);
void ReceiveCheckPoint(uint8_t buffer[]);
uint32_t bitsToDec(uint8_t bits[],uint8_t length);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 *
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  LOT_Receive(buffer);
/*	  if (buttonFlag){
		  LOT_Receive(buffer);
		  buttonFlag = false;
	  }*/
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

void EXTI0_1_IRQHandler(void)
{
	uint32_t current = HAL_GetTick();
	uint32_t prev = HAL_GetTick();
	if ((current-prev) && GPIO_PIN_0 == 1){
		buttonFlag = true;
		prev = current;
	}
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); // Clear interrupt flags
}


void LOT_Receive(uint8_t *buffer){
	GPIO_PinState firstBit = GPIO_PIN_RESET;
	GPIO_PinState secondBit = GPIO_PIN_RESET;
	uint32_t currentTime =0;
	uint32_t startTime =0;
	startTime = HAL_GetTick();

	firstBit = HAL_GPIO_ReadPin(PORT_IN, PIN_IN);

	//wait for first bit
	//timeout of 50s
	while(!firstBit){
		firstBit = HAL_GPIO_ReadPin(PORT_IN , PIN_IN);
		currentTime = HAL_GetTick();
		if(currentTime-startTime > 50000){
			return;
		}
	}
	HAL_Delay(delayTime*1.5);
	//read second bit
	secondBit = HAL_GPIO_ReadPin(PORT_IN, PIN_IN);
	HAL_Delay(delayTime);

	//if message is checkpoint
	if (firstBit && secondBit){
		ReceiveCheckPoint(buffer);
	}
	//if message is sample
	else if(firstBit && !secondBit){
		ReceiveSample(buffer);
	}

}

void ReceiveCheckPoint(uint8_t *buffer){
	GPIO_PinState startBit = HAL_GPIO_ReadPin(PORT_IN, PIN_IN);
	uint8_t bitArray[8] = {0};

	while(!startBit){
		startBit = HAL_GPIO_ReadPin(PORT_IN, PIN_IN);
	}

	HAL_Delay(delayTime*1.1);
	for (uint8_t i = 0; i< 8;i++){
		bitArray[i] = HAL_GPIO_ReadPin(PORT_IN, PIN_IN);
		buffer[i] = HAL_GPIO_ReadPin(PORT_IN, PIN_IN);
		HAL_Delay(delayTime);
	}

	uint32_t val = bitsToDec(bitArray,8);

	if (val == numSamples){
		HAL_UART_Transmit(&huart2, "No missing samples \n", 20, 1000);
	}else
	{
		numSamples = val;
		HAL_UART_Transmit(&huart2, "Missing Samples \n", 20, 1000);
	}

}

void ReceiveSample(uint8_t *buffer){
	uint32_t currentTime =0;
	uint32_t startTime =0;
	uint8_t bitArray[32] = {0};
	GPIO_PinState startBit = HAL_GPIO_ReadPin(PORT_IN, PIN_IN);  //read start bit
	GPIO_PinState tempBit = GPIO_PIN_RESET;
	startTime = HAL_GetTick();
	uint8_t countPackets = 0;

	//Timeout 50s
	//loop until startBit is 1
	while(!startBit){
		//increment timer
		currentTime = HAL_GetTick();
		if (currentTime -startTime >50000){
			return;
		}
		startBit = HAL_GPIO_ReadPin(PORT_IN , PIN_IN);
	}
	//delay*1.5 so polling is at middle of wave
	HAL_Delay(delayTime*1);

	//read data packets
	while (startBit){
		for (uint8_t i = (countPackets*8);i<(countPackets*8+8);i++){
			tempBit =  HAL_GPIO_ReadPin(PORT_IN, PIN_IN);
			bitArray[i] = tempBit;
			buffer[i] = tempBit;
			HAL_Delay(delayTime);
		}
		//read start/stop bit
		startBit = HAL_GPIO_ReadPin(PORT_IN, PIN_IN);
		countPackets++;
		HAL_Delay(delayTime);
	}
	uint32_t val = bitsToDec(bitArray,32);
	numSamples++;
	//HAL_UART_Transmit(&huart2, bitArray,32, 1000);
}

uint32_t bitsToDec(uint8_t bits[],uint8_t length){
	uint32_t multiplier = 1;
	uint32_t value = 0;

	for(int i =0;i<length;i++){
		value += (bits[i] * multiplier);
		multiplier *= 2;
	}
	return value;
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
