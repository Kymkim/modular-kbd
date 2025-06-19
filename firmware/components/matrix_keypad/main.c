#include "main.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint8_t keyPressed = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  currentMillis = HAL_GetTick();
  if (currentMillis - previousMillis > 10) // Debounce delay
  {
    /* Set PB6, PB7 as input (to scan) */
    GPIO_InitStructPrivate.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
    GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructPrivate);

    /* --------- Row 1 LOW (PA15), Row 2 HIGH (PB3) --------- */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

    if (GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
      keyPressed = '1';
    else if (GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
      keyPressed = '2';

    /* --------- Row 1 HIGH, Row 2 LOW --------- */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

    if (GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
      keyPressed = '3';
    else if (GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
      keyPressed = '4';

    /* Reset rows to HIGH */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

    /* Set columns back to EXTI */
    GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructPrivate.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructPrivate);

    previousMillis = currentMillis;
  }
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* Set all rows HIGH initially */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

  while (1)
  {
    if (keyPressed != 0)
    {
      char msg[16];
      snprintf(msg, sizeof(msg), "Key: %c\r\n", keyPressed);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
      keyPressed = 0;
    }
  }
}

/* GPIO, Clock, and UART Init below — don't modify unless needed */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure row outputs: PA15, PB3 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure column inputs with EXTI: PB6, PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Enable EXTI interrupts */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}



///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2025 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
//
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
//
//
///* Private variables ---------------------------------------------------------*/
//UART_HandleTypeDef huart2;
//GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
//uint32_t previousMillis = 0;
//uint32_t currentMillis = 0;
//uint8_t keyPressed = 0;
//
///* USER CODE BEGIN PV */
//
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_USART2_UART_Init(void);
///* USER CODE BEGIN PFP */
//
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
///* USER CODE END 0 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//  currentMillis = HAL_GetTick();
//  if (currentMillis - previousMillis > 10) {
//    /*Configure GPIO pins : PB6 PB7 PB8 PB9 to GPIO_INPUT*/
//    GPIO_InitStructPrivate.Pin = GPIO_PIN_6|GPIO_PIN_7;
//    GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
//    GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStructPrivate);
//
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
//
//    if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
//    {
//      keyPressed = '1';
//    }
//    else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
//    {
//      keyPressed = '2';
//    }
////    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
////    {
////      keyPressed = 48; //ASCII value of B 66
////    }
////    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
////    {
////      keyPressed = 42; //ASCII value of A 65
////    }
//
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
//    if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
//    {
//      keyPressed = '3';
//    }
//    else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
//    {
//      keyPressed = '4';
//    }
////    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
////    {
////      keyPressed = 56; //ASCII value of 6 54
////    }
////    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
////    {
////      keyPressed = 55; //ASCII value of 3 51
////    }
//
////    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
////    if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
////    {
////      keyPressed = 66; //ASCII value of 0 48
////    }
////    else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
////    {
////      keyPressed = 54; //ASCII value of 8 56
////    }
////    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
////    {
////      keyPressed = 53; //ASCII value of 5 53
////    }
////    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
////    {
////      keyPressed = 52; //ASCII value of 2 50
////    }
////
////    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
////    if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
////    {
////      keyPressed = 65; //ASCII value of * 42
////    }
////    else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
////    {
////      keyPressed = 51; //ASCII value of 7 55
////    }
////    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
////    {
////      keyPressed = 50; //ASCII value of 4 52
////    }
////    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
////    {
////      keyPressed = 49; //ASCII value of 1 49
////    }
//
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
//    /*Configure GPIO pins : PB6 PB7 PB8 PB9 back to EXTI*/
//    GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING;
//    GPIO_InitStructPrivate.Pull = GPIO_PULLDOWN;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStructPrivate);
//    previousMillis = currentMillis;
//  }
//}
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//
//  /* USER CODE BEGIN 1 */
//
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_USART2_UART_Init();
//  /* USER CODE BEGIN 2 */
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//  }
//  /* USER CODE END 3 */
//}
//
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//  currentMillis = HAL_GetTick();
//  if (currentMillis - previousMillis > 10) // Debouncing check
//  {
//    /* Configure rows as inputs */
//    GPIO_InitStructPrivate.Pin = GPIO_PIN_6|GPIO_PIN_7;
//    GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStructPrivate);
//
//    /* Scan columns one by one */
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
//
//    if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
//      keyPressed = '1';
//    else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
//      keyPressed = '2';
////    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
////      keyPressed = 48; // ASCII of '0'
////    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
////      keyPressed = 42; // ASCII of '*'
//
//    /* Repeat for other columns */
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
//
//    if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
//      keyPressed = '3';
//    else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
//      keyPressed = '4';
////    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
////      keyPressed = 56; // ASCII of '8'
////    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
////      keyPressed = 55; // ASCII of '7'
//
////    /* Continue scanning for remaining columns */
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
////    if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
////      keyPressed = 66; // ASCII of 'B'
////    else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
////      keyPressed = 54; // ASCII of '6'
////    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
////      keyPressed = 53; // ASCII of '5'
////    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
////      keyPressed = 52; // ASCII of '4'
////
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
////    if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
////      keyPressed = 65; // ASCII of 'A'
////    else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
////      keyPressed = 51; // ASCII of '3'
////    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
////      keyPressed = 50; // ASCII of '2'
////    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
////      keyPressed = 49; // ASCII of '1'
//
//    /* Reset columns to HIGH */
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
//
//    /* Set rows back to interrupt mode */
//    GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING;
//    GPIO_InitStructPrivate.Pull = GPIO_PULLDOWN;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStructPrivate);
//
//    previousMillis = currentMillis;
//  }
//}
//
//
/////**
////  * @brief System Clock Configuration
////  * @retval None
////  */
////void SystemClock_Config(void)
////{
////  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
////  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
////
////  /** Initializes the RCC Oscillators according to the specified parameters
////  * in the RCC_OscInitTypeDef structure.
////  */
////  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
////  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
////  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
////  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
////  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
////  {
////    Error_Handler();
////  }
////
////  /** Initializes the CPU, AHB and APB buses clocks
////  */
////  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
////                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
////  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
////  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
////  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
////  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
////
////  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
////  {
////    Error_Handler();
////  }
////}
//
///**
//  * @brief USART2 Initialization Function
//  * @param None
//  * @retval None
//  */
////static void MX_USART2_UART_Init(void)
////{
////
////  /* USER CODE BEGIN USART2_Init 0 */
////
////  /* USER CODE END USART2_Init 0 */
////
////  /* USER CODE BEGIN USART2_Init 1 */
////
////  /* USER CODE END USART2_Init 1 */
////  huart2.Instance = USART2;
////  huart2.Init.BaudRate = 115200;
////  huart2.Init.WordLength = UART_WORDLENGTH_8B;
////  huart2.Init.StopBits = UART_STOPBITS_1;
////  huart2.Init.Parity = UART_PARITY_NONE;
////  huart2.Init.Mode = UART_MODE_TX_RX;
////  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
////  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
////  if (HAL_UART_Init(&huart2) != HAL_OK)
////  {
////    Error_Handler();
////  }
////  /* USER CODE BEGIN USART2_Init 2 */
////
////  /* USER CODE END USART2_Init 2 */
////
////}
//
//
//
//
/////**
////  * @brief GPIO Initialization Function
////  * @param None
////  * @retval None
////  */
////static void MX_GPIO_Init(void)
////{
////  GPIO_InitTypeDef GPIO_InitStruct = {0};
////  /* USER CODE BEGIN MX_GPIO_Init_1 */
////
////  /* USER CODE END MX_GPIO_Init_1 */
////
////  /* GPIO Ports Clock Enable */
////  __HAL_RCC_GPIOA_CLK_ENABLE();
////  __HAL_RCC_GPIOB_CLK_ENABLE();
////
////  /*Configure GPIO pin Output Level */
////  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
////
////  /*Configure GPIO pin Output Level */
////  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
////
////  /*Configure GPIO pin : PA15 */
////  GPIO_InitStruct.Pin = GPIO_PIN_15;
////  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
////  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
////
////  /*Configure GPIO pins : PB3 PB4 PB5 */
////  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
////  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
////  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
////
////  /*Configure GPIO pins : PB6 PB7 PB8 PB9 */
////  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
////  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
////
////  /* USER CODE BEGIN MX_GPIO_Init_2 */
////
////  /* USER CODE END MX_GPIO_Init_2 */
////}
////
/////* USER CODE BEGIN 4 */
////
/////* USER CODE END 4 */
////
/////**
////  * @brief  This function is executed in case of error occurrence.
////  * @retval None
////  */
////void Error_Handler(void)
////{
////  /* USER CODE BEGIN Error_Handler_Debug */
////  /* User can add his own implementation to report the HAL error return state */
////  __disable_irq();
////  while (1)
////  {
////  }
////  /* USER CODE END Error_Handler_Debug */
////}
////
////#ifdef  USE_FULL_ASSERT
/////**
////  * @brief  Reports the name of the source file and the source line number
////  *         where the assert_param error has occurred.
////  * @param  file: pointer to the source file name
////  * @param  line: assert_param error line source number
////  * @retval None
////  */
////void assert_failed(uint8_t *file, uint32_t line)
////{
////  /* USER CODE BEGIN 6 */
////  /* User can add his own implementation to report the file name and line number,
////     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
////  /* USER CODE END 6 */
////}
////#endif /* USE_FULL_ASSERT */
