/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

void SystemClock_Config(void);
int8_t NormalizeEncoder(int16_t raw_delta, uint8_t type);

//Matrix
uint16_t COL_ARR[] = {COL_1_Pin,COL_2_Pin,COL_3_Pin};
uint16_t ROW_ARR[] = {ROW_1_Pin, ROW_2_Pin, ROW_3_Pin};
uint16_t ETC_ARR[] = {START_Pin, SEL_Pin, VEFX_Pin, EFFECT_Pin};

//Encoders
int16_t scratch_val_prev = 0;
int16_t lazerL_val_prev = 0;
int16_t lazerR_val_prev = 0;
int16_t scratch_val;
int16_t lazerL_val;
int16_t lazerR_val;
int16_t deltaSC;
int16_t deltaL;
int16_t deltaR;

//Make this adjustable in the future
int8_t SC_DEDZONE = 2;
int8_t LZ_DEDZONE = 2;
int8_t SC_MAXDELTA = 15;
int8_t LZ_MAXDELTA = 15;

typedef struct{
  uint16_t buttons;
  int8_t scratch;
  int8_t left_laser;
  int8_t right_laser;
} controllerReport;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

  controllerReport report;

  while (1)
  {

    //POLL FOR KEYPADS
    for(int x = 0; x < 3; x++){
      HAL_GPIO_WritePin(GPIOA, COL_ARR[x], GPIO_PIN_SET);
      for(int y = 0; y < 3; y++){
        if(HAL_GPIO_ReadPin(GPIOC, ROW_ARR[y])){
          //Add Report HID
          uint8_t buttonIndex = y + x * 3;
          report.buttons |= (0x1 << buttonIndex);
        }
      }
      HAL_GPIO_WritePin(GPIOA, COL_ARR[x], GPIO_PIN_RESET);
    }

    //Read Encoders
    scratch_val_prev = scratch_val;
    lazerR_val_prev = lazerR_val;
    lazerL_val_prev = lazerL_val;
    scratch_val = __HAL_TIM_GET_COUNTER(&htim2);
    lazerL_val = __HAL_TIM_GET_COUNTER(&htim3);
    lazerR_val = __HAL_TIM_GET_COUNTER(&htim4);
    //Get Delta
    deltaSC = scratch_val - scratch_val_prev;
    deltaL = lazerL_val - lazerL_val_prev;
    deltaR = lazerR_val - lazerR_val_prev;
    //Add Report HID
    report.scratch = NormalizeEncoder(deltaSC,0);
    report.left_laser = NormalizeEncoder(deltaL,1);
    report.right_laser = NormalizeEncoder(deltaR,1);

    //Read START SEL VEFX EFFECT
    for(int i = 0; i < 4; i++){
      if(HAL_GPIO_ReadPin(GPIOB,ETC_ARR[i])){
        report.buttons |= (0x1 << (i + 9));
      }
    }
    HAL_Delay(10);
  }
}

/**
 * @brief Normalizes the Raw Encoder Data
 * @param raw_delta the raw delta number
 * @param type the type of normalization to apply. 0x0 for scratch, 0x1 for lazers
 * @retval Normalized encoder value from -127 to 127
 */
int8_t NormalizeEncoder(int16_t raw_delta, uint8_t type){

  //Set for normalization type
  uint8_t max_delta = SC_MAXDELTA;
  uint8_t dedzone = SC_DEDZONE;
  if(type){
    max_delta = LZ_MAXDELTA;
    dedzone = LZ_DEDZONE;
  }

  //Apply Deadzone
  if(raw_delta >= -dedzone && raw_delta <= dedzone){
    return 0;
  }

  //Normalize to -1.0 to 1.0
  float normalized = (float)raw_delta / max_delta;
  
  //Clamp
  if (normalized > 1.0f) normalized = 1.0f;
  if (normalized < -1.0f) normalized = -1.0f; 

  //Scale up to -127 to 127
  return(int8_t)(normalized*127.0f);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/* USER CODE BEGIN 4 */

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
