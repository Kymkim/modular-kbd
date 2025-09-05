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
#include "usb_device.h"
#include "mainboard_usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
  uint8_t MODIFIER;
  uint8_t RESERVED;
  uint8_t KEYPRESS[14];
} HIDReportNKRO;

typedef struct{
  uint8_t MODIFIER;
  uint8_t RESERVED;
  uint8_t KEYPRESS[6]; // for 6 Key Rollover, changed index to 6.
} HIDReport6KRO;

typedef struct{
  GPIO_TypeDef* PORT;
  uint16_t PIN;
} KbdPins;

extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROWS 2
#define COLS 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//Keymap
uint8_t matrix[ROWS][COLS] = {
  {KEY_M, KEY_K},
  {KEY_I, KEY_U}
};

//Pins For the column {PORT, PIN}
KbdPins row_pins[ROWS] = {
  {GPIOC, GPIO_PIN_6},
  {GPIOB, GPIO_PIN_15}
};
KbdPins col_pins[COLS] = {
  {GPIOB, GPIO_PIN_14},
  {GPIOB, GPIO_PIN_13}
};

//Report - This what get sent to the USB 
HIDReportNKRO REPORT = {0};

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void addHIDReport(uint8_t usageID, uint8_t isPressed);
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
  MX_CAN1_Init();
  MX_USB_DEVICE_Init();
  HAL_Delay(200);
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_Delay(50);

  while (1)
  {
    //TODO: FIX ME
    //Keycode Scan
    // Clear keypresses at the start of each scan for 6 key rollover
    /* USER CODE END WHILE */
    // for(int col = 0; col < COLS; col++){
    //   HAL_GPIO_WritePin(col_pins[col].PORT, col_pins[col].PIN, GPIO_PIN_SET);
    //   HAL_Delay(1);
    //   for(int row = 0; row < ROWS; row++){
    //     if(HAL_GPIO_ReadPin(row_pins[row].PORT, row_pins[row].PIN)){
    //       addHIDReport(matrix[row][col], 1);
    //     }
    //   }
    //   HAL_GPIO_WritePin(col_pins[col].PORT, col_pins[col].PIN, GPIO_PIN_RESET);
    // }
    addHIDReport(KEY_A, 1);
    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&REPORT, sizeof(REPORT));
    HAL_Delay(50);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
  addHIDReport(RxData[0], (RxData[1] & 0x01) ? 1 : 0);
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
//Converts UsageID to NKRO bit field and add it to the report
void addHIDReport(uint8_t usageID, uint8_t isPressed){
  if(usageID < 0x04 || usageID > 0x73) return; //Usage ID is out of bounds
  uint16_t bit_index = usageID - 0x04; //Offset, UsageID starts with 0x04. Gives us the actual value of the bit
  uint8_t byte_index = bit_index/8;    //Calculates which byte in the REPORT array 
  uint8_t bit_offset = bit_index%8;    //Calculates which bits in the REPORT[byte_index] should be set/unset

  if(isPressed){
    REPORT.KEYPRESS[byte_index] |= (1 << bit_offset);
  }else{
    REPORT.KEYPRESS[byte_index] &= ~(1 << bit_offset);
  }
}

// 6 KEY ROLLOVER CODE STRUCTURE
/*
if (usageID < 0x04 || usageID > 0x73) return;

    int i, empty = -1;
    if (isPressed) {
        for (i = 0; i < 6; i++) {
            if (REPORT.KEYPRESS[i] == usageID) return; // Already present
            if (REPORT.KEYPRESS[i] == 0 && empty == -1) empty = i;
        }
        if (empty != -1) {
            REPORT.KEYPRESS[empty] = usageID;
        }
        // If no empty slot, ignore (6KRO: only 6 keys allowed)
    } else {
        // Remove from the array when released
        for (i = 0; i < 6; i++) {
            if (REPORT.KEYPRESS[i] == usageID) {
                REPORT.KEYPRESS[i] = 0;
            }
        }
    }
*/

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
