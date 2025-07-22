#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "usb_device.h"
#include "mainboard_usb.h"
#include "gpio.h"

#define ROWS 2
#define COLS 2

typedef struct {
    uint8_t MODIFIER;
    uint8_t RESERVED;
    uint8_t KEYPRESS[12];
} HIDReportNKRO;

typedef struct {
    uint8_t MODIFIER;
    uint8_t RESERVED;
    uint8_t KEYPRESS[6];  // for 6 Key Rollover, changed index to 6.
} HIDReport6KRO;

typedef struct {
    GPIO_TypeDef* PORT;
    uint16_t PIN;
} KbdPins;

uint8_t matrix[ROWS][COLS] = {
    {KEY_M, KEY_K},
    {KEY_I, KEY_U}
};

KbdPins row_pins[ROWS] = {
    {GPIOC, GPIO_PIN_6},
    {GPIOB, GPIO_PIN_15}
};

KbdPins col_pins[COLS] = {
    {GPIOB, GPIO_PIN_14},
    {GPIOB, GPIO_PIN_13}
};

bool matrix_state[ROWS][COLS] = {false};
bool last_state[ROWS][COLS] = {false};

volatile bool isMaster; // Flag to indicate if the device is master or slave

HIDReportNKRO REPORT = {0, 0, {0}};
HIDReportNKRO MODULEREPORT = {0, 0, {0}}; // Report for the module

CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

extern USBD_HandleTypeDef hUsbDeviceFS;

void SystemClock_Config(void);
void addHIDReport(HIDReportNKRO* report, uint8_t usageID, uint8_t isPressed);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_USB_DEVICE_Init();

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_Delay(50); // Stop CAN to configure filters

    isMaster = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0); // Read the master/slave status from a GPIO pin

    TxHeader.DLC = 2;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = 0x100;

    while (1)
    {
        // Keycode Scan
        for (int col = 0; col < COLS; col++)
        {
            HAL_GPIO_WritePin(col_pins[col].PORT, col_pins[col].PIN, GPIO_PIN_SET);
            HAL_Delay(1);

            for (int row = 0; row < ROWS; row++)
            {
                bool pressed = HAL_GPIO_ReadPin(row_pins[row].PORT, row_pins[row].PIN);

                if (pressed != last_state[row][col])
                {
                    last_state[row][col] = pressed;

                    if (isMaster)
                    {
                        addHIDReport(&REPORT, matrix[row][col], pressed);
                    }
                    else
                    {
                        TxData[0] = matrix[row][col];
                        TxData[1] = pressed ? 0x01 : 0x00;
                        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
                        memset(TxData, 0, sizeof(TxData));
                    }
                }
            }
            HAL_GPIO_WritePin(col_pins[col].PORT, col_pins[col].PIN, GPIO_PIN_RESET);
        }

        // USB report is sent when in master mode;
        if (isMaster)
        {
            USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&REPORT, sizeof(REPORT));
            HAL_Delay(1);
            USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&MODULEREPORT, sizeof(MODULEREPORT));
        }
        HAL_Delay(20);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // Only process messages if this is the master device
    if (isMaster)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
        addHIDReport(&MODULEREPORT, RxData[0], (RxData[1] & 0x01) ? 1 : 0);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

void addHIDReport(HIDReportNKRO* report, uint8_t usageID, uint8_t isPressed)
{
    if (usageID < 0x04 || usageID > 0x73) return; // Usage ID is out of bounds

    uint16_t bit_index = usageID - 0x04;   // Offset, UsageID starts at 0x04
    uint8_t byte_index = bit_index / 8;    // Byte in KEYPRESS array
    uint8_t bit_offset = bit_index % 8;    // Bit position in byte

    if (isPressed)
    {
        report->KEYPRESS[byte_index] |= (1 << bit_offset);
    }
    else
    {
        report->KEYPRESS[byte_index] &= ~(1 << bit_offset);
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

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
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
