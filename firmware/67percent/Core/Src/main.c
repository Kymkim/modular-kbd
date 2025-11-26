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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include <stdbool.h>
#include "pwm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    uint8_t MODIFIER;       // Modifier keys (Ctrl, Shift, Alt, Win)
    uint8_t RESERVED;       // Always 0
    uint8_t KEYPRESS[12];   // Up to 12 keycodes
} __attribute__((packed)) HIDReport;


// Switch pin mapping structure
typedef struct {
  GPIO_TypeDef* GPIOx;    // Pointer to GPIO port (e.g., GPIOA, GPIOB)
  uint16_t PIN;           // Pin number on the GPIO port
} SwitchPins;


// UART message structure for sending/receiving key events
typedef struct {
  uint16_t DEPTH;         // Custom field: could represent queue depth, layer, or message size
  uint16_t TYPE;          // Message type identifier (defines what kind of message this is)
  uint8_t KEYPRESS[12];   // Keypress data (similar to HIDReport, but for UART transmission)
} __attribute__((packed)) UARTMessage;

#define PACKET_SIZE 12
#define QUEUE_CAPACITY 32

typedef struct {
    uint8_t data[QUEUE_CAPACITY][PACKET_SIZE];
    volatile uint8_t head; // accessed in main
    volatile uint8_t tail; // accessed in ISR
    volatile uint8_t count; // optional, only if needed
} PacketQueue;

// Initialize
void pq_init(PacketQueue *q){
    q->head = 0;
    q->tail = 0;
    q->count = 0;
}

// Called from ISR
bool pq_push(PacketQueue *q, const uint8_t packet[PACKET_SIZE]){
    uint8_t nextTail = (q->tail + 1) % QUEUE_CAPACITY;
    if(nextTail == q->head) return false; // queue full

    memcpy(q->data[q->tail], packet, PACKET_SIZE);
    q->tail = nextTail;
    return true;
}

// Called from main
bool pq_pop(PacketQueue *q, uint8_t out_packet[PACKET_SIZE]){
    if(q->head == q->tail) return false; // queue empty

    memcpy(out_packet, q->data[q->head], PACKET_SIZE);
    q->head = (q->head + 1) % QUEUE_CAPACITY;
    return true;
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ROW 6
#define COL 5
#define MAXQUEUE 256
#define MODE_INACTIVE 0
#define MODE_MAINBOARD 1
#define MODE_ACTIVE 2
#define MODE_DEBUG 3
#define UART_RX_BUFF_SIZE 64
#define QUEUE_SIZ 8
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Initialize HID report properly
HIDReport REPORT = {0, 0, {0}};
UARTMessage RX5Msg; //Buffer for messages on uart5
UARTMessage RX1Msg; //Buffer for messages on uart5
UARTMessage RX2Msg; //Buffer for messages on uart5
UARTMessage RX4Msg; //Buffer for messages on uart5


SwitchPins ROW_PINS[ROW] = {
	{GPIOB, GPIO_PIN_10},
	{GPIOB, GPIO_PIN_2},
	{GPIOB, GPIO_PIN_1},
	{GPIOB, GPIO_PIN_0},
    {GPIOC, GPIO_PIN_5},
    {GPIOC, GPIO_PIN_4},
};

SwitchPins COLUMN_PINS[COL] = {
	{GPIOA, GPIO_PIN_8},
	{GPIOC, GPIO_PIN_9},
	{GPIOC, GPIO_PIN_8},
    {GPIOC, GPIO_PIN_7},
	{GPIOC, GPIO_PIN_6}
};

// Initialize keycodes array
uint8_t KEYCODES[ROW][COL] = {
    {0x00,		KEY_F13, 		KEY_F14, 		KEY_F15,			KEY_F16},
	{KEY_F17,	NUM_LOCK,		KEYPAD_SLASH,	KEYPAD_ASTERISK, 	KEYPAD_MINUS},
	{KEY_F18, 	KEYPAD_7,		KEYPAD_8,		KEYPAD_9,			KEYPAD_PLUS},
	{KEY_F19,	KEYPAD_4,		KEYPAD_5,		KEYPAD_6,			0x00},
	{KEY_F20,	KEYPAD_1,		KEYPAD_2,		KEYPAD_3,			KEYPAD_ENTER},
	{KEY_F21,	KEYPAD_0,		0x00,			KEYPAD_DOT,			0x00}
};

uint16_t DEPTH = 0;
uint16_t PORT_DEPTH[] = {0xFF, 0xFF, 0xFF, 0xFF};
UART_HandleTypeDef* PARENT;
UART_HandleTypeDef* PORTS[] = {&huart5, &huart1, &huart2, &huart4};
uint8_t KEYSTATE_CHANGED_FLAG = 0;
uint8_t KEYSTATE[ROW][COL];
								//North	 East	 South	 West
UARTMessage reportBuff;

extern USBD_HandleTypeDef hUsbDeviceFS;
volatile uint8_t MODE = MODE_INACTIVE;

UARTMessage uartBuffer;
volatile int uartUpdateFlag = 0;
// Encoder state (TIM3 in encoder mode on PA6/PA7)
volatile int32_t LAST_ENCODER_COUNT = 0;

uint8_t UART_KEYSTATE[4][12];


PacketQueue huart1q;
PacketQueue huart2q;
PacketQueue huart4q;
PacketQueue huart5q;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void handleUARTMessages(uint8_t *data, UART_HandleTypeDef *huart);
void UART_DMA_SendReport(UART_HandleTypeDef *huart);
void addUSBReport(uint8_t usageID);
void handleUARTMessages(uint8_t *data, UART_HandleTypeDef *sender);
void matrixScan(void);
void encoderProcess(void);
void resetReport(void);
void sendMessage(void);
void findBestParent();
void mergeChild();
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
  MX_DMA_Init();MX_PWM_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_PWM_Init();
  /* USER CODE BEGIN 2 */

  //Enable UART RX DMA for all ports
  HAL_UART_Receive_DMA(&huart1, (uint8_t*)&RX1Msg, sizeof(UARTMessage));
  HAL_UART_Receive_DMA(&huart2, (uint8_t*)&RX2Msg, sizeof(UARTMessage));
  HAL_UART_Receive_DMA(&huart4, (uint8_t*)&RX4Msg, sizeof(UARTMessage));
  HAL_UART_Receive_DMA(&huart5, (uint8_t*)&RX5Msg, sizeof(UARTMessage));

  // Start TIM3 encoder (PA6/PA7) so we can read encoder delta
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  LAST_ENCODER_COUNT = __HAL_TIM_GET_COUNTER(&htim3);

  //Prealloc Kestate matrix
  memset(KEYSTATE, 0, sizeof(KEYSTATE));
  pq_init(&huart1q);
  pq_init(&huart2q);
  pq_init(&huart4q);
  pq_init(&huart5q);

  PWM_Start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 67);
	  switch (MODE){
	  case MODE_ACTIVE:
		  KEYSTATE_CHANGED_FLAG = 1;
		  resetReport();
		  matrixScan();
		  mergeChild();
		  encoderProcess();
		  if(KEYSTATE_CHANGED_FLAG == 1){
			  UARTMessage UARTREPORT;
			  UARTREPORT.DEPTH = DEPTH;
			  UARTREPORT.TYPE = 0xEE;
			  memcpy(UARTREPORT.KEYPRESS, REPORT.KEYPRESS, sizeof(UARTREPORT.KEYPRESS));
			  HAL_UART_Transmit_DMA(PARENT, (uint8_t*)&UARTREPORT, sizeof(UARTREPORT));
		  }
		  break;

	  case MODE_INACTIVE:
		  //If the module is connected through the USB then mode is mainboard
		  if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED){
			  MODE = MODE_MAINBOARD;
			  DEPTH = 0;
		  }else{
			  //TODO: Look for a parent module...



			  UARTMessage REQ;
			  REQ.DEPTH = 0;
			  REQ.TYPE = 0xFF;	//Message code for request is 0xFF
			  memset(REQ.KEYPRESS, 0, sizeof(REQ.KEYPRESS));

			  //Send query' for parent module
			  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&REQ, sizeof(REQ));
			  HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&REQ, sizeof(REQ));
			  HAL_UART_Transmit_DMA(&huart4, (uint8_t*)&REQ, sizeof(REQ));
			  HAL_UART_Transmit_DMA(&huart5, (uint8_t*)&REQ, sizeof(REQ));
			  HAL_Delay(500);
			  findBestParent(); //So true...
		  }
		  break;

	  case MODE_MAINBOARD:
		  resetReport();
		  matrixScan();//Something related to this making the key stick. Likely due to race conditions
		  mergeChild();
		  encoderProcess();
		  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&REPORT, sizeof(REPORT));
		  break;

	  default:
		  break;
	  }

	  HAL_Delay(20);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void mergeChild(){
	uint8_t packet[12];
	if (pq_pop(&huart1q, packet)) {
		memcpy(UART_KEYSTATE[1], packet, 12);
		KEYSTATE_CHANGED_FLAG = 1;
	}
	if (pq_pop(&huart2q, packet)) {
		memcpy(UART_KEYSTATE[2], packet, 12);
		KEYSTATE_CHANGED_FLAG = 1;
	}
	if (pq_pop(&huart4q, packet)) {
		memcpy(UART_KEYSTATE[3], packet, 12);
		KEYSTATE_CHANGED_FLAG = 1;
	}
	if (pq_pop(&huart5q, packet)) {
		memcpy(UART_KEYSTATE[0], packet, 12);
		KEYSTATE_CHANGED_FLAG = 1;
	}
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 12; j++){
			REPORT.KEYPRESS[j] |= UART_KEYSTATE[i][j];
		}
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator out put  voltage
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
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// UART Message Requests Goes Here
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        handleUARTMessages((uint8_t*)&RX1Msg, &huart1);
        HAL_UART_Receive_DMA(&huart1, (uint8_t*)&RX1Msg, sizeof(UARTMessage));
    }
    else if (huart->Instance == USART2) {
        handleUARTMessages((uint8_t*)&RX2Msg, &huart2);
        HAL_UART_Receive_DMA(&huart2, (uint8_t*)&RX2Msg, sizeof(UARTMessage));
    }
    else if (huart->Instance == UART4) {
        handleUARTMessages((uint8_t*)&RX4Msg, &huart4);
        HAL_UART_Receive_DMA(&huart4, (uint8_t*)&RX4Msg, sizeof(UARTMessage));
    }
    else if (huart->Instance == UART5) {
        handleUARTMessages((uint8_t*)&RX5Msg, &huart5);
        HAL_UART_Receive_DMA(&huart5, (uint8_t*)&RX5Msg, sizeof(UARTMessage));
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    // Restart DMA on error
    if (huart->Instance == USART1) {
        HAL_UART_Receive_DMA(&huart1, (uint8_t*)&RX1Msg, sizeof(UARTMessage));
    }
    else if (huart->Instance == USART2) {
        HAL_UART_Receive_DMA(&huart2, (uint8_t*)&RX2Msg, sizeof(UARTMessage));
    }
    else if (huart->Instance == UART4) {
        HAL_UART_Receive_DMA(&huart4, (uint8_t*)&RX4Msg, sizeof(UARTMessage));
    }
    else if (huart->Instance == UART5) {
        HAL_UART_Receive_DMA(&huart5, (uint8_t*)&RX5Msg, sizeof(UARTMessage));
    }
}



void findBestParent(){
  //Find least depth parent
  uint16_t least_val = 0xFF;
  UART_HandleTypeDef* least_port = NULL;
  for(uint8_t i = 0; i < 4; i++){
	  if(PORT_DEPTH[i]<least_val){
		  least_port = PORTS[i];
		  least_val = PORT_DEPTH[i];
	  }
  }

  //Assign if valid
  if(least_val < 0xFF){
	  PARENT = least_port;
	  DEPTH = least_val + 1;
	  MODE = MODE_ACTIVE;
	  HAL_Delay(500);
  }
}

// Called when UART RX interrupt completes
void handleUARTMessages(uint8_t *data, UART_HandleTypeDef *sender) {
    UARTMessage msg;
    UARTMessage reply;

    // Parse incoming message into struct
    memcpy(&msg, data, sizeof(UARTMessage));

    switch(msg.TYPE) {

        // Parent request reply
        case 0xAA:
            if(sender == &huart5) {
                PORT_DEPTH[0] = msg.DEPTH;
            } else if(sender == &huart1) {
                PORT_DEPTH[1] = msg.DEPTH;
            } else if(sender == &huart2) {
                PORT_DEPTH[2] = msg.DEPTH;
            } else if(sender == &huart4) {
                PORT_DEPTH[3] = msg.DEPTH;
            }
            break;

        // Requested to be a parent
        case 0xFF:
        	if(MODE!=MODE_INACTIVE){
				reply.TYPE = 0xAA;
				reply.DEPTH = DEPTH;   // use your local DEPTH
				memset(reply.KEYPRESS, 0, sizeof(reply.KEYPRESS));
				HAL_UART_Transmit_DMA(sender, (uint8_t*)&reply, sizeof(reply));
        	}
            break;

        case 0xEE:
        	//TODO: Append message to the thingy
//            if (MODE != MODE_INACTIVE) {
//                for (int i = 0; i < sizeof(REPORT.KEYPRESS); i++) {
//                    uartBuffer.KEYPRESS[i] |= msg.KEYPRESS[i];
//                }
//                uartUpdateFlag = 1;
//            }
        	if(sender == &huart5) {
        	    pq_push(&huart5q, msg.KEYPRESS);
        	} else if(sender == &huart1) {
        		pq_push(&huart1q, msg.KEYPRESS);
        	} else if(sender == &huart2) {
        		pq_push(&huart2q, msg.KEYPRESS);
        	} else if(sender == &huart4) {
        		pq_push(&huart4q, msg.KEYPRESS);
        	}

        	break;

        default:
        	break;

    }
}


void addUSBReport(uint8_t usageID){
  if(usageID < 0x04 || usageID > 0x73) return; //Usage ID is out of bounds
  uint16_t bit_index = usageID - 0x04; //Offset, UsageID starts with 0x04. Gives us the actual value of the bit
  uint8_t byte_index = bit_index/8;    //Calculates which byte in the REPORT array
  uint8_t bit_offset = bit_index%8;    //Calculates which bits in the REPORT[byte_index] should be set/unset
  REPORT.KEYPRESS[byte_index] |= (1 << bit_offset);
}

void matrixScan(void){

    for (uint8_t col = 0; col < COL; col++){
        HAL_GPIO_WritePin(COLUMN_PINS[col].GPIOx, COLUMN_PINS[col].PIN, GPIO_PIN_SET);
        HAL_Delay(1);
        for(uint8_t row = 0; row < ROW; row++){
            uint8_t new_key = HAL_GPIO_ReadPin(ROW_PINS[row].GPIOx, ROW_PINS[row].PIN);
            if(new_key != KEYSTATE[row][col]){
                KEYSTATE_CHANGED_FLAG = 1;
                KEYSTATE[row][col] = new_key;
            }
            if(new_key){
            	addUSBReport(KEYCODES[row][col]);
            }
        }
        HAL_GPIO_WritePin(COLUMN_PINS[col].GPIOx, COLUMN_PINS[col].PIN, GPIO_PIN_RESET);
    }

}


// Read TIM3 encoder counter, calculate delta and add corresponding keycodes
void encoderProcess(void){
  int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
  int32_t diff = cnt - LAST_ENCODER_COUNT;
  // TIM3 configured as 16-bit counter (period 65535). Fix wrap-around.
  if(diff > 32767) diff -= 65536;
  if(diff < -32768) diff += 65536;
  if(diff > 0){
    int steps = diff;
    if(steps > 10) steps = 10; // cap bursts
    for(int i = 0; i < steps; i++){
      // CW -> KEYCODES[0][0]
      addUSBReport(KEYCODES[3][3]);
    }
  }else if(diff < 0){
    int steps = -diff;
    if(steps > 10) steps = 10;
    for(int i = 0; i < steps; i++){
      // CCW -> KEYCODES[0][1]
      addUSBReport(KEYCODES[2][1]);
    }
  }
  LAST_ENCODER_COUNT = cnt;
}

void resetReport(void){
	memset(REPORT.KEYPRESS, 0, sizeof(REPORT.KEYPRESS));
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

