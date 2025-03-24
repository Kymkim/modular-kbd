#include "stm32f1xx_hal.h"

#define NUM_COLS 2
#define NUM_ROWS 2

//Stores the GPIO and the PIN location - would be useful for the matrix
typedef struct {
    GPIO_TypeDef* GPIO;
    uint16_t PIN;
} keyMatrix ;

keyMatrix matrix[NUM_COLS][NUM_ROWS] = {
    {{GPIOA, GPIO_PIN_0},{GPIOA, GPIO_PIN_1}},  //Column
    {{GPIOA, GPIO_PIN_2},{GPIOA, GPIO_PIN_3}}   //Row     
};

int main(void){

    HAL_Init(); //Initialize the HAL Abstraction Layer

    //Initialize a 2x2 pins
    for(int col = 0; col<NUM_COLS; col++){
        GPIO_InitTypeDef GPIO_InitCols = {0};
        GPIO_InitCols.Pin = matrix[0][col].PIN;
        GPIO_InitCols.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitCols.Pull = GPIO_NOPULL;
        GPIO_InitCols.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(matrix[0][col].GPIO, &GPIO_InitCols);
    }

    for(int row = 0; row<NUM_ROWS; row++){
        GPIO_InitTypeDef GPIO_InitRows = {0};
        GPIO_InitRows.Pin = matrix[1][row].PIN;
        GPIO_InitRows.Mode = GPIO_MODE_INPUT;
        GPIO_InitRows.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(matrix[0][row].GPIO, &GPIO_InitRows);
    }


    // TODO: Deboucing Code - I suggest not to use HAL_Delay() beacause it is blocking 
    // and might affect performance and introduce input delay. 
    for(int col=0; col<NUM_COLS; col++){
        //Set column to high
        HAL_GPIO_WritePin(matrix[0][col].GPIO, matrix[0][col].PIN, GPIO_PIN_SET);
        
        //Read each row pin
        for(int row=0; row<NUM_ROWS; row++){
            if(HAL_GPIO_ReadPin(matrix[1][row].GPIO, matrix[0][col].PIN)){
                //Register Key Press here
            }
        }

        //Set column to back to low
        HAL_GPIO_WritePin(matrix[0][col].GPIO, matrix[0][col].PIN, GPIO_PIN_RESET);
    }

}

/*
    Would be nice if someone can abstract all of keyboard checking and initialization
    into a separate include file so we can reuse it with other modules
*/