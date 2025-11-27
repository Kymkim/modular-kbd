#include "pwm.h"
#include "tim.h"

void MX_PWM_Init(void){
    TIM_OC_InitTypeDef sConfigOC = {0};

    // Initialize TIM2 for PWM (safe to call even if TIM2 was previously initialized for OC)
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK){
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK){
        Error_Handler();
    }
}

void PWM_Start(void){
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void PWM_Stop(void){
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void PWM_SetDuty(uint32_t duty){
    // Clamp duty to timer period
    uint32_t period = htim2.Init.Period;
    if(duty > period) duty = period;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
}

void PWM_SetPercent(uint8_t percent){
    if(percent > 100) percent = 100;
    uint32_t period = htim2.Init.Period;
    uint32_t duty = (period * percent) / 100;
    PWM_SetDuty(duty);
}
