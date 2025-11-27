#ifndef __PWM_H__
#define __PWM_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize PWM on TIM2 Channel 1 (PA5)
void MX_PWM_Init(void);
// Start/stop PWM output
void PWM_Start(void);
void PWM_Stop(void);
// Set duty as raw timer compare value (0..Period)
void PWM_SetDuty(uint32_t duty);
// Helper: set duty as percent (0..100)
void PWM_SetPercent(uint8_t percent);

#ifdef __cplusplus
}
#endif

#endif // __PWM_H__
