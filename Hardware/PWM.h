#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"

#define PWM_DUTY_MAX 10000

extern uint32_t Period_Data;

void PWM_Init(uint32_t Freq);
void PWMA_SetDuty(int16_t Duty);
void PWMB_SetDuty(int16_t Duty);

#endif
