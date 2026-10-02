#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

extern volatile int16_t Speed_Basic;

void Motor_Init(void);
void Motor_Out(int16_t L_Speed, int16_t R_Speed);
void CarMotor_Control(void);

#endif
