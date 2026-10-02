#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

extern uint8_t Key_Num;
extern uint8_t Key_Last;

void Key_Init(void);
uint8_t Key_GetNum(void);

#endif
