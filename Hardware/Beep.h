#ifndef __BEEP_H
#define __BEEP_H

#include "stm32f10x.h"

extern volatile uint16_t Beep_Timer;

void Beep_Init(void);
void Beep_On(uint16_t time_ms);
void Beep_Off(void);
void Beep_Tick(void);

#endif
