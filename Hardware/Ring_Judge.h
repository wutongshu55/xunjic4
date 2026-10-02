#ifndef __RING_JUDGE_H
#define __RING_JUDGE_H

#include "stm32f10x.h"

extern uint8_t  Ring_Flag;
extern uint16_t Ring_CountNum;
extern uint8_t  Cross_Stage;
extern uint16_t Cross_Timer;
extern uint8_t  Diamond_Stage;
extern uint16_t Diamond_Timer;

void Cross_Judge(void);
void Diamond_Judge(void);
void Ring_Judge(void);
void Lap_Judge(void);
void Track_Judge(void);

#endif
