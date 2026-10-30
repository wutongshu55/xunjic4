#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"

extern volatile float kp, kd;
extern volatile int16_t Speed_Delta;
extern volatile float D_Filt;

/* 丢线保护状态 */
extern uint8_t  Lost_State;
extern uint16_t Lost_Timer;
extern int8_t   Last_Dir;

/* 急弯(锯齿线/直角弯)保持状态 */
extern uint8_t  Sharp_State;
extern uint16_t Sharp_Timer;
extern uint16_t Sharp_ExitCnt;
extern int8_t   Sharp_Dir;

void PID_Calculate(void);
void PID_LostLine_Check(void);
void PID_Sharp_Check(void);
int16_t PID_Apply(int16_t delta);
int16_t PID_Speed_Schedule(int16_t basic);

#endif
