#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"

extern volatile float kp, kd;
extern volatile int16_t Speed_Delta;

void PID_Calculate(void);

#endif
