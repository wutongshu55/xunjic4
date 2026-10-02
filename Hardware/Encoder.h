#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

extern volatile int16_t Encoder_L_Count;
extern volatile int16_t Encoder_R_Count;
extern volatile int32_t Encoder_L_Total;
extern volatile int32_t Encoder_R_Total;
extern volatile int32_t Speed_L_RPM;
extern volatile int32_t Speed_R_RPM;
extern volatile int32_t Speed_L_mms;
extern volatile int32_t Speed_R_mms;
extern volatile int32_t Speed_Avg_mms;

void Encoder_Init(void);
void Encoder_Update(float dt);

#endif
