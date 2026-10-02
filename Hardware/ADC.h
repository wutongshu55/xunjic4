#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"

extern volatile uint16_t ADC_TempDat[4];
extern volatile uint16_t ADC_Dat[20][4];
extern volatile uint16_t ADC_Value[4];
extern volatile int16_t ADC_L, ADC_ML, ADC_MR, ADC_R;
extern volatile float Error, Error_Last;
extern uint16_t ADC_MAX[4], ADC_MIN[4];
extern float A, B, C;
extern uint8_t Calib_State;

void ADC_MyInit(void);
void ADC_GetValue(void);
void ADC_Filter(void);
void Get_Error(void);
void ADC_Calibrate(void);

#endif
