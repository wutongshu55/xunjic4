#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"

/* 判定"压在线上"的整体场强阈值(0~100), 低于此值视为丢线 */
#define TRACK_ONLINE_TH   12

/* 硬件峰值上限: 十字/环岛处峰值应 3200~3600, 超过 3700 必须降增益 */
#define ADC_SAT_TH_VALUE  3700

extern volatile uint16_t ADC_TempDat[4];
extern volatile uint16_t ADC_Dat[20][4];
extern volatile uint16_t ADC_Value[4];
extern volatile int16_t ADC_L, ADC_ML, ADC_MR, ADC_R;
/* 浮点归一化值: 误差计算与状态机判据用这个, 避免 8 位量化误差 */
extern volatile float ADC_Lf, ADC_MLf, ADC_MRf, ADC_Rf;
extern volatile float Error, Error_Last, Error_Raw;
extern uint16_t ADC_MAX[4], ADC_MIN[4];
extern float A, B, C;

/* 总场强(0~100): 横竖按 80/20 加权, 只用于"整条线还在不在"的粗判 */
extern volatile uint8_t ADC_Strength;
extern volatile float ADC_StrengthF;
/* 横置/竖置分开的场强: 判据必须按类型各自比较(横竖不互比) */
extern volatile float ADC_H_StrengthF;   /* 横置 (L,R)   平均 */
extern volatile float ADC_V_StrengthF;   /* 竖置 (ML,MR) 平均 */
/* 是否压在线上 */
extern volatile uint8_t ADC_Online;
/* 自动标定开关 */
extern uint8_t ADC_AutoCalib;
extern uint8_t Calib_State;

/* 峰值保持(调电位器用) + 饱和标志 */
extern volatile uint16_t ADC_Peak[4];
extern volatile uint8_t  ADC_SatFlag;   /* 本轮饱和通道位图 */
extern volatile uint8_t  ADC_SatSeen;   /* 是否曾经饱和(粘滞) */

void ADC_MyInit(void);
void ADC_GetValue(void);
void ADC_Filter(void);
void Get_Error(void);
void ADC_Calibrate(void);
void ADC_Calib_Reset(void);
void ADC_Peak_Reset(void);

#endif