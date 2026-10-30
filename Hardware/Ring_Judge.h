#ifndef __RING_JUDGE_H
#define __RING_JUDGE_H

#include "stm32f10x.h"

extern uint8_t  HexDia_Flag;     /* 六角菱形环状态机 */
extern uint16_t HexDia_Timer;    /* 六角菱形环计时 */
extern uint8_t  Cross_Stage;
extern uint16_t Cross_Timer;
extern uint8_t  Diamond_Stage;
extern uint16_t Diamond_Timer;

/* 元素对控制的输出 */
extern uint8_t  Track_Straight;    /* 1 = 强制直行 */
extern int16_t  Track_Steer_Bias;  /* 转向偏置 */
extern float    Track_Speed_Ratio; /* 元素速度系数 */

/* 横置/竖置各自的巡航基线(按类型各自比较, 不互比) */
extern float H_Base;
extern float V_Base;

void Cross_Judge(void);
void Diamond_Judge(void);
void HexDia_Judge(void);
void Lap_Judge(void);
void Track_Judge(void);

#endif
