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

/* 赛道状态(供显示与分状态策略使用) */
#define TRK_STRAIGHT 0   /* 直道 */
#define TRK_CURVE    1   /* 普通弯 */
#define TRK_SHARP    2   /* 直角弯(含"十字后连续两个直角弯") */
#define TRK_S_CURVE  3   /* 两条距离很近的直道 / S 弯 */
#define TRK_CROSS    4   /* 十字路口 */
#define TRK_HEXDIA   5   /* 六角菱形环 */
#define TRK_ARC      6   /* 半圆弧 */
#define TRK_ZIGZAG   7   /* 锯齿道路 */

extern uint8_t  Track_State;       /* 当前赛道状态 */
extern uint16_t Track_State_Ms;    /* 当前状态持续时间(ms) */
extern float    Track_Kd_Gain;     /* 状态决定的 D 增益倍数 */

void Track_StateMachine_Reset(void);

/* 横置/竖置各自的巡航基线(按类型各自比较, 不互比) */
extern float H_Base;
extern float V_Base;

void Cross_Judge(void);
void Diamond_Judge(void);
void HexDia_Judge(void);
void Lap_Judge(void);
void Track_Judge(void);

#endif
