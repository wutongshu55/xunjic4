#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

/* 长按判定时长(ms): 按住超过这个时间算长按。
 * 目前只有 KEY4 用长按 —— 进入/退出调试页, 故意设长一点避免误触。
 * 觉得还是容易误进就把这个值继续加大(比如 1200)。 */
#define KEY_LONG_MS    800

/* KEY4 长按事件号(不能和 1~4 的短按键号冲突) */
#define KEY_EV_LONG4   0x14

extern uint8_t Key_Num;
extern uint8_t Key_Last;

/* 系统毫秒时基: 由 TIM1 中断(5ms)累加 */
extern volatile uint32_t Sys_Tick_ms;

void Key_Init(void);

/* 必须由 TIM1 的 5ms 中断周期调用(去抖/长按/紧急停车都在里面) */
void Key_Scan_5ms(void);

/* 主循环取事件: 0=无, 1~4=短按, KEY_EV_LONG4=KEY4长按 */
uint8_t Key_GetNum(void);

#endif
