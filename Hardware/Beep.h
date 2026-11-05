#ifndef __BEEP_H
#define __BEEP_H

#include "stm32f10x.h"

extern volatile uint16_t Beep_Timer;
/* 蜂鸣器驱动模式(菜单第 6 页 Beep 条目可切换, 换一种响就说明硬件类型不同):
 *   0 = 无源蜂鸣器, 软件 2.8kHz 方波(默认)
 *   1 = 有源蜂鸣器, 高电平驱动
 *   2 = 有源蜂鸣器, 低电平驱动(带三极管/模块常见) */
extern volatile uint8_t Beep_Mode;

void Beep_Init(void);
void Beep_On(uint16_t time_ms);
void Beep_Off(void);
void Beep_Tick(void);

#endif
