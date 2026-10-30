#ifndef __MENU_H
#define __MENU_H

#include "stm32f10x.h"

/* 发车页在菜单中的页号(改页数时这里要同步) */
#define MENU_PAGE_START 2

/* 目标圈数: 菜单里不再可选, 固定值, 需要改圈数在这里改 */
#define MENU_TARGET_LAPS 2

extern uint8_t Menu_HexDiaEn;      /* 六角菱形环开关 */
extern uint8_t Menu_SpeedLevel;
extern uint8_t Menu_TargetLaps;
extern uint8_t Menu_DoneLaps;
extern uint8_t Menu_Running;
extern uint8_t Menu_CrossCount;
extern uint8_t Menu_Page;
extern uint8_t Menu_Inited;
extern uint8_t Menu_RunInited;

/* 电感标定模式 0=固定标定值 1=运行中自动标定 */
extern uint8_t Menu_CalibMode;
/* 急弯(锯齿线/直角弯)保持模式 0=关 1=开 */
extern uint8_t Menu_ZigEn;
/* 距离测试页显示模式: 0=归一化 1=原始ADC 2=峰值保持 */
extern uint8_t Menu_DistRaw;

extern const int16_t Speed_Level_Table[3];
extern const int16_t Speed_Level_MMS[3];   /* 三档目标速度 mm/s */

void Menu_KeyHandler(uint8_t key);
void Menu_Display(void);
void Menu_RunDisplay(void);

#endif
