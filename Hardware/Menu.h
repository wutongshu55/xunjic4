#ifndef __MENU_H
#define __MENU_H

#include "stm32f10x.h"

extern uint8_t Menu_RouteMode;
extern uint8_t Menu_DiamondEn;
extern uint8_t Menu_RingEn;
extern uint8_t Menu_SpeedLevel;
extern uint8_t Menu_TargetLaps;
extern uint8_t Menu_DoneLaps;
extern uint8_t Menu_Running;
extern uint8_t Menu_CrossCount;
extern uint8_t Menu_Page;
extern uint8_t Menu_Item;

extern const int16_t Speed_Level_Table[3];

void Menu_KeyHandler(uint8_t key);
void Menu_Display(void);
void Menu_RunDisplay(void);

#endif
