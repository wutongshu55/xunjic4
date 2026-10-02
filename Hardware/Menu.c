#include "Common_headfile.h"

/* ===================== 全局运行参数 ===================== */
/* 路线选择: 0=主路线(直行优先) 1=进菱形环 2=进圆环 */
uint8_t  Menu_RouteMode   = 0;
/* 是否进菱形环: 0=不进 1=进 */
uint8_t  Menu_DiamondEn   = 0;
/* 是否进圆环: 0=不进 1=进 */
uint8_t  Menu_RingEn      = 0;
/* 车速档位: 0=慢 1=中 2=快 */
uint8_t  Menu_SpeedLevel  = 1;
/* 运行圈数(到达后停车) */
uint8_t  Menu_TargetLaps  = 2;
/* 已完成圈数 */
uint8_t  Menu_DoneLaps    = 0;
/* 是否已发车(0=待发车 1=运行中 2=完成停车) */
uint8_t  Menu_Running     = 0;
/* 十字路口计数 */
uint8_t  Menu_CrossCount  = 0;
/* 菜单页索引 */
uint8_t  Menu_Page        = 0;
/* 菜单项索引(每页内) */
uint8_t  Menu_Item        = 0;

/* 三档车速对应的基础PWM占空比(0~10000) */
const int16_t Speed_Level_Table[3] = {3000, 5000, 7500};

/* ===================== 菜单结构 ===================== */
/* 每页显示一个参数, KEY1=上一项, KEY2=下一项, KEY3=减, KEY4=加/确认 */

#define MENU_PAGES 7

/* 菜单标题(每页一个) */
static const char *Menu_Title[MENU_PAGES] =
{
    "1.Route ",   /* 路线模式 */
    "2.Diamnd",   /* 菱形环开关 */
    "3.Ring  ",   /* 圆环开关 */
    "4.Speed ",   /* 车速档位 */
    "5.Laps  ",   /* 目标圈数 */
    "6.Start ",   /* 发车 */
    "7.Reset ",   /* 复位计数 */
};

/* 获取某页参数指针 */
static uint8_t *Menu_GetParam(uint8_t page)
{
    switch (page)
    {
        case 0: return &Menu_RouteMode;
        case 1: return &Menu_DiamondEn;
        case 2: return &Menu_RingEn;
        case 3: return &Menu_SpeedLevel;
        case 4: return &Menu_TargetLaps;
        case 5: return &Menu_Running;
        case 6: return &Menu_DoneLaps;
        default: return &Menu_Page;
    }
}

/* 每页参数上限 */
static uint8_t Menu_GetMax(uint8_t page)
{
    switch (page)
    {
        case 0: return 2;   /* 0,1,2 */
        case 1: return 1;   /* 0,1 */
        case 2: return 1;   /* 0,1 */
        case 3: return 2;   /* 0慢 1中 2快 */
        case 4: return 9;   /* 1~9 */
        case 5: return 2;   /* 0待发 1运行 2完成 */
        case 6: return 0;   /* 复位项, 无数值 */
        default: return 0;
    }
}

/* 菜单按键处理: KEY1=上一页 KEY2=下一页 KEY3=数值- KEY4=数值+ */
void Menu_KeyHandler(uint8_t key)
{
    uint8_t *p;
    uint8_t  mx;

    if (key == 0) return;

    if (key == 1)
    {
        if (Menu_Page == 0) Menu_Page = MENU_PAGES - 1;
        else Menu_Page--;
    }
    else if (key == 2)
    {
        Menu_Page++;
        if (Menu_Page >= MENU_PAGES) Menu_Page = 0;
    }
    else if (key == 3)
    {
        p = Menu_GetParam(Menu_Page);
        if (*p > 0) (*p)--;
    }
    else if (key == 4)
    {
        if (Menu_Page == 6)
        {
            /* 复位计数 */
            Menu_DoneLaps   = 0;
            Menu_CrossCount = 0;
            Menu_Running    = 0;
            Beep_On(100);
        }
        else
        {
            p  = Menu_GetParam(Menu_Page);
            mx = Menu_GetMax(Menu_Page);
            if (*p < mx) (*p)++;
            /* 在发车页按下KEY4启动 */
            if (Menu_Page == 5 && Menu_Running == 1)
            {
                Beep_On(200);
            }
        }
    }
}

/* 显示菜单页 */
void Menu_Display(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "Menu Config");
    OLED_ShowString(2, 1, (char*)Menu_Title[Menu_Page]);

    switch (Menu_Page)
    {
        case 0: /* 路线 */
            OLED_ShowString(3, 1, "0:Main 1:Dia 2:Rng");
            OLED_ShowString(4, 1, "Sel:");
            OLED_ShowNum(4, 6, Menu_RouteMode, 1);
            break;
        case 1: /* 菱形环 */
            OLED_ShowString(3, 1, "0:Skip 1:Enter");
            OLED_ShowString(4, 1, "Dia:");
            OLED_ShowNum(4, 6, Menu_DiamondEn, 1);
            break;
        case 2: /* 圆环 */
            OLED_ShowString(3, 1, "0:Skip 1:Enter");
            OLED_ShowString(4, 1, "Rng:");
            OLED_ShowNum(4, 6, Menu_RingEn, 1);
            break;
        case 3: /* 车速档位 */
            OLED_ShowString(3, 1, "0:Slow 1:Mid 2:Fast");
            OLED_ShowString(4, 1, "Spd:");
            OLED_ShowNum(4, 6, Menu_SpeedLevel, 1);
            OLED_ShowString(4, 8, "PWM");
            OLED_ShowNum(4, 12, Speed_Level_Table[Menu_SpeedLevel], 4);
            break;
        case 4: /* 圈数 */
            OLED_ShowString(3, 1, "Target Laps");
            OLED_ShowString(4, 1, "Lap:");
            OLED_ShowNum(4, 6, Menu_TargetLaps, 1);
            break;
        case 5: /* 发车 */
            OLED_ShowString(3, 1, "K4 to Start/Stop");
            OLED_ShowString(4, 1, "Run:");
            if (Menu_Running == 0)      OLED_ShowString(4, 6, "IDLE ");
            else if (Menu_Running == 1) OLED_ShowString(4, 6, "RUN  ");
            else                        OLED_ShowString(4, 6, "DONE ");
            break;
        case 6: /* 复位 */
            OLED_ShowString(3, 1, "K4 to Reset");
            OLED_ShowString(4, 1, "Done:");
            OLED_ShowNum(4, 6, Menu_DoneLaps, 1);
            OLED_ShowString(4, 9, "Crs:");
            OLED_ShowNum(4, 13, Menu_CrossCount, 2);
            break;
    }
}

/* 运行时显示页(发车后) */
void Menu_RunDisplay(void)
{
    OLED_Clear();
    /* 第1行: 四路电感归一化值 */
    OLED_ShowString(1, 1, "L");
    OLED_ShowNum(1, 2, ADC_L, 3);
    OLED_ShowString(1, 6, "ML");
    OLED_ShowNum(1, 8, ADC_ML, 3);
    OLED_ShowString(1, 12, "MR");
    OLED_ShowNum(1, 14, ADC_MR, 2);
    /* 第2行: 右电感 + 误差 */
    OLED_ShowString(2, 1, "R");
    OLED_ShowNum(2, 2, ADC_R, 3);
    OLED_ShowString(2, 7, "E");
    OLED_ShowSignedNum(2, 9, (int32_t)Error, 4);
    /* 第3行: 速度 */
    OLED_ShowString(3, 1, "V");
    OLED_ShowNum(3, 2, Speed_Avg_mms, 4);
    OLED_ShowString(3, 8, "mm/s");
    OLED_ShowString(3, 13, "L");
    OLED_ShowNum(3, 14, Menu_DoneLaps, 1);
    OLED_ShowString(3, 16, "/");
    /* 第4行: 圈数/目标 + 十字 + 环状态 */
    OLED_ShowNum(4, 1, Menu_TargetLaps, 1);
    OLED_ShowString(4, 3, "Crs");
    OLED_ShowNum(4, 6, Menu_CrossCount, 2);
    OLED_ShowString(4, 9, "R");
    OLED_ShowNum(4, 10, Ring_Flag, 1);
    OLED_ShowString(4, 12, "Spd");
    OLED_ShowNum(4, 15, Speed_Basic, 4);
}
