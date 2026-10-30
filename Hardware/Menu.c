#include "Common_headfile.h"

/* ===================== 全局运行参数 ===================== */
/* 是否进入六角菱形环: 0=不进 1=进 */
uint8_t  Menu_HexDiaEn    = 0;
/* 车速档位: 0=慢 1=中 2=快 */
uint8_t  Menu_SpeedLevel  = 1;
/* 目标圈数(菜单里不再可调, 固定为下面的常量) */
uint8_t  Menu_TargetLaps  = MENU_TARGET_LAPS;
/* 已完成圈数 */
uint8_t  Menu_DoneLaps    = 0;
/* 是否已发车(0=待发车 1=运行中 2=完成停车) */
uint8_t  Menu_Running     = 0;
/* 十字路口计数 */
uint8_t  Menu_CrossCount  = 0;
/* 菜单页索引 */
uint8_t  Menu_Page        = 0;

/* 新增: 电感标定模式 0=用固定标定值 1=运行中自动标定 */
uint8_t  Menu_CalibMode   = 0;
/* 新增: 急弯(锯齿线/直角弯)保持模式 0=关 1=开 */
uint8_t  Menu_ZigEn       = 1;
/* 新增: 距离测试页显示原始值还是归一化值 */
uint8_t  Menu_DistRaw     = 0;

/* 三档车速对应的基础PWM占空比(0~10000) */
const int16_t Speed_Level_Table[3] = {3000, 5000, 7500};

/* 菜单页数: 0=车速 1=六角菱形环 2=发车 3=标定 4=距离 5=Kp 6=Kd 7=急弯 */
#define MENU_PAGES 8

/* 菜单是否需要重画标签 */
uint8_t Menu_Inited = 0;

/* 获取某页参数指针(仅适用于整型参数页) */
static uint8_t *Menu_GetParam(uint8_t page)
{
    switch (page)
    {
        case 0: return &Menu_SpeedLevel;
        case 1: return &Menu_HexDiaEn;
        case 3: return &Menu_CalibMode;
        case 7: return &Menu_ZigEn;
        default: return &Menu_Page;
    }
}

/* 每页参数上限 */
static uint8_t Menu_GetMax(uint8_t page)
{
    switch (page)
    {
        case 0: return 2;   /* 0慢 1中 2快 */
        case 1: return 1;   /* 0,1 */
        case 3: return 1;   /* 标定: 0关 1自动 */
        case 7: return 1;   /* 急弯: 0关 1开 */
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
        Menu_Inited = 0;
        return;
    }
    if (key == 2)
    {
        Menu_Page++;
        if (Menu_Page >= MENU_PAGES) Menu_Page = 0;
        Menu_Inited = 0;
        return;
    }

    /* ---- 以下是 KEY3(减) / KEY4(加) ---- */
    switch (Menu_Page)
    {
        case MENU_PAGE_START: /* 发车页: KEY4 = 发车 / 停车 */
            if (key == 4)
            {
                if (Menu_Running == 1)
                {
                    /* 停车(运行页按 KEY4 也走这里, 保证两处逻辑一致) */
                    Menu_Running = 0;
                    Beep_On(100);
                }
                else
                {
                    /* 上一次已经跑完(圈数达标)则视为新一次比赛, 清零计数,
                     * 否则重新发车会带着旧的圈数, 跑一圈就直接停车 */
                    if (Menu_DoneLaps >= Menu_TargetLaps)
                    {
                        Menu_DoneLaps   = 0;
                        Menu_CrossCount = 0;
                    }
                    Menu_Running = 1;   /* 发车 */
                    Beep_On(200);
                }
            }
            break;

        case 3: /* 标定页: KEY3=重新标定  KEY4=固定/自动切换 */
            if (key == 3)
            {
                ADC_Calib_Reset();
                Beep_On(60);
            }
            else
            {
                Menu_CalibMode = !Menu_CalibMode;
                ADC_AutoCalib = Menu_CalibMode;
            }
            break;

        case 4: /* 距离测试页: KEY3=清零峰值  KEY4=切换显示模式 */
            if (key == 3)
            {
                ADC_Peak_Reset();
                Beep_On(60);
            }
            else if (key == 4)
            {
                Menu_DistRaw++;
                if (Menu_DistRaw > 2) Menu_DistRaw = 0;
                Menu_Inited = 0;        /* 标签随模式变化, 强制重画 */
            }
            break;

        case 5: /* Kp 在线调参 */
            if (key == 3) { if (kp > 1.0f)  kp -= 0.5f; }
            else           { if (kp < 60.0f) kp += 0.5f; }
            break;

        case 6: /* Kd 在线调参 */
            if (key == 3) { if (kd > 0.0f)  kd -= 0.5f; }
            else           { if (kd < 30.0f) kd += 0.5f; }
            break;

        default: /* 通用整型参数增减 */
            p  = Menu_GetParam(Menu_Page);
            mx = Menu_GetMax(Menu_Page);
            if (key == 3)
            {
                if (*p > 0) (*p)--;
            }
            else
            {
                if (*p < mx) (*p)++;
            }
            /* 标定开关与自动标定模块联动 */
            if (Menu_Page == 3)
            {
                ADC_AutoCalib = Menu_CalibMode;
            }
            break;
    }
}

/* 显示菜单页 - 首次画标签, 之后只更新数字 */
void Menu_Display(void)
{
    if (!Menu_Inited)
    {
        OLED_Clear();
        OLED_ShowString(1, 1, "== Menu ===");
        switch (Menu_Page)
        {
            case 0: /* 车速 */
                OLED_ShowString(2, 1, "Speed");
                OLED_ShowString(3, 1, "0:Sl 1:Mi 2:Fa");
                OLED_ShowString(4, 1, "Spd:  PWM    ");
                break;
            case 1: /* 六角菱形环 */
                OLED_ShowString(2, 1, "HexDia");
                OLED_ShowString(3, 1, "0:Skip 1:Enter");
                OLED_ShowString(4, 1, "Hex:         ");
                break;
            case MENU_PAGE_START: /* 发车 */
                OLED_ShowString(2, 1, "Start");
                OLED_ShowString(3, 1, "K4:Go/Stop");
                OLED_ShowString(4, 1, "Run:          ");
                break;
            case 3: /* 标定 */
                OLED_ShowString(2, 1, "Calib");
                OLED_ShowString(3, 1, "0:fix 1:auto");
                OLED_ShowString(4, 1, "MIN    MAX   ");
                break;
            case 4: /* 距离测试: 三种模式循环(KEY4 切换, KEY3 清零峰值) */
                if (Menu_DistRaw == 0)
                {
                    /* 归一化实时值: 横/竖各自 0~100 */
                    OLED_ShowString(1, 1, "== Dist N ==");
                    OLED_ShowString(2, 1, "L    ML ");
                    OLED_ShowString(3, 1, "MR   R  ");
                    OLED_ShowString(4, 1, "S    H    V");
                }
                else if (Menu_DistRaw == 1)
                {
                    /* 原始 ADC 实时值: 按 横(H)/竖(V) 分组 */
                    OLED_ShowString(1, 1, "= Dist Raw =");
                    OLED_ShowString(2, 1, "H");
                    OLED_ShowString(3, 1, "V");
                    OLED_ShowString(4, 1, "SAT  K3:CLR");
                }
                else
                {
                    /* 峰值保持: 推到十字/环岛看峰值是否落在 3200~3600 */
                    OLED_ShowString(1, 1, "= Dist Peak=");
                    OLED_ShowString(2, 1, "H");
                    OLED_ShowString(3, 1, "V");
                    OLED_ShowString(4, 1, "SAT  K3:CLR");
                }
                break;
            case 5: /* Kp */
                OLED_ShowString(2, 1, "Kp x10");
                OLED_ShowString(3, 1, "K3:-  K4:+   ");
                OLED_ShowString(4, 1, "Kp:          ");
                break;
            case 6: /* Kd */
                OLED_ShowString(2, 1, "Kd x10");
                OLED_ShowString(3, 1, "K3:-  K4:+   ");
                OLED_ShowString(4, 1, "Kd:          ");
                break;
            case 7: /* 急弯保持 */
                OLED_ShowString(2, 1, "Sharp");
                OLED_ShowString(3, 1, "0:off 1:on  ");
                OLED_ShowString(4, 1, "Zig:         ");
                break;
        }
        Menu_Inited = 1;
    }

    /* 只更新数值 */
    switch (Menu_Page)
    {
        case 0: /* 车速 */
            OLED_ShowNum(4, 6, Menu_SpeedLevel, 1);
            OLED_ShowNum(4, 12, Speed_Level_Table[Menu_SpeedLevel], 4);
            break;
        case 1: /* 六角菱形环 */
            OLED_ShowNum(4, 6, Menu_HexDiaEn, 1);
            break;
        case MENU_PAGE_START: /* 发车 */
            if (Menu_Running == 0)      OLED_ShowString(4, 6, "IDLE ");
            else if (Menu_Running == 1) OLED_ShowString(4, 6, "RUN  ");
            else                        OLED_ShowString(4, 6, "DONE ");
            break;
        case 3: /* 标定: 显示左电感(通道3)的标定范围 */
            OLED_ShowNum(4, 5, ADC_MIN[3], 4);
            OLED_ShowNum(4, 12, ADC_MAX[3], 4);
            break;
        case 4: /* 距离测试 */
            if (Menu_DistRaw == 0)
            {
                /* 归一化实时值(横竖各自 0~100) */
                OLED_ShowNum(2, 2, ADC_L, 3);      /* 左横置 PA7 */
                OLED_ShowNum(2, 7, ADC_ML, 3);     /* 中左竖置 PA6 */
                OLED_ShowNum(3, 2, ADC_MR, 3);     /* 中右竖置 PA5 */
                OLED_ShowNum(3, 7, ADC_R, 3);      /* 右横置 PA4 */
                OLED_ShowNum(4, 2, ADC_Strength, 2);
                OLED_ShowNum(4, 7, (uint16_t)ADC_H_StrengthF, 2);
                OLED_ShowNum(4, 12, (uint16_t)ADC_V_StrengthF, 2);
            }
            else
            {
                /* 原始值 / 峰值: 按 横(H: L,R) 与 竖(V: ML,MR) 分组 */
                uint16_t h1, h2, v1, v2;

                if (Menu_DistRaw == 1)
                {
                    h1 = ADC_Value[3];  h2 = ADC_Value[0];   /* L, R   */
                    v1 = ADC_Value[2];  v2 = ADC_Value[1];   /* ML, MR */
                }
                else
                {
                    h1 = ADC_Peak[3];   h2 = ADC_Peak[0];
                    v1 = ADC_Peak[2];   v2 = ADC_Peak[1];
                }
                OLED_ShowNum(2, 2, h1, 4);
                OLED_ShowNum(2, 7, h2, 4);
                OLED_ShowNum(3, 2, v1, 4);
                OLED_ShowNum(3, 7, v2, 4);
                OLED_ShowNum(4, 5, ADC_SatFlag, 2);      /* 饱和通道位图 */
            }
            break;
        case 5: /* Kp */
            OLED_ShowNum(4, 5, (uint32_t)(kp * 10.0f + 0.5f), 3);
            break;
        case 6: /* Kd */
            OLED_ShowNum(4, 5, (uint32_t)(kd * 10.0f + 0.5f), 3);
            break;
        case 7: /* 急弯 */
            OLED_ShowNum(4, 6, Menu_ZigEn, 1);
            break;
    }
}

/* 运行显示页是否需要重画标签 */
uint8_t Menu_RunInited = 0;

/* 运行时显示页(发车后) - 不清屏, 只更新数据位置 */
void Menu_RunDisplay(void)
{
    if (!Menu_RunInited)
    {
        OLED_Clear();
        OLED_ShowString(1, 1,  "L   ML   MR  ");
        OLED_ShowString(2, 1,  "R    E    ");
        OLED_ShowString(3, 1,  "V    S    L/");
        OLED_ShowString(4, 1,  "Crs  Hd   ");
        Menu_RunInited = 1;
    }
    /* 第1行: 四路电感归一化值 */
    OLED_ShowNum(1, 2, ADC_L, 3);
    OLED_ShowNum(1, 8, ADC_ML, 3);
    OLED_ShowNum(1, 14, ADC_MR, 2);
    /* 第2行: 右电感 + 循迹误差 */
    OLED_ShowNum(2, 2, ADC_R, 3);
    OLED_ShowSignedNum(2, 9, (int32_t)Error, 4);
    /* 第3行: 速度 + 场强 + 已完成圈数/目标圈数 */
    OLED_ShowNum(3, 2, Speed_Avg_mms, 4);
    OLED_ShowNum(3, 7, ADC_Strength, 3);
    OLED_ShowNum(3, 13, Menu_DoneLaps, 1);
    OLED_ShowNum(3, 14, Menu_TargetLaps, 1);
    /* 第4行: 十字计数 + 六角菱形环状态 + 实际基础PWM */
    OLED_ShowNum(4, 4, Menu_CrossCount, 2);
    OLED_ShowNum(4, 8, HexDia_Flag, 1);
    OLED_ShowNum(4, 11, Speed_Basic, 4);
}
