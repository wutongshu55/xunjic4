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

/* 三档车速: 现在给的是【目标速度 mm/s】, 由速度环闭环执行 */
const int16_t Speed_Level_MMS[3] = {300, 480, 700};

/* 旧的 PWM 开环表, 仅在需要对照时参考 */
const int16_t Speed_Level_Table[3] = {3000, 5000, 7500};

/* 菜单页数(精简为 7 页):
 * 0=车速 1=发车 2=标定 3=距离/编码器 4=Kp 5=Kd 6=模式开关(环/急弯/闭环)
 * 原来独立的 HexDia / Sharp / Loop 三页合并成第 7 页"Mode",
 * KEY3 选择条目, KEY4 切换该条目的值。 */
#define MENU_PAGES 7

/* 模式页当前选中的条目: 0=六角环 1=急弯 2=开环闭环 */
uint8_t Menu_Mode_Sel = 0;

/* 菜单是否需要重画标签 */
uint8_t Menu_Inited = 0;

/* 获取某页参数指针(仅适用于整型参数页) */
static uint8_t *Menu_GetParam(uint8_t page)
{
    switch (page)
    {
        case 0: return &Menu_SpeedLevel;
        case 2: return &Menu_CalibMode;
        default: return &Menu_Page;
    }
}

/* 每页参数上限 */
static uint8_t Menu_GetMax(uint8_t page)
{
    switch (page)
    {
        case 0: return 2;   /* 车速: 0慢 1中 2快 */
        case 2: return 1;   /* 标定: 0固定 1自动 */
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

    /* ---- 以下是 KEY3(减/选择) / KEY4(加/切换) ---- */
    switch (Menu_Page)
    {
        case MENU_PAGE_START: /* 发车页(第 2 页): KEY4 = 发车 / 停车 */
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

                    /* 发车前把闭环/状态机的状态清干净 */
                    Speed_Loop_Reset();          /* 速度环积分 */
                    Speed_Fault_Reset();         /* 故障标志重新检测 */
                    Track_StateMachine_Reset();  /* 赛道状态机 */
                    Menu_Running = 1;            /* 发车 */
                    Beep_On(200);
                }
            }
            break;

        case 2: /* 标定页: KEY3=重新标定  KEY4=固定/自动切换 */
            if (key == 3)
            {
                ADC_Calib_Reset();
                Beep_On(60);
            }
            else
            {
                Menu_CalibMode = !Menu_CalibMode;
                ADC_AutoCalib  = Menu_CalibMode;
                Beep_On(40);
            }
            break;

        case 3: /* 距离/编码器页: KEY3=清零  KEY4=切换显示模式 */
            if (key == 3)
            {
                if (Menu_DistRaw == 3)
                {
                    Encoder_L_Total = 0;    /* 模式4: 清零里程计数 */
                    Encoder_R_Total = 0;
                }
                else
                {
                    ADC_Peak_Reset();
                }
                Beep_On(60);
            }
            else
            {
                Menu_DistRaw++;
                if (Menu_DistRaw > 3) Menu_DistRaw = 0;
                Menu_Inited = 0;        /* 标签随模式变化, 强制重画 */
            }
            break;

        case 4: /* Kp 在线调参 */
            if (key == 3) { if (kp > 1.0f)  kp -= 0.5f; }
            else          { if (kp < 60.0f) kp += 0.5f; }
            break;

        case 5: /* Kd 在线调参 */
            if (key == 3) { if (kd > 0.0f)  kd -= 0.5f; }
            else          { if (kd < 30.0f) kd += 0.5f; }
            break;

        case 6: /* 模式页: KEY3=选择条目  KEY4=切换该条目的值 */
            if (key == 3)
            {
                Menu_Mode_Sel++;
                if (Menu_Mode_Sel > 2) Menu_Mode_Sel = 0;
                Beep_On(30);
            }
            else
            {
                if (Menu_Mode_Sel == 0)
                {
                    Menu_HexDiaEn = !Menu_HexDiaEn;
                }
                else if (Menu_Mode_Sel == 1)
                {
                    Menu_ZigEn = !Menu_ZigEn;
                }
                else
                {
                    Speed_Loop_En = !Speed_Loop_En;
                    Speed_Loop_Reset();      /* 切换模式时清积分, 避免冲一下 */
                }
                Beep_On(60);
            }
            break;

        default: /* 通用整型参数增减(车速页) */
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
                OLED_ShowString(4, 1, "Gear");
                OLED_ShowString(4, 8, "|");
                OLED_ShowString(4, 14, "mms");
                break;
            case MENU_PAGE_START: /* 发车 */
                OLED_ShowString(2, 1, "Start");
                OLED_ShowString(3, 1, "K4: GO/STOP ");
                OLED_ShowString(4, 1, "Run:          ");
                break;
            case 2: /* 标定 */
                OLED_ShowString(2, 1, "Calib");
                OLED_ShowString(3, 1, "0:fix 1:auto");
                OLED_ShowString(4, 1, "MIN    MAX   ");
                break;
            case 3: /* 距离/编码器: 四种模式循环(KEY4 切换, KEY3 清零) */
                if (Menu_DistRaw == 0)
                {
                    /* 归一化实时值: 横/竖各自 0~100 */
                    OLED_ShowString(1, 1, "== Dist N ==");
                    OLED_ShowString(2, 1, "L");
                    OLED_ShowString(2, 8, "ML");
                    OLED_ShowString(3, 1, "R");
                    OLED_ShowString(3, 8, "MR");
                    OLED_ShowString(4, 1, "S");
                    OLED_ShowString(4, 7, "H");
                    OLED_ShowString(4, 12, "V");
                }
                else if (Menu_DistRaw == 1)
                {
                    /* 原始 ADC 实时值: 按 横(H)/竖(V) 分组 */
                    OLED_ShowString(1, 1, "= Dist Raw =");
                    OLED_ShowString(2, 1, "H");
                    OLED_ShowString(3, 1, "V");
                    OLED_ShowString(4, 1, "SAT     K3:CLR");
                }
                else if (Menu_DistRaw == 2)
                {
                    /* 峰值保持: 推到十字/环岛看峰值是否落在 3200~3600 */
                    OLED_ShowString(1, 1, "= Dist Peak=");
                    OLED_ShowString(2, 1, "H");
                    OLED_ShowString(3, 1, "V");
                    OLED_ShowString(4, 1, "SAT     K3:CLR");
                }
                else
                {
                    /* 编码器自检: 手推车 1 米, 看 L/R 速度和总计数是否在动 */
                    OLED_ShowString(1, 1, "= Enc  K3CLR=");
                    OLED_ShowString(2, 1, "L");
                    OLED_ShowString(2, 8, "R");
                    OLED_ShowString(3, 1, "CL");
                    OLED_ShowString(4, 1, "CR");
                }
                break;
            case 4: /* Kp: 顺带显示实时 E / D, 一边拧一边看 */
                OLED_ShowString(2, 1, "Kp x10  K3-/K4+");
                OLED_ShowString(3, 1, "E");
                OLED_ShowString(3, 7, "D");
                OLED_ShowString(4, 1, "Kp:");
                OLED_ShowString(4, 9, "Dl");
                break;
            case 5: /* Kd: 同上 */
                OLED_ShowString(2, 1, "Kd x10  K3-/K4+");
                OLED_ShowString(3, 1, "E");
                OLED_ShowString(3, 7, "D");
                OLED_ShowString(4, 1, "Kd:");
                OLED_ShowString(4, 9, "Dl");
                break;
            case 6: /* 模式页: 三合一开关 */
                OLED_ShowString(1, 1, "= Mode 1/3 =");
                OLED_ShowString(2, 1, " HexDia     ");
                OLED_ShowString(3, 1, " Sharp      ");
                OLED_ShowString(4, 1, " Loop       ");
                break;
        }
        Menu_Inited = 1;
    }

    /* 只更新数值 */
    switch (Menu_Page)
    {
        case 0: /* 车速: 档位 + 目标 mm/s(3 位) */
            OLED_ShowNum(4, 6, Menu_SpeedLevel, 1);
            OLED_ShowNum(4, 10, Speed_Level_MMS[Menu_SpeedLevel], 3);
            break;
        case MENU_PAGE_START: /* 发车 */
            if (Menu_Running == 0)      OLED_ShowString(4, 6, "IDLE ");
            else if (Menu_Running == 1) OLED_ShowString(4, 6, "RUN  ");
            else                        OLED_ShowString(4, 6, "DONE ");
            break;
        case 2: /* 标定: 显示左电感(通道3)的标定范围 */
            OLED_ShowNum(4, 5, ADC_MIN[3], 4);
            OLED_ShowNum(4, 12, ADC_MAX[3], 4);
            break;
        case 3: /* 距离/编码器 */
            if (Menu_DistRaw == 0)
            {
                OLED_ShowNum(2, 3, ADC_L, 3);      /* 左横置 PA7 */
                OLED_ShowNum(2, 11, ADC_ML, 3);    /* 中左竖置 PA6 */
                OLED_ShowNum(3, 3, ADC_R, 3);      /* 右横置 PA4 */
                OLED_ShowNum(3, 11, ADC_MR, 3);    /* 中右竖置 PA5 */
                OLED_ShowNum(4, 3, ADC_Strength, 2);
                OLED_ShowNum(4, 9, (uint16_t)ADC_H_StrengthF, 2);
                OLED_ShowNum(4, 14, (uint16_t)ADC_V_StrengthF, 2);
            }
            else if (Menu_DistRaw == 3)
            {
                /* 编码器自检: 左/右速度(mm/s) + 累计计数 */
                OLED_ShowSignedNum(2, 2, Speed_L_mms, 4);
                OLED_ShowSignedNum(2, 9, Speed_R_mms, 4);
                OLED_ShowSignedNum(3, 3, Encoder_L_Total, 6);
                OLED_ShowSignedNum(4, 3, Encoder_R_Total, 6);
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
        case 4: /* Kp: 误差 / 微分项 / 参数 / 输出差速 */
            OLED_ShowSignedNum(3, 2, (int32_t)Error, 4);
            OLED_ShowSignedNum(3, 8, (int32_t)D_Filt, 3);
            OLED_ShowNum(4, 5, (uint32_t)(kp * 10.0f + 0.5f), 3);
            OLED_ShowSignedNum(4, 11, Speed_Delta, 3);
            break;
        case 5: /* Kd: 同上 */
            OLED_ShowSignedNum(3, 2, (int32_t)Error, 4);
            OLED_ShowSignedNum(3, 8, (int32_t)D_Filt, 3);
            OLED_ShowNum(4, 5, (uint32_t)(kd * 10.0f + 0.5f), 3);
            OLED_ShowSignedNum(4, 11, Speed_Delta, 3);
            break;
        case 6: /* 模式页: 选中标记 + 三个开关值 */
            OLED_ShowNum(1, 8, (uint32_t)(Menu_Mode_Sel + 1), 1);
            OLED_ShowString(2, 1, (Menu_Mode_Sel == 0) ? ">" : " ");
            OLED_ShowString(3, 1, (Menu_Mode_Sel == 1) ? ">" : " ");
            OLED_ShowString(4, 1, (Menu_Mode_Sel == 2) ? ">" : " ");
            OLED_ShowNum(2, 12, Menu_HexDiaEn, 1);
            OLED_ShowNum(3, 12, Menu_ZigEn, 1);
            OLED_ShowNum(4, 12, Speed_Loop_En, 1);
            break;
    }
}
/* 运行显示页是否需要重画标签 */
uint8_t Menu_RunInited = 0;

/* 运行时显示页(发车后) - 不清屏, 只更新数据位置
 *
 * 布局(16 列 x 4 行):
 *   L 55 ML 02 MR 02     四路电感归一化值(横置 L/R, 竖置 ML/MR)
 *   R 55 E+0012 S45      右横置 / 循迹误差 / 综合场强
 *   T300 A296 L1/2       左轮目标速度 / 平均实际速度 / 已完成圈数
 *   S2 X03 D4800         赛道状态 / 十字计数 / 左轮占空比
 */
void Menu_RunDisplay(void)
{
    if (!Menu_RunInited)
    {
        OLED_Clear();
        OLED_ShowString(1, 1,  "L");
        OLED_ShowString(1, 5,  "ML");
        OLED_ShowString(1, 10, "MR");
        OLED_ShowString(2, 1,  "R");
        OLED_ShowString(2, 5,  "E");
        OLED_ShowString(2, 11, "S");
        OLED_ShowString(3, 1,  "T");
        OLED_ShowString(3, 6,  "A");
        OLED_ShowString(3, 11, "L");
        OLED_ShowString(3, 13, "/");
        OLED_ShowString(4, 1,  "S");
        OLED_ShowString(4, 4,  "X");
        OLED_ShowString(4, 8,  "D");
        Menu_RunInited = 1;
    }

    /* 第1行: 四路电感归一化值(0~100) */
    OLED_ShowNum(1, 2,  ADC_L,  3);
    OLED_ShowNum(1, 7,  ADC_ML, 3);
    OLED_ShowNum(1, 12, ADC_MR, 3);
    /* 第2行: 右横置 + 循迹误差 + 综合场强 */
    OLED_ShowNum(2, 2,  ADC_R, 3);
    OLED_ShowSignedNum(2, 6, (int32_t)Error, 4);
    OLED_ShowNum(2, 12, ADC_Strength, 2);
    /* 第3行: 左轮目标速度 / 平均实际速度 / 圈数进度 */
    OLED_ShowNum(3, 2,  Speed_Target_L, 3);
    OLED_ShowNum(3, 7,  Speed_Avg_mms, 3);
    OLED_ShowNum(3, 12, Menu_DoneLaps, 1);
    OLED_ShowNum(3, 14, Menu_TargetLaps, 1);
    /* 第4行: 赛道状态 / 十字计数 / 左轮占空比 / 速度环故障标志 */
    OLED_ShowNum(4, 2,  Track_State, 1);
    OLED_ShowNum(4, 5,  Menu_CrossCount, 2);
    OLED_ShowNum(4, 9,  Duty_L, 4);

    /* F1 = 测不到速度(已自动降级开环)  F2 = 编码器反接(已自动纠正)
     * 两个都是"车还能跑但配置有问题"的提示, 看到就停车检查编码器。 */
    if (Speed_Fault == 0)      OLED_ShowString(4, 14, "  ");
    else if (Speed_Fault == 1) OLED_ShowString(4, 14, "F1");
    else                       OLED_ShowString(4, 14, "F2");
}
