#include "Common_headfile.h"

/* ===================== 全局运行参数 ===================== */
/* 是否进入六角菱形环: 0=不进 1=进
 * 本赛道(蓝底黑线场地)右侧有白色标记的菱形环, 默认开启;
 * 若现场发现普通直角弯被误判成环, 在菜单第 6 页 Mode 关掉。 */
uint8_t  Menu_HexDiaEn    = 1;
/* 是否已发车(0=待发车 1=运行中)
 * 注: 原来还有 2=跑满圈数自动停车, 已删除 —— 现在只能靠按 KEY4 停车。 */
uint8_t  Menu_Running     = 0;
/* 十字路口计数 */
uint8_t  Menu_CrossCount  = 0;
/* 菜单页索引 */
uint8_t  Menu_Page        = 0;

/* 电感标定模式 0=用固定标定值 1=运行中自动标定 */
uint8_t  Menu_CalibMode   = 0;
/* 急弯(锯齿线/直角弯)保持模式 0=关 1=开 */
uint8_t  Menu_ZigEn       = 1;
/* 距离测试页显示模式 0=归一化 1=原始 2=峰值 3=编码器 4=编码器引脚 */
uint8_t  Menu_DistRaw     = 0;

/* 速度控制方式: 【固定 20% PWM 开环】, 不再可选(车速选择已删除)。
 * 变量保留是为了让 Motor.c 的执行路径与主机端仿真正常工作。 */
uint8_t  Menu_SpdMode     = SPD_MODE_PWM;

/* 菜单页数(7 页):
 * 0=发车 1=标定 2=距离/编码器 3=Kp 4=Kd 5=模式开关(环/急弯/蜂鸣器) 6=滞后
 * 车速选择页已删除(车速固定 20% PWM), SpdMod 条目一并删除。 */
#define MENU_PAGES 7

/* 模式页当前选中的条目: 0=六角环 1=急弯 2=蜂鸣器模式 */
uint8_t Menu_Mode_Sel = 0;

/* 菜单是否需要重画标签 */
uint8_t Menu_Inited = 0;

/* 菜单按键处理: KEY1=上一页 KEY2=下一页 KEY3=数值- KEY4=数值+ */
void Menu_KeyHandler(uint8_t key)
{
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
                    Menu_CrossCount = 0;         /* 路口计数(只显示用) */

                    /* 发车前把闭环/状态机的状态清干净 */
                    Speed_Loop_Reset();          /* 速度环积分 */
                    Speed_Fault_Reset();         /* 故障标志重新检测 */
                    Track_StateMachine_Reset();  /* 赛道状态机 */
                    Cross_Reset();               /* 十字/路口状态机 */
                    PID_Apply_Reset();           /* 差速斜率/D 项残留清零 */
                    Menu_Running = 1;            /* 发车 */
                    Beep_On(200);
                }
            }
            break;

        case 1: /* 标定页: KEY3=重新标定  KEY4=固定/自动切换 */
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

        case 2: /* 距离/编码器页: KEY3=清零  KEY4=切换显示模式 */
            if (key == 3)
            {
                if (Menu_DistRaw >= 3)
                {
                    Encoder_L_Total = 0;    /* 模式4/5: 清零里程与脉冲计数 */
                    Encoder_R_Total = 0;
                    Enc_Pulse_L     = 0;
                    Enc_Pulse_R     = 0;
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
                if (Menu_DistRaw > 4) Menu_DistRaw = 0;
                Menu_Inited = 0;        /* 标签随模式变化, 强制重画 */
            }
            break;

        case 3: /* Kp 在线调参(横置/直道 PID) */
            if (key == 3) { if (kp > 1.0f)  kp -= 0.5f; }
            else          { if (kp < 60.0f) kp += 0.5f; }
            break;

        case 4: /* Kd 在线调参(横置/直道 PID) */
            if (key == 3) { if (kd > 0.0f)  kd -= 0.5f; }
            else          { if (kd < 30.0f) kd += 0.5f; }
            break;

        case 5: /* 模式页: KEY3=选择条目  KEY4=切换该条目的值 */
            if (key == 3)
            {
                Menu_Mode_Sel++;
                if (Menu_Mode_Sel > 2) Menu_Mode_Sel = 0;
                Beep_On(40);
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
                    /* 蜂鸣器模式: 0=无源方波 1=有源高电平 2=有源低电平
                     * 每种切换后立刻试鸣 300ms, 现场听到哪个响就用哪个 */
                    Beep_Mode++;
                    if (Beep_Mode > 2) Beep_Mode = 0;
                    Beep_On(300);
                    break;
                }
                Beep_On(60);
            }
            break;

        case 6: /* 滞后页: 前瞻延迟比例, 每按一次 0.05(5%) */
            if (key == 3)
            {
                Lag_Ratio -= 0.05f;
                if (Lag_Ratio < 0.0f) Lag_Ratio = 0.0f;
            }
            else
            {
                Lag_Ratio += 0.05f;
                if (Lag_Ratio > 1.0f) Lag_Ratio = 1.0f;
            }
            break;
    }
}

/* ------------------------------------------------------------
 *  Kp / Kd / Lag 三页共用布局(16x4), 原来三页各写一遍, 合并后少 30 行:
 *    行2: 标题 + 步进提示
 *    行3: PID 页 = 误差 E / D 项 / 航向 Y;  Lag 页 = 比例 R / 实际延迟 K
 *    行4: 参数值 + 输出差速 Dl
 * ------------------------------------------------------------ */
static void Menu_TuneLabels(uint8_t page)
{
    if (page == 6)
    {
        OLED_ShowString(2, 1, "Lag x100 K3-/K4+");
        OLED_ShowString(3, 1, "R");
        OLED_ShowString(3, 6, "K");
        OLED_ShowString(4, 1, "Lag:");
    }
    else
    {
        OLED_ShowString(2, 1, (page == 3) ? "Kp x10  K3-/K4+" : "Kd x10  K3-/K4+");
        OLED_ShowString(3, 1, "E");
        OLED_ShowString(3, 7, "D");
        OLED_ShowString(3, 11, "Y");
        OLED_ShowString(4, 1, (page == 3) ? "Kp:" : "Kd:");
    }
    OLED_ShowString(4, 9, "Dl");
}

static void Menu_TuneValues(uint8_t page)
{
    if (page == 6)
    {
        uint32_t lag = (uint32_t)(Lag_Ratio * 100.0f + 0.5f);

        OLED_ShowNum(3, 2, lag, 3);          /* 前瞻比例 ×100 */
        OLED_ShowNum(3, 7, Lag_Cycles, 3);   /* 当前实际延迟周期 */
        OLED_ShowNum(4, 5, lag, 3);
    }
    else
    {
        float k = (page == 3) ? kp : kd;

        OLED_ShowSignedNum(3, 2, (int32_t)Error_H, 4);
        OLED_ShowSignedNum(3, 8, (int32_t)D_Filt, 3);
        OLED_ShowSignedNum(3, 12, (int32_t)Yaw_Deg, 3);
        OLED_ShowNum(4, 5, (uint32_t)(k * 10.0f + 0.5f), 3);
    }
    OLED_ShowSignedNum(4, 11, Speed_Delta, 3);
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
            case MENU_PAGE_START: /* 发车页: 顺带显示三个可调参数, 不用逐页确认 */
                OLED_ShowString(2, 1, "Start");
                OLED_ShowString(3, 1, "Kp");
                OLED_ShowString(3, 6, "Kd");
                OLED_ShowString(3, 11, "L");
                OLED_ShowString(4, 1, "K4:GO");
                break;
            case 1: /* 标定: 当前模式 + 四路峰值(目标 3200~3600) */
                OLED_ShowString(2, 1, "Calib  M");
                OLED_ShowString(3, 1, "L");
                OLED_ShowString(3, 7, "R");
                OLED_ShowString(4, 1, "ML");
                OLED_ShowString(4, 9, "MR");
                break;
            case 2: /* 距离/编码器: 五种模式循环(KEY4 切换, KEY3 清零) */
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
                else if (Menu_DistRaw == 3)
                {
                    /* 编码器自检: 手推车 1 米, 看 L/R 速度和总计数是否在动 */
                    OLED_ShowString(1, 1, "= Enc  K3CLR=");
                    OLED_ShowString(2, 1, "L");
                    OLED_ShowString(2, 8, "R");
                    OLED_ShowString(3, 1, "CL");
                    OLED_ShowString(4, 1, "CR");
                }
                else
                {
                    /* 编码器引脚诊断: 手推车看 4 个脚的电平是否跳变 + 累计脉冲 */
                    OLED_ShowString(1, 1, "= EncPin K3CLR");
                    OLED_ShowString(2, 1, "LA B  RA B ");
                    OLED_ShowString(3, 1, "PL     PR");
                    OLED_ShowString(4, 1, "CPR    E  M ");
                }
                break;
            case 3: /* Kp / Kd: 同一套布局 */
            case 4:
                Menu_TuneLabels(Menu_Page);
                break;
            case 5: /* 模式页: 三个条目 */
                OLED_ShowString(1, 1, "= Mode 1/3 =");
                OLED_ShowString(2, 1, " HexDia     ");
                OLED_ShowString(3, 1, " Sharp      ");
                OLED_ShowString(4, 1, " Beep       ");
                break;
            case 6: /* 滞后页: 比例 + 实际延迟周期 */
                Menu_TuneLabels(6);
                break;
        }
        Menu_Inited = 1;
    }

    /* 只更新数值 */
    switch (Menu_Page)
    {
        case MENU_PAGE_START: /* 发车: Kp / Kd / Lag 一览 + 运行状态 */
            OLED_ShowNum(3, 3,  (uint32_t)(kp * 10.0f + 0.5f), 3);
            OLED_ShowNum(3, 8,  (uint32_t)(kd * 10.0f + 0.5f), 3);
            OLED_ShowNum(3, 12, (uint32_t)(Lag_Ratio * 100.0f + 0.5f), 3);
            OLED_ShowString(4, 7, (Menu_Running == 1) ? "RUN " : "IDLE");
            break;
        case 1: /* 标定: 模式 0=固定 1=自动 + 四路 MAX 峰值 */
            OLED_ShowNum(2, 9,  Menu_CalibMode, 1);
            OLED_ShowNum(3, 3,  ADC_MAX[3], 4);   /* L  PA7 */
            OLED_ShowNum(3, 9,  ADC_MAX[0], 4);   /* R  PA4 */
            OLED_ShowNum(4, 4,  ADC_MAX[2], 4);   /* ML PA6 */
            OLED_ShowNum(4, 12, ADC_MAX[1], 4);   /* MR PA5 */
            break;
        case 2: /* 距离/编码器 */
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
            else if (Menu_DistRaw == 4)
            {
                /* 引脚诊断: 电平位图 + 上电起的累计脉冲 + 每转计数/边沿/方式 */
                uint8_t lv = Encoder_ReadLevel();

                OLED_ShowNum(2, 3,  (uint32_t)((lv >> 0) & 0x01), 1);   /* PA0 左A */
                OLED_ShowNum(2, 5,  (uint32_t)((lv >> 1) & 0x01), 1);   /* PA1 左B */
                OLED_ShowNum(2, 9,  (uint32_t)((lv >> 2) & 0x01), 1);   /* PB6 右A */
                OLED_ShowNum(2, 11, (uint32_t)((lv >> 3) & 0x01), 1);   /* PB7 右B */
                OLED_ShowSignedNum(3, 3,  Enc_Pulse_L, 5);
                OLED_ShowSignedNum(3, 10, Enc_Pulse_R, 5);
                OLED_ShowNum(4, 4,  (uint32_t)Encoder_CPR, 3);
                OLED_ShowNum(4, 9,  (uint32_t)Encoder_Edge, 1);
                OLED_ShowNum(4, 12, (uint32_t)Encoder_Method, 1);
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
        case 3: /* Kp / Kd */
        case 4:
            Menu_TuneValues(Menu_Page);
            break;
        case 5: /* 模式页: 选中标记 + 三个条目的值 */
            OLED_ShowNum(1, 8, (uint32_t)(Menu_Mode_Sel + 1), 1);
            OLED_ShowString(2, 1, (Menu_Mode_Sel == 0) ? ">" : " ");
            OLED_ShowString(3, 1, (Menu_Mode_Sel == 1) ? ">" : " ");
            OLED_ShowString(4, 1, (Menu_Mode_Sel == 2) ? ">" : " ");
            OLED_ShowNum(2, 12, Menu_HexDiaEn, 1);
            OLED_ShowNum(3, 12, Menu_ZigEn, 1);
            OLED_ShowNum(4, 12, Beep_Mode, 1);
            break;
        case 6: /* 滞后页 */
            Menu_TuneValues(6);
            break;
    }
}
/* 运行显示页是否需要重画标签 */
uint8_t Menu_RunInited = 0;

static char *Menu_RunStateCode(void)
{
    if (Cross_Stage != 0 || Track_Straight) return "X";
    if (HexDia_Flag != 0) return "H";
    if (Sharp_State) return "R";
    if (Lost_State) return "L";

    switch (Track_State)
    {
        case TRK_STRAIGHT: return "S";
        case TRK_CURVE:    return "C";
        case TRK_SHARP:    return "R";
        case TRK_S_CURVE:  return "B";
        case TRK_CROSS:    return "X";
        case TRK_HEXDIA:   return "H";
        case TRK_ARC:      return "A";
        case TRK_ZIGZAG:   return "Z";
        default:           return "?";
    }
}

static char *Menu_RunTurnCode(void)
{
    if (Track_Straight || Cross_Stage != 0) return "F";
    if (Sharp_State) return (Sharp_Dir > 0) ? "R" : "L";
    if (Speed_Delta > 40) return "R";
    if (Speed_Delta < -40) return "L";
    return "F";
}

/* 运行时显示页(发车后) - 不清屏, 只更新数据位置
 *
 * 布局(16 列 x 4 行):
 *   L 55 ML 02 MR 02     四路电感归一化值(横置 L/R, 竖置 ML/MR)
 *   R 55 E+0012 S45      右横置 / 循迹误差 / 综合场强
 *   T300 A296 d+400      左轮目标速度 / 平均实际速度 / 循迹差速
 *   MRTFX03D4800F0       模式 / 转向 / 十字计数 / 左轮占空比 / 故障码
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
        OLED_ShowString(3, 11, "d");
        OLED_ShowString(4, 1,  "M");
        OLED_ShowString(4, 3,  "T");
        OLED_ShowString(4, 5,  "X");
        OLED_ShowString(4, 8,  "D");
        OLED_ShowString(4, 13, "F");
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
    /* 第3行: 左轮目标速度 / 平均实际速度 / 循迹差速(圈数已删除) */
    OLED_ShowNum(3, 2,  Speed_Target_L, 3);
    OLED_ShowNum(3, 7,  Speed_Avg_mms, 3);
    OLED_ShowSignedNum(3, 11, (int32_t)Speed_Delta, 3);
    /* 第4行: 状态码 / 当前转向 / 十字计数 / 左轮占空比 / 速度故障码 */
    OLED_ShowString(4, 2, Menu_RunStateCode());
    OLED_ShowString(4, 4, Menu_RunTurnCode());
    OLED_ShowNum(4, 6, Menu_CrossCount, 2);
    OLED_ShowNum(4, 9, Duty_L, 4);
    OLED_ShowNum(4, 14, Speed_Fault, 1);
}
