#include "Common_headfile.h"

/* ===================== 全局运行参数 ===================== */
/* 是否进入六角菱形环: 0=不进 1=进
 * 【默认关】现场圆环先不进(避免把 90°/110° 弯误判成环)。
 * 十字路口不受影响: 环关掉后 Cross_Judge 正常工作, 仍然强制直行。 */
uint8_t  Menu_HexDiaEn    = 0;
/* 是否已发车(0=待发车 1=运行中)
 * 注: 原来还有 2=跑满圈数自动停车, 已删除 —— 现在只能靠按 KEY4 停车。 */
uint8_t  Menu_Running     = 0;
/* 十字路口计数 */
uint8_t  Menu_CrossCount  = 0;
/* 菜单页索引 */
uint8_t  Menu_Page        = 0;

/* 电感标定模式 0=用固定标定值 1=运行中自动标定 */
uint8_t  Menu_CalibMode   = 0;
/* 急弯(锯齿线/直角弯)保持模式 0=关 1=开
 * 【默认关】45°/90°/110° 由新的"按角度分级 + 过程实时判断"逻辑处理,
 * 不再锁方向(锁方向是转反/回不来的主要来源)。需要对比时在菜单第 6 页打开。 */
uint8_t  Menu_ZigEn       = 0;
/* 距离测试页显示模式 0=归一化 1=原始 2=峰值 3=编码器 4=编码器引脚 */
uint8_t  Menu_DistRaw     = 0;

/* 速度控制方式: 【固定 35% PWM 开环】, 不再可选(车速选择已删除)。
 * 变量保留是为了让 Motor.c 的执行路径与主机端仿真正常工作。 */
uint8_t  Menu_SpdMode     = SPD_MODE_PWM;

/* 菜单页数(7 页):
 * 0=发车 1=标定 2=距离/编码器 3=Kp 4=Kd 5=模式开关(环/急弯) 6=滞后
 * 车速选择页已删除(车速固定 35% PWM), SpdMod 条目一并删除。 */
#define MENU_PAGES 7

/* 模式页当前选中的条目: 0=六角环 1=急弯 */
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
                }
            }
            break;

        case 1: /* 标定页: KEY3=重新标定  KEY4=固定/自动切换 */
            if (key == 3)
            {
                ADC_Calib_Reset();
            }
            else
            {
                Menu_CalibMode = !Menu_CalibMode;
                ADC_AutoCalib  = Menu_CalibMode;
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
            }
            else
            {
                Menu_DistRaw++;
                if (Menu_DistRaw > 5) Menu_DistRaw = 0;   /* 新增模式5=弯道判定页 */
                Menu_Inited = 0;        /* 标签随模式变化, 强制重画 */
            }
            break;

        case 3: /* Kp 在线调参(横置/直道 PID), 上限 100 便于现场直接拧到 90 */
            if (key == 3) { if (kp > 1.0f)   kp -= 0.5f; }
            else          { if (kp < 100.0f) kp += 0.5f; }
            break;

        case 4: /* Kd 在线调参(横置/直道 PID), 上限 100 */
            if (key == 3) { if (kd > 0.0f)   kd -= 0.5f; }
            else          { if (kd < 100.0f) kd += 0.5f; }
            break;

        case 5: /* 模式页: KEY3=选择条目  KEY4=切换该条目的值(只剩 HexDia / Sharp) */
            if (key == 3)
            {
                Menu_Mode_Sel++;
                if (Menu_Mode_Sel > 1) Menu_Mode_Sel = 0;
            }
            else
            {
                if (Menu_Mode_Sel == 0)
                {
                    Menu_HexDiaEn = !Menu_HexDiaEn;
                }
                else
                {
                    Menu_ZigEn = !Menu_ZigEn;
                }
            }
            break;

        case 6: /* 滞后页: 0=不延迟(默认) 1=完全延迟, 每按一次 1 */
            if (key == 3)
            {
                Lag_Ratio -= 1.0f;
                if (Lag_Ratio < 0.0f) Lag_Ratio = 0.0f;
            }
            else
            {
                Lag_Ratio += 1.0f;
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
            case 1: /* 标定: 当前模式 + 四路峰值(目标 3200~3550) */
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
                    /* 峰值保持: 推到十字/环岛看峰值是否落在 3200~3550 */
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
                else if (Menu_DistRaw == 4)
                {
                    /* 编码器引脚诊断: 手推车看 4 个脚的电平是否跳变 + 累计脉冲 */
                    OLED_ShowString(1, 1, "= EncPin K3CLR");
                    OLED_ShowString(2, 1, "LA B  RA B ");
                    OLED_ShowString(3, 1, "PL     PR");
                    OLED_ShowString(4, 1, "CPR    E  M ");
                }
                else
                {
                    /* 弯道判定页: 推车全程录像看这一页就能对上每段赛道 */
                    OLED_ShowString(1, 1, "= Curve Judge=");
                    OLED_ShowString(2, 1, "EH");
                    OLED_ShowString(2, 8, "EV");
                    OLED_ShowString(3, 1, "G");
                    OLED_ShowString(3, 5, "T");
                    OLED_ShowString(3, 9, "C");
                    OLED_ShowString(4, 1, "d");
                    OLED_ShowString(4, 8, "E");
                }
                break;
            case 3: /* Kp / Kd: 同一套布局 */
            case 4:
                Menu_TuneLabels(Menu_Page);
                break;
            case 5: /* 模式页: 两个条目 */
                OLED_ShowString(1, 1, "= Mode 1/2 =");
                OLED_ShowString(2, 1, " HexDia     ");
                OLED_ShowString(3, 1, " Sharp      ");
                OLED_ShowString(4, 1, "            ");
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
            else if (Menu_DistRaw == 5)
            {
                /* 弯道判定页: EH/EV 各自误差, G=角度档(1=45 2=90 3=110),
                 * T=转弯进行中, C=同方向确认计数, d=输出差速, E=合成误差 */
                OLED_ShowSignedNum(2, 3,  (int32_t)Error_H, 4);
                OLED_ShowSignedNum(2, 10, (int32_t)Error_V, 4);
                OLED_ShowNum(3, 2, (uint32_t)Curve_Grade_Now, 1);
                OLED_ShowNum(3, 6, (uint32_t)Curve_Turn_Now, 1);
                OLED_ShowNum(3, 10, (uint32_t)Curve_Conf_Now, 2);
                OLED_ShowSignedNum(4, 3,  (int32_t)Speed_Delta, 4);
                OLED_ShowSignedNum(4, 10, (int32_t)Error, 4);
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
        case 5: /* 模式页: 选中标记 + 两个条目的值 */
            OLED_ShowNum(1, 8, (uint32_t)(Menu_Mode_Sel + 1), 1);
            OLED_ShowString(2, 1, (Menu_Mode_Sel == 0) ? ">" : " ");
            OLED_ShowString(3, 1, (Menu_Mode_Sel == 1) ? ">" : " ");
            OLED_ShowString(4, 1, " ");
            OLED_ShowNum(2, 12, Menu_HexDiaEn, 1);
            OLED_ShowNum(3, 12, Menu_ZigEn, 1);
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

/* ---- 运行页增量刷新 ----
 * 软件 I2C 刷新一个数字要几百微秒, 而 50ms 内多数数值并不变。
 * 值没变就跳过, 把 CPU 时间还给按键扫描与主循环。
 * 切进运行页时(Menu_RunInited=0)强制全刷一次。 */
#define RUN_SLOTS 12
static int32_t s_run_cache[RUN_SLOTS];
static char    s_run_state = 0, s_run_turn = 0;
static uint8_t s_run_valid = 0;

static void Run_Num(uint8_t slot, uint8_t line, uint8_t col,
                    int32_t v, uint8_t len, uint8_t sign)
{
    if (s_run_valid && s_run_cache[slot] == v) return;
    s_run_cache[slot] = v;
    if (sign) OLED_ShowSignedNum(line, col, v, len);
    else      OLED_ShowNum(line, col, (uint32_t)v, len);
}

/* 运行时显示页(发车后) - 不清屏, 只更新数据位置
 *
 * 布局(16 列 x 4 行):
 *   L 55 ML 02 MR 02     左横置 / 中左竖置 / 中右竖置 归一化
 *   R 55 E+0012 S45      右横置 / 循迹误差 / 综合场强
 *   L180 R180 d+000      左轮实测 / 右轮实测 / 循迹差速(左右分开)
 *   MSX03D3500F0         状态 / 转向 / 十字计数 / 左轮占空比 / 故障码
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
        OLED_ShowString(3, 1,  "L");
        OLED_ShowString(3, 6,  "R");
        OLED_ShowString(3, 11, "d");
        OLED_ShowString(4, 1,  "M");
        OLED_ShowString(4, 3,  "T");
        OLED_ShowString(4, 5,  "X");
        OLED_ShowString(4, 8,  "D");
        OLED_ShowString(4, 13, "F");
        s_run_valid    = 0;      /* 标签重画后所有数值都要刷一遍 */
        Menu_RunInited = 1;
    }

    /* 第1行: 左横置 / 中左竖置 / 中右竖置 归一化(0~100) */
    Run_Num(0, 1, 2,  ADC_L,  3, 0);
    Run_Num(1, 1, 7,  ADC_ML, 3, 0);
    Run_Num(2, 1, 12, ADC_MR, 3, 0);
    /* 第2行: 右横置 + 循迹误差 + 综合场强 */
    Run_Num(3, 2, 2,  ADC_R, 3, 0);
    Run_Num(4, 2, 6,  (int32_t)Error, 4, 1);
    Run_Num(5, 2, 12, ADC_Strength, 2, 0);
    /* 第3行: 左轮实测 / 右轮实测 / 循迹差速(左右分开) */
    Run_Num(6, 3, 2,  Speed_L_mms, 3, 1);
    Run_Num(7, 3, 7,  Speed_R_mms, 3, 1);
    Run_Num(8, 3, 12, (int32_t)Speed_Delta, 3, 1);
    /* 第4行: 状态码 / 转向码 / 十字计数 / 左轮占空比 / 故障码 */
    {
        char st = *Menu_RunStateCode();
        char tn = *Menu_RunTurnCode();

        if (!s_run_valid || st != s_run_state)
        {
            s_run_state = st;
            OLED_ShowString(4, 2, Menu_RunStateCode());
        }
        if (!s_run_valid || tn != s_run_turn)
        {
            s_run_turn = tn;
            OLED_ShowString(4, 4, Menu_RunTurnCode());
        }
    }
    Run_Num(9,  4, 6,  Menu_CrossCount, 2, 0);
    Run_Num(10, 4, 9,  Duty_L, 4, 0);
    Run_Num(11, 4, 14, Speed_Fault, 1, 0);

    s_run_valid = 1;
}
