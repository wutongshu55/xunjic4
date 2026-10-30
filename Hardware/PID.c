#include "Common_headfile.h"

/* ============================================================
 *  循迹控制器 (2026 优化版)
 *
 *  差速: 左轮 = 基础 + Δ, 右轮 = 基础 - Δ
 *
 *  【重要: 误差量纲已随布局改变】
 *  复合(2横+2竖)布局下, 误差改为"横置/竖置各自归一化后 80/20 加权合成",
 *  同样工况下的 |Error| 约为四路全横置旧版的 1.34 倍(仿真实测 52.94 -> 71.1)。
 *  原因是纯横置偏摆时竖置无信号(20% 权重那一路贡献 0), 分母仍是 5 份。
 *  因此本文件里【所有以 Error 为量纲的门限与增益都按 1/1.34 缩小】,
 *  保证整条环路的实际行为与旧版一致(否则车会明显更冲、更容易甩出)。
 *     kp 15.0 -> 11.0    kd 5.0 -> 3.7
 *     弯道降速拐点 20/60 -> 15/45
 *     急弯饱和门限 55 -> 41, 回中门限 25 -> 19
 *  若以后改 W_OUT/W_IN 权重, 这些常数要按同一比例复查。
 * ============================================================ */

volatile float kp = 11.0f;
volatile float kd = 3.7f;
volatile int16_t Speed_Delta = 0;

/* D 项低通系数 */
#define D_LPF            0.30f

/* 弯道降速的两个拐点(误差绝对值, 已按新量纲缩放) */
#define CURVE_TH1        15.0f
#define CURVE_TH2        45.0f

/* 丢线保护 */
#define LOST_CONFIRM     8       /* 连续 8 周期(40ms)低场强才判丢线 */
#define LOST_SPEED_RATIO 0.45f   /* 丢线时速度降到 45% */
#define LOST_SEARCH_DELTA 900    /* 找线时的差速 */
uint8_t  Lost_State = 0;
uint16_t Lost_Timer = 0;
int8_t   Last_Dir = 1;

/* 急弯(锯齿线/直角弯)保持 */
#define SHARP_TH         41.0f   /* 误差饱和阈值(新量纲) */
#define SHARP_CONFIRM    3       /* 连续 3 周期饱和才进入 */
#define SHARP_EXIT_TH    19.0f   /* 误差回落到此值以下算已回中 */
#define SHARP_EXIT_CNT   3       /* 连续 3 周期回中才退出 */
#define SHARP_MAX_TICK   60      /* 最长保持 60 周期(300ms) */
#define SHARP_BIAS       350     /* 保持期间额外转向偏置 */
#define SHARP_SPEED_RATIO 0.55f  /* 急弯时速度降到 55% */
uint8_t  Sharp_State = 0;
uint16_t Sharp_Timer = 0;
uint16_t Sharp_ExitCnt = 0;
int8_t   Sharp_Dir = 1;

volatile float D_Filt = 0.0f;

/* ------------------------------------------------------------
 *  PD 计算: Δ = kp*Error + kd*d(Error)
 *  D 项先低通, 并按车速做增益调度。
 * ------------------------------------------------------------ */
void PID_Calculate(void)
{
    float d_raw, kp_eff, kd_eff, speed_ratio;

    d_raw = Error - Error_Last;
    D_Filt = D_Filt + (d_raw - D_Filt) * D_LPF;

    /* 速度越高: 比例增益略降(防直线蛇形), 微分增益略升(加阻尼) */
    speed_ratio = (float)Speed_Basic / 7500.0f;
    if (speed_ratio > 1.0f) speed_ratio = 1.0f;
    if (speed_ratio < 0.0f) speed_ratio = 0.0f;

    kp_eff = kp * (1.0f - 0.15f * speed_ratio);
    kd_eff = kd * (1.0f + 0.35f * speed_ratio);

    Speed_Delta = (int16_t)(kp_eff * Error + kd_eff * D_Filt);
}

/* ------------------------------------------------------------
 *  丢线检测: 总场强(ADC_Strength)过低说明四路都压不到导线
 * ------------------------------------------------------------ */
void PID_LostLine_Check(void)
{
    if (fabsf(Error) > 11.0f)
    {
        Last_Dir = (Error > 0.0f) ? 1 : -1;    /* 记住最后偏差方向 */
    }

    if (ADC_Strength < TRACK_ONLINE_TH)
    {
        if (Lost_Timer < 60000) Lost_Timer++;
        if (Lost_Timer >= LOST_CONFIRM)
        {
            Lost_State = 1;
        }
    }
    else
    {
        Lost_Timer = 0;
        Lost_State = 0;
    }
}

/* ------------------------------------------------------------
 *  急弯检测: 误差长时间饱和 = 车头指向赛道边缘/直角弯/锯齿
 *  进入后锁定转向方向, 避免 PD 在两路传感器之间来回抽动
 * ------------------------------------------------------------ */
void PID_Sharp_Check(void)
{
    float a = fabsf(Error);

    /* 菜单里可以关掉急弯保持(普通赛道不需要) */
    if (!Menu_ZigEn)
    {
        Sharp_State = 0;
        Sharp_Timer = 0;
        Sharp_ExitCnt = 0;
        return;
    }

    if (!Sharp_State)
    {
        if (a > SHARP_TH)
        {
            if (Sharp_Timer < 60000) Sharp_Timer++;
            if (Sharp_Timer >= SHARP_CONFIRM)
            {
                Sharp_State = 1;
                Sharp_Timer = 0;
                Sharp_ExitCnt = 0;
                Sharp_Dir = (Error > 0.0f) ? 1 : -1;
            }
        }
        else
        {
            Sharp_Timer = 0;
        }
    }
    else
    {
        Sharp_Timer++;

        if (a < SHARP_EXIT_TH)
        {
            if (Sharp_ExitCnt < 60000) Sharp_ExitCnt++;
            if (Sharp_ExitCnt >= SHARP_EXIT_CNT)
            {
                Sharp_State = 0;
                Sharp_Timer = 0;
                Sharp_ExitCnt = 0;
            }
        }
        else
        {
            Sharp_ExitCnt = 0;
        }

        /* 安全退出: 保持太久说明判断错了, 交还给 PD */
        if (Sharp_Timer >= SHARP_MAX_TICK)
        {
            Sharp_State = 0;
            Sharp_Timer = 0;
            Sharp_ExitCnt = 0;
        }
    }
}

/* ------------------------------------------------------------
 *  把 PD 输出、元素偏置、丢线/急弯策略合成为最终差速
 * ------------------------------------------------------------ */
int16_t PID_Apply(int16_t delta)
{
    int16_t out = delta;
    int16_t maxd;

    if ((HexDia_Flag == 0) && Lost_State)
    {
        /* 丢线: 丢线时不能用传感器误差(噪声大), 沿最后方向低速找线
         * (在六角菱形环里不适用, 环内的丢线由环状态机自己处理) */
        out = (int16_t)(Last_Dir * LOST_SEARCH_DELTA);
    }
    else if ((HexDia_Flag == 0) && Sharp_State)
    {
        /* 急弯: 不允许反向抽动, 并叠加固定转向偏置 */
        if ((delta > 0 && Sharp_Dir < 0) || (delta < 0 && Sharp_Dir > 0))
        {
            delta = 0;
        }
        out = (int16_t)(delta + Sharp_Dir * SHARP_BIAS);
    }

    /* 赛道元素(十字/圆环)附加的转向偏置 */
    out = (int16_t)(out + Track_Steer_Bias);

    /* 限幅: 内轮保留 10% 牵引力, 防止单轮反转打滑 */
    maxd = (int16_t)((float)Speed_Basic * 0.90f);
    if (maxd < 100) maxd = 100;
    if (out > maxd) out = maxd;
    if (out < -maxd) out = -maxd;

    return out;
}

/* ------------------------------------------------------------
 *  速度调度: 误差越大速度越低; 丢线/急弯/元素各有自己的系数
 * ------------------------------------------------------------ */
int16_t PID_Speed_Schedule(int16_t basic)
{
    float a = fabsf(Error);
    float ratio = 1.0f;

    if (HexDia_Flag != 0)
    {
        ratio = 1.0f;        /* 环内速度由赛道元素模块单独给 */
    }
    else if (Lost_State)
    {
        ratio = LOST_SPEED_RATIO;
    }
    else if (Sharp_State)
    {
        ratio = SHARP_SPEED_RATIO;
    }
    else
    {
        if (a <= CURVE_TH1)
        {
            ratio = 1.0f;
        }
        else if (a <= CURVE_TH2)
        {
            /* 20~60 之间线性降到 70% */
            ratio = 1.0f - (a - CURVE_TH1) / (CURVE_TH2 - CURVE_TH1) * 0.30f;
        }
        else
        {
            /* 60 以上继续降到 50% */
            ratio = 0.70f - (a - CURVE_TH2) / (100.0f - CURVE_TH2) * 0.20f;
        }
    }

    ratio *= Track_Speed_Ratio;   /* 赛道元素的速度系数 */

    if (ratio < 0.20f) ratio = 0.20f;
    return (int16_t)((float)basic * ratio);
}
