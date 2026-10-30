#include "Common_headfile.h"

/* ============================================================
 *  赛道元素识别 —— 复合(2 横 + 2 竖)电感布局版
 *
 *  布局与分工:
 *    两侧【横置】 L / R    —— 循迹主力(80%), 反映左右偏差
 *    中间【竖置】 ML / MR  —— 特殊元素探测器(20%), 直道上接近 0,
 *                             到十字/环岛磁场叠加处才显著抬头
 *
 *  【本版最重要的改动】
 *  旧判据拿"四路最大值/最小值"互相比较(四路齐平 -> 十字)。在复合布局下
 *  这是错的: 竖置在直道上本来就接近 0, 永远不可能和横置"齐平", 十字会
 *  【永远检不出来】; 反过来只要竖置一抬头, 又会误判。
 *
 *  现在改为"按类型各自判":
 *    · 十字路口 -> 主要看【竖置自己】有没有抬到它峰值的一定比例
 *                  (竖置的峰值就是在十字处标定的, 所以这是它本职工作);
 *    · 六角菱形环 -> 看【横置】是否强右偏, 且【竖置】也抬头(环内回线)。
 *    横置与竖置的绝对值从不直接比较。
 * ============================================================ */

uint8_t  HexDia_Flag = 0;       /* 六角菱形环状态机 */
uint16_t HexDia_Timer = 0;      /* 六角菱形环计时 */
uint8_t  Cross_Stage = 0;       /* 十字路口检测阶段 */
uint16_t Cross_Timer = 0;       /* 十字路口计时 */
uint8_t  Diamond_Stage = 0;     /* 菱形环状态机(保留) */
uint16_t Diamond_Timer = 0;

/* 元素对控制的输出 */
uint8_t  Track_Straight = 0;    /* 1 = 强制直行(十字路口) */
int16_t  Track_Steer_Bias = 0;  /* 叠加到差速上的转向偏置 */
float    Track_Speed_Ratio = 1.0f; /* 元素期间的速度系数 */

/* ------------------------------------------------------------
 *  阈值
 * ------------------------------------------------------------ */
#define ABS_MIN        4     /* 绝对下限: 归一化值低于此视为噪声 */

/* 十字路口 —— 竖置电感的本职工作 */
#define CROSS_V_MIN      40    /* 竖置归一化 >= 40(其峰值标定为 100, 即达峰值40%) */
#define CROSS_V_BASE_MIN 8     /* 竖置巡航基线高于此值时才启用"相对抬升"判断 */
#define CROSS_V_RISE     1.8f  /* 竖置须达到自身巡航基线的 1.8 倍 */
#define CROSS_BAL        45    /* 横置左右偏差(%)须 <= 45 才认十字 —— 见下方说明 */
#define CROSS_CONFIRM    4     /* 至少连续 4 周期(20ms)才算特征成立 */
#define CROSS_GAP_MAX    5     /* 特征允许间断 5 周期, 防止中途断开重新计时 */
#define CROSS_HOLD       24    /* 保持直行至少 24 周期(120ms) */
#define LAP_MIN_TICKS    45    /* "特征"持续超过 45 周期(225ms)判为起点线 */

/* 六角菱形环(赛道右侧进入) */
#define HEX_R_DOMINANT   45    /* 横置右偏程度 >= 45% 才可能是右弯类元素 */
#define HEX_V_MIN        20    /* 竖置抬头下限(环内回线让竖置也起来) */
#define HEX_CONFIRM      10    /* 入口确认周期(太短会在普通右弯误触发) */
#define HEX_TURN_MAX     40    /* 进环最长 40 周期(200ms) */
#define HEX_IN_MAX       220   /* 环内最长 220 周期(1.1s) */
#define HEX_EXIT_MAX     60    /* 出环最长 60 周期(300ms) */
#define HEX_BACK_DIFF    35    /* 出环: 导线从左侧重新出现的落差阈值(%) */
#define HEX_BIAS_IN      (-900)  /* 进环转向偏置(负=右转) */
#define HEX_BIAS_KEEP    (-300)  /* 环内保持偏置 */
#define HEX_BIAS_SEARCH  (-700)  /* 环内丢线时的找线偏置(转更多) */
#define HEX_BIAS_OUT     (-500)  /* 出环偏置 */
#define HEX_SPD_IN       0.60f
#define HEX_SPD_KEEP     0.72f
#define HEX_SPD_SEARCH   0.58f
#define HEX_SPD_OUT      0.70f

/* 横置/竖置各自的巡航基线: 只在非元素期间缓慢跟随, 反映"平时这个类型的
 * 电平有多高"。竖置平时接近 0, 十字处猛升 -> 相对抬升倍数就是很好的判据。
 * 注意是【各类型各一个基线】, 绝不混算。 */
float H_Base = 0.0f;    /* 横置基线 */
float V_Base = 0.0f;    /* 竖置基线 */
#define BASE_ALPHA 0.01f

/* ------------------------------------------------------------
 *  十字路口: 竖置电感抬升 = 磁场叠加 = 十字/环岛的交叉点
 *
 *  为什么还要加"横置左右对称"这一条:
 *    90° 直角弯处, 前方那根横着的导线同样会从中间竖置电感下面扫过,
 *    竖置一样会抬头 —— 只凭竖置会把直角弯误判成十字, 车被强制直行冲出去。
 *    区分点: 十字路口是【两根导线同时存在】, 左右横置都被激励 -> 基本对称;
 *    直角弯是车正在转, 导线偏向一侧 -> 横置左右明显不对称。
 *    注意这里比的是【横置自己内部】的 L 与 R, 仍然没有和竖置互比。
 * ------------------------------------------------------------ */
static uint8_t Cross_Pattern(void)
{
    float v = ADC_V_StrengthF;
    float hsum = ADC_Lf + ADC_Rf;
    float hbias;

    /* 竖置必须明显抬头: 这是复合布局下十字的核心特征 */
    if (v < (float)CROSS_V_MIN) return 0;

    /* 若竖置平时也有可观读数(基线偏高), 还要求相对自身基线明显抬升 */
    if (V_Base > (float)CROSS_V_BASE_MIN && v < V_Base * CROSS_V_RISE) return 0;

    /* 横置必须有信息, 且在左右方向上基本对称(排除直角弯) */
    if (hsum < 2.0f) return 0;
    hbias = fabsf(ADC_Lf - ADC_Rf) / hsum * 100.0f;
    if (hbias > (float)CROSS_BAL) return 0;

    return 1;
}

/* ------------------------------------------------------------
 *  十字路口状态机
 * ------------------------------------------------------------ */
void Cross_Judge(void)
{
    static uint16_t hold = 0;
    static uint16_t gap = 0;
    static uint16_t pat = 0;    /* 特征真正成立的周期数(不含间断容忍) */

    Track_Straight = 0;

    switch (Cross_Stage)
    {
        case 0:
            if (Cross_Pattern())
            {
                Cross_Stage = 1;
                Cross_Timer = 0;
                gap = 0;
                pat = 0;
            }
            break;

        case 1:
            Cross_Timer++;
            if (Cross_Pattern())
            {
                gap = 0;
                pat++;
                /* 特征持续够久 -> 宽的起点线, 直接计圈(不必等驶出) */
                if (pat >= LAP_MIN_TICKS)
                {
                    Menu_DoneLaps++;
                    Beep_On(100);
                    if (Menu_DoneLaps >= Menu_TargetLaps)
                    {
                        Menu_Running = 2;
                    }
                    hold = CROSS_HOLD;
                    Cross_Stage = 2;
                    Cross_Timer = 0;
                    gap = 0;
                }
            }
            else
            {
                gap++;
                if (gap > CROSS_GAP_MAX)
                {
                    /* 特征结束: 按"真正成立"的持续时间判定路口还是起点 */
                    if (pat >= LAP_MIN_TICKS)
                    {
                        Menu_DoneLaps++;
                        Beep_On(100);
                        if (Menu_DoneLaps >= Menu_TargetLaps)
                        {
                            Menu_Running = 2;
                        }
                    }
                    else if (pat >= CROSS_CONFIRM)
                    {
                        Menu_CrossCount++;
                        Beep_On(30);
                    }
                    hold = CROSS_HOLD;
                    Cross_Stage = 2;
                    Cross_Timer = 0;
                    gap = 0;
                }
            }
            break;

        case 2:
            /* 路口保持直行: 不许 PD 乱打方向, 直到驶出路口特征 */
            if (hold > 0) hold--;
            if (hold > 0 || Cross_Pattern())
            {
                Track_Straight = 1;
                if (hold == 0) hold = CROSS_HOLD;   /* 还在路口, 续期 */
            }
            else
            {
                Cross_Stage = 0;
                Cross_Timer = 0;
            }
            break;

        default:
            Cross_Stage = 0;
            Cross_Timer = 0;
            break;
    }
}

/* ------------------------------------------------------------
 *  菱形环判断 (暂不启用, 保留接口)
 * ------------------------------------------------------------ */
void Diamond_Judge(void)
{
    Diamond_Stage = 0;
    Diamond_Timer = 0;
}

/* ============================================================
 *  六角菱形环 (赛道右侧进入, 顺时针绕行)
 *
 *  入口特征(复合布局下):
 *    ① 横置强右偏: R 明显强于 L  -> 说明车正被拉向右弯
 *    ② 竖置也抬头: 环内回线从侧面扫过, 竖置不会像普通右弯那样一直是 0
 *    两条同时成立才认, 所以普通右弯(竖置仍接近 0)不会误触发。
 *
 *  环内: 多边形有尖角, 电感可能长时间压不到导线, 不能死等固定时间。
 *        丢线就继续右转去找线; 导线从【左侧】重新出现说明绕完一圈。
 * ============================================================ */

/* 入口 */
static uint8_t HexDia_Entry_Pattern(void)
{
    float hsum = ADC_Lf + ADC_Rf;
    float rdom;
    float v = ADC_V_StrengthF;

    if (hsum < 2.0f) return 0;                       /* 横置没信息 */
    if (ADC_Rf < ADC_Lf) return 0;                   /* 右横置必须是较强侧 */
    rdom = (ADC_Rf - ADC_Lf) / hsum * 100.0f;        /* 横置自身归一化右偏度 */
    if (rdom < (float)HEX_R_DOMINANT) return 0;
    if (v < (float)HEX_V_MIN) return 0;              /* 竖置没抬头 -> 普通右弯 */
    return 1;
}

/* 绕完一圈: 导线从左侧重新出现(横置自身归一化) */
static uint8_t HexDia_Back_Pattern(void)
{
    float hsum = ADC_Lf + ADC_Rf;
    float ldom;

    if (hsum < 2.0f) return 0;
    if (ADC_Lf < ADC_Rf) return 0;
    ldom = (ADC_Lf - ADC_Rf) / hsum * 100.0f;
    if (ldom < (float)HEX_BACK_DIFF) return 0;
    return 1;
}

/* 环内完全压不到导线 */
static uint8_t HexDia_Lost(void)
{
    return (ADC_StrengthF < (float)ABS_MIN) ? 1 : 0;
}

void HexDia_Judge(void)
{
    if (Menu_HexDiaEn == 0)
    {
        HexDia_Flag = 0;
        HexDia_Timer = 0;
        return;
    }

    /* 十字路口优先: 路口期间不认环 */
    if (Cross_Stage != 0)
    {
        return;
    }

    switch (HexDia_Flag)
    {
        case 0:
            if (HexDia_Entry_Pattern())
            {
                if (++HexDia_Timer >= HEX_CONFIRM)
                {
                    HexDia_Flag = 1;
                    HexDia_Timer = 0;
                    Beep_On(40);
                }
            }
            else
            {
                HexDia_Timer = 0;
            }
            break;

        case 1:
            /* 进环: 强制右转 */
            HexDia_Timer++;
            Track_Steer_Bias = HEX_BIAS_IN;
            Track_Speed_Ratio = HEX_SPD_IN;
            if (HexDia_Timer >= HEX_TURN_MAX)
            {
                HexDia_Flag = 2;
                HexDia_Timer = 0;
            }
            break;

        case 2:
            /* 环内: 维持右转; 丢线时转更多去找线; 导线从左侧出现即绕完 */
            HexDia_Timer++;
            if (HexDia_Lost())
            {
                Track_Steer_Bias = HEX_BIAS_SEARCH;
                Track_Speed_Ratio = HEX_SPD_SEARCH;
            }
            else
            {
                Track_Steer_Bias = HEX_BIAS_KEEP;
                Track_Speed_Ratio = HEX_SPD_KEEP;
            }
            if (HexDia_Back_Pattern())
            {
                HexDia_Flag = 3;
                HexDia_Timer = 0;
            }
            else if (HexDia_Timer >= HEX_IN_MAX)
            {
                HexDia_Flag = 3;
                HexDia_Timer = 0;
            }
            break;

        case 3:
            /* 出环: 减小转向, 等误差回中 */
            HexDia_Timer++;
            Track_Steer_Bias = HEX_BIAS_OUT;
            Track_Speed_Ratio = HEX_SPD_OUT;
            if ((HexDia_Timer >= HEX_EXIT_MAX) ||
                (fabsf(Error) < 14.0f && HexDia_Timer > 10))
            {
                HexDia_Flag = 0;
                HexDia_Timer = 0;
                Track_Steer_Bias = 0;
                Track_Speed_Ratio = 1.0f;
                Beep_On(50);
            }
            break;

        default:
            HexDia_Flag = 0;
            HexDia_Timer = 0;
            break;
    }
}

/* ------------------------------------------------------------
 *  圈数判断已并入 Cross_Judge (靠"特征持续时间"区分起点线与普通路口)
 * ------------------------------------------------------------ */
void Lap_Judge(void)
{
}

/* ============================================================
 *  赛道状态机
 *
 *  0=直道  1=普通弯  2=直角弯  3=近距离双直道/S弯
 *  4=十字  5=六角菱形环  6=半圆弧  7=锯齿
 *
 *  为什么要单独分类, 而不是只看误差大小:
 *    误差只告诉你"偏了多少", 不告诉你"接下来会怎样"。
 *    同样 |E|=50, 在半个圆弧上应该稳住方向小油门过,
 *    在锯齿上却必须加大阻尼并且允许方向翻转。
 *    所以速度系数和 D 增益按【赛道类型】给, 而不是只按误差给。
 *
 *  切换带滞回: 新状态连续确认 2 个周期(10ms)才切,
 *  S弯/锯齿一旦进入至少保持 300ms, 防止在临界点来回抖。
 * ============================================================ */

#define FLIP_ERR_MIN       20.0f   /* 方向反转算数的误差下限 */
#define FLIP_FAST_MS       250     /* 两次反转间隔 < 250ms 算"快速抖动" */
#define FLIP_SLOW_MS       900     /* 两次反转间隔 < 900ms 算"慢速换向" */
#define FLIP_FAST_CNT      3       /* 快速反转 3 次 -> 锯齿 */
#define FLIP_SLOW_CNT      2       /* 慢速反转 2 次 -> S弯/双直道 */
#define FLIP_FORGET_MS     1500    /* 超过 1.5s 没反转, 计数清零 */
#define CURVE_STATE_TH     22.0f   /* 进入"普通弯"的误差门槛 */
#define ARC_ERR_MIN        40.0f   /* 半圆弧: 误差要持续大于此值 */
#define ARC_MAX_ERR       80.0f    /* 超过这个更像直角弯, 不算圆弧 */
#define ARC_HOLD_MS        400     /* 半圆弧: 持续 400ms 才认定 */
#define STATE_CONFIRM      2       /* 新状态确认周期数(5ms/次) */
#define STATE_MIN_MS       300     /* S弯/锯齿的最小保持时间 */

uint8_t  Track_State = TRK_STRAIGHT;
float    Track_Kd_Gain = 1.0f;
uint16_t Track_State_Ms = 0;      /* 当前状态已持续的时间(ms) */

static uint8_t  s_cand = TRK_STRAIGHT;   /* 候选状态 */
static uint8_t  s_cand_cnt = 0;
static uint8_t  s_flip_fast = 0;
static uint8_t  s_flip_slow = 0;
static uint16_t s_since_flip = 0;
static uint16_t s_arc_ms = 0;
static int8_t   s_last_sign = 0;
static uint16_t s_quiet_ms  = 0;
/* 复位状态机(发车时调用, 避免带上一轮的残留状态) */
void Track_StateMachine_Reset(void)
{
    Track_State       = TRK_STRAIGHT;
    Track_State_Ms    = 0;
    Track_Speed_Ratio = 1.0f;
    Track_Kd_Gain     = 1.0f;
    s_cand      = TRK_STRAIGHT;
    s_cand_cnt  = 0;
    s_flip_fast = 0;
    s_flip_slow = 0;
    s_since_flip = 0;
    s_arc_ms    = 0;
    s_last_sign = 0;
    s_quiet_ms  = 0;
}

/* 把状态翻译成"速度系数 + D 增益" */
static void Track_Apply(uint8_t st)
{
    float ratio = 1.0f;
    float kd_g  = 1.0f;

    switch (st)
    {
        case TRK_STRAIGHT: ratio = 1.00f; kd_g = 1.0f; break;
        case TRK_CURVE:    ratio = 1.00f; kd_g = 1.0f; break;  /* 降速由误差调度负责 */
        case TRK_ARC:      ratio = 0.90f; kd_g = 1.1f; break;  /* 半圆弧: 稳住, 别猛打 */
        case TRK_S_CURVE:  ratio = 0.65f; kd_g = 1.6f; break;  /* 双直道/S弯: 加阻尼防超调 */
        case TRK_ZIGZAG:   ratio = 0.55f; kd_g = 1.8f; break;  /* 锯齿: 更要阻尼 */
        case TRK_SHARP:    ratio = 1.00f; kd_g = 1.0f; break;  /* 急弯自己的 0.55 系数已生效 */
        case TRK_CROSS:    ratio = 1.00f; kd_g = 1.0f; break;
        case TRK_HEXDIA:   ratio = 1.00f; kd_g = 1.0f; break;
        default:           ratio = 1.00f; kd_g = 1.0f; break;
    }

    /* 十字/环/急弯自身已经给过速度系数, 这里不重复乘, 只更新 D 增益 */
    if ((st != TRK_CROSS) && (st != TRK_HEXDIA) && (st != TRK_SHARP))
    {
        Track_Speed_Ratio = ratio;
    }
    Track_Kd_Gain = kd_g;
}

static void Track_Classify(void)
{
    float   a  = fabsf(Error);
    int8_t  sg = (Error >  1.0f) ?  1 : ((Error < -1.0f) ? -1 : 0);
    uint8_t want;
    uint8_t holdable;

    /* ---- 方向反转统计: 区分 S弯(慢换向) 与 锯齿(快抖动) ---- */
    s_since_flip += 5;
    if (s_since_flip > FLIP_FORGET_MS)
    {
        s_flip_fast = 0;
        s_flip_slow = 0;
        s_since_flip = 0;
    }

    if ((sg != 0) && (s_last_sign != 0) && (sg != s_last_sign) && (a > FLIP_ERR_MIN))
    {
        if      (s_since_flip < FLIP_FAST_MS) s_flip_fast++;
        else if (s_since_flip < FLIP_SLOW_MS) s_flip_slow++;
        s_since_flip = 0;
    }
    if (sg != 0) s_last_sign = sg;

    /* 回中后很快清空反转计数: 否则 S弯/锯齿会拖尾到后面的直道上 */
    if (a < 10.0f)
    {
        if (s_quiet_ms < 60000) s_quiet_ms += 5;
        if (s_quiet_ms > 200)
        {
            s_flip_fast = 0;
            s_flip_slow = 0;
        }
    }
    else
    {
        s_quiet_ms = 0;
    }

    /* ---- 半圆弧: 同一方向的大误差持续很久 ---- */
    if ((a > ARC_ERR_MIN) && (sg != 0))
    {
        if (s_arc_ms < 65000) s_arc_ms += 5;
    }
    else
    {
        s_arc_ms = 0;
    }

    /* ---- 分类 ----
     * 优先级: 元素 > 方向反转(锯齿/S弯) > 半圆弧 > 直角弯 > 普通弯 > 直道
     * 反转是"确凿证据", 必须排在圆弧前面:
     * 锯齿和 S 弯同样会持续大误差, 若让圆弧优先会永远进不了锯齿状态。 */
    if      (Cross_Stage  != 0)            want = TRK_CROSS;
    else if (HexDia_Flag  != 0)            want = TRK_HEXDIA;
    else if (s_flip_fast >= FLIP_FAST_CNT) want = TRK_ZIGZAG;
    else if (s_flip_slow >= FLIP_SLOW_CNT) want = TRK_S_CURVE;
    /* 半圆弧与直角弯的区分: 两者误差都大, 但圆弧的误差"稳在中间"
     * (约 45~80), 直角弯往往直接顶到 90~100。 */
    else if ((s_arc_ms >= ARC_HOLD_MS) && (a < ARC_MAX_ERR)) want = TRK_ARC;
    else if (Sharp_State  != 0)            want = TRK_SHARP;
    else if (a > CURVE_STATE_TH)           want = TRK_CURVE;
    else                                   want = TRK_STRAIGHT;

    /* ---- 状态切换: 确认计数 + 最小保持时间 ---- */
    holdable = (Track_State == TRK_S_CURVE) || (Track_State == TRK_ZIGZAG);

    if (want == Track_State)
    {
        s_cand_cnt = 0;
        s_cand = Track_State;
    }
    else if (holdable && (Track_State_Ms < STATE_MIN_MS) && (want != TRK_CROSS) &&
             (want != TRK_HEXDIA) && (want != TRK_SHARP))
    {
        /* S弯/锯齿还在最小保持期内, 且新状态不是元素 -> 先不切 */
        s_cand_cnt = 0;
    }
    else if (want == s_cand)
    {
        if (++s_cand_cnt >= STATE_CONFIRM)
        {
            Track_State = want;
            Track_State_Ms = 0;
            s_cand_cnt = 0;
            /* 注意: 这里【不清】反转计数。
             * 清掉的话, 刚进 S弯的下一轮判据就找不到证据,
             * 会被"直角弯"(误差 > 41)抢回去, 状态在 3 和 2 之间来回跳。
             * 计数只靠"安静 200ms"或"1 秒内没反转"自然衰减。 */
        }
    }
    else
    {
        s_cand = want;
        s_cand_cnt = 0;
    }

    if (Track_State_Ms < 65000) Track_State_Ms += 5;

    Track_Apply(Track_State);
}

/* 总判断函数, 在中断中调用 */
void Track_Judge(void)
{
    Track_Steer_Bias = 0;
    Track_Speed_Ratio = 1.0f;

    Cross_Judge();
    Diamond_Judge();
    HexDia_Judge();
    Lap_Judge();
    Track_Classify();      /* 赛道状态分类(直道/弯/急弯/S弯/圆弧/锯齿...) */

    /* 横置/竖置各自的巡航基线: 只在"没有任何元素状态"时才更新,
     * 这样它反映的是平日巡航时的电平, 而不是元素处的峰值。 */
    if ((Cross_Stage == 0) && (HexDia_Flag == 0))
    {
        if (H_Base <= 0.0f)
        {
            H_Base = ADC_H_StrengthF;           /* 首次取值 */
        }
        else
        {
            H_Base += (ADC_H_StrengthF - H_Base) * BASE_ALPHA;
        }

        if (V_Base <= 0.0f)
        {
            V_Base = ADC_V_StrengthF;
        }
        else
        {
            V_Base += (ADC_V_StrengthF - V_Base) * BASE_ALPHA;
        }
    }
}
