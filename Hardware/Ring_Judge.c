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

/* 总判断函数, 在中断中调用 */
void Track_Judge(void)
{
    Track_Steer_Bias = 0;
    Track_Speed_Ratio = 1.0f;

    Cross_Judge();
    Diamond_Judge();
    HexDia_Judge();
    Lap_Judge();

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