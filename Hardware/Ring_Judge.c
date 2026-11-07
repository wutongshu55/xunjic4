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
 *
 *  2026-11 按公开方案补充的三条(本车没有陀螺仪, 角度一律用编码器):
 *    · 十字出口: "路口特征消失"就是出口 —— 交叉线甩到车后(竖置连续回落)
 *      就提前交还 PD, 不必每次都等满 hold 延时;
 *    · 环防连判: 出环后 HEX_COOLDOWN(1s) 内不接受入口特征 —— 出环过程中
 *      横置还带着偏侧, 立即重判容易把"正在出环"当成"又一次入环";
 *    · 环出环门槛: 对侧导线重现之外, 还要求编码器航向(Yaw_Deg, 左右轮速差
 *      积分)沿环方向累计绕够 HEX_YAW_EXIT_MIN 度 —— 环内摆动把对侧导线
 *      短暂带到强侧时不提前出环; 编码器失效(Yaw_Valid=0)自动退回时间门槛。
 * ============================================================ */

uint8_t  HexDia_Flag = 0;       /* 六角菱形环状态机 */
uint16_t HexDia_Timer = 0;      /* 六角菱形环计时 */
uint16_t HexDia_Cool = 0;       /* 出环冷却剩余周期(防连判, 菜单判定页可见) */
uint8_t  Cross_Stage = 0;       /* 十字路口检测阶段 */
uint16_t Cross_Timer = 0;       /* 十字路口计时 */
static uint16_t s_cross_rel = 0;    /* 十字提前出路口的回落计数(复位用) */
static float    s_hex_yaw0 = 0.0f;  /* 入环时刻的编码器航向(出环门槛基准) */

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
#define CROSS_ERR_MAX    35.0f /* 横置误差 > 35 = 正在转弯, 不认十字。
                                * 90° 弯的交叉导线同样会让竖置抬头, 以前只靠
                                * 横置对称性排除, 会误判成路口强制直行 120ms,
                                * 表现就是"直角弯转弯延迟高 / 冲出去才打方向"。 */
#define CROSS_V_STRONG   70    /* 竖置特别强 = 确定是交叉点, 放宽对称性 */
#define CROSS_BAL_STRONG 60    /* 强竖置时允许的横置偏差上限 */
#define CROSS_CONFIRM    4     /* 至少连续 4 周期(20ms)才算特征成立 */
#define CROSS_GAP_MAX    5     /* 特征允许间断 5 周期, 防止中途断开重新计时 */
#define CROSS_HOLD       24    /* 保持直行至少 24 周期(120ms) */
#define CROSS_HOLD_MAX   240   /* 直行最长 240 周期(1.2s)。
                                * 若竖置电感误判(阈值太低/装太近/标定坏),
                                * 特征会"一直成立", 没有上限车就永远直行不拐弯。
                                * 300mm/s 下 1.2s 走 36cm, 任何起点线都够通过。 */

/* 十字出口(提前结束直行): 通用做法是"路口特征消失 = 出口"。
 * 交叉线甩到车后, 竖置会从峰值连续回落; 与其干等 hold 满 120ms,
 * 不如确认回落就交还 PD —— 路口后紧接弯道时能早 ~90ms 打方向。 */
#define CROSS_V_RELEASE   20   /* 竖置回落到该值以下视为交叉线已过
                                 * (竖置巡航基线偏高时改用 基线*1.3) */
#define CROSS_RELEASE_MIN 16   /* 本次直行至少已持续 16 周期(80ms)才允许提前结束 */
#define CROSS_RELEASE_CNT 6    /* 竖置连续 6 周期(30ms)回落才提前结束 */

/* 说明: 早期版本在这里按"特征持续多久"区分普通十字与起点线, 并计圈,
 * 跑满目标圈数就自动停车。现已按需求【全部删除】:
 *   · 十字路口、起点线、折返线 —— 一律只"保持直行通过";
 *   · 不再计圈, 不存在"到圈数自动停车";
 *   · 唯一能让车停下来的只有按 KEY4。
 * 运行页第 4 行的 X 只是路口计数(看识别有没有工作), 不参与任何控制。 */

/* 六角菱形环(赛道右侧进入) */
#define HEX_DIR          (+1)   /* 入环方向: +1 = 右转入环(环在赛道右侧, 默认)
                                 *          -1 = 左转入环(整台状态机镜像)。
                                 * 现场看车行进方向: 环在右手边就 +1, 左手边就 -1 */
#define HEX_R_DOMINANT   45    /* 靠环侧横置优势 >= 45% 才可能是环入口 */
#define HEX_V_MIN        20    /* 竖置抬头下限(环内回线让竖置也起来) */
#define HEX_CONFIRM      10    /* 入口确认周期(太短会在普通右弯误触发) */
#define HEX_TURN_MAX     40    /* 进环最长 40 周期(200ms) */
#define HEX_IN_MAX       220   /* 环内最长 220 周期(1.1s) */
#define HEX_EXIT_MAX     60    /* 出环最长 60 周期(300ms) */
#define HEX_BACK_DIFF    35    /* 出环: 导线从对侧重新出现的落差阈值(%) */
#define HEX_BIAS_IN      (-1000) /* 进环转向偏置(负=右转, 乘 HEX_DIR 镜像), 加大转角 */
#define HEX_BIAS_KEEP    (-500)  /* 环内保持偏置, 六边形每段都要转够 */
#define HEX_BIAS_SEARCH  (-800)  /* 环内丢线时的找线偏置(转更多) */
#define HEX_BIAS_OUT     (-650)  /* 出环偏置 */

/* 出环冷却(防连判): 出环完成后一段时间内不接受入口特征。
 * 出环过程中横置仍带着偏侧、竖置也可能没落干净, 立即重判容易把
 * "正在出环"当成"又一次入环"。参考公开方案: 出环后缓冲 1 秒。 */
#define HEX_COOLDOWN     200    /* 200 周期 = 1s */

/* 出环还要"确实绕了大半圈": 环内摆动可能把对侧导线短暂带到强侧,
 * 只看落差会提前出环。没有陀螺仪 —— 用编码器左右轮速差积分出的航向
 * (Yaw_Deg)代替陀螺仪积分: 入环时记下航向, 出环特征要求航向沿环方向
 * 累计超过 HEX_YAW_EXIT_MIN 度才接受。
 * 编码器不可用(Yaw_Valid=0)时自动退回"环内至少 HEX_IN_MIN 周期"。 */
#define HEX_YAW_EXIT_MIN 150.0f /* 出环需要的最小航向累计(度) */
#define HEX_IN_MIN       40     /* 无航向时的出环最短环内周期(200ms) */
/* 靠环侧 / 对侧的横置电感(随 HEX_DIR 自动镜像) */
#define HEX_NEAR(ad_l, ad_r)  ((HEX_DIR) > 0 ? (ad_r) : (ad_l))
#define HEX_FAR(ad_l, ad_r)   ((HEX_DIR) > 0 ? (ad_l) : (ad_r))
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

    /* 横置误差大 = 车正在转弯, 绝不是十字(根治 90° 弯被强制直行导致转晚) */
    if (fabsf(Error_H) > (float)CROSS_ERR_MAX) return 0;

    /* 竖置必须明显抬头: 这是复合布局下十字的核心特征 */
    if (v < (float)CROSS_V_MIN) return 0;

    /* 若竖置平时也有可观读数(基线偏高), 还要求相对自身基线明显抬升 */
    if (V_Base > (float)CROSS_V_BASE_MIN && v < V_Base * CROSS_V_RISE) return 0;

    /* 横置两路都要有实质信号: 真十字是两根线同时激励。
     * 直角弯只有单侧导线, 常出现一侧很强另一侧接近 0 -> 在这里被排除。 */
    if ((ADC_Lf < 20.0f) || (ADC_Rf < 20.0f)) return 0;

    /* 横置必须有信息, 且在左右方向上基本对称(排除直角弯)。
     * 竖置特别强时放宽对称要求, 避免真实十字因轻微偏心被漏掉(没成绩)。 */
    if (hsum < 2.0f) return 0;
    hbias = fabsf(ADC_Lf - ADC_Rf) / hsum * 100.0f;
    if (v >= (float)CROSS_V_STRONG)
    {
        if (hbias > (float)CROSS_BAL_STRONG) return 0;
    }
    else if (hbias > (float)CROSS_BAL)
    {
        return 0;
    }

    return 1;
}

/* ------------------------------------------------------------
 *  十字路口状态机
 * ------------------------------------------------------------ */
/* 发车时调用: 复位路口状态机(不再涉及圈数) */
void Cross_Reset(void)
{
    Cross_Stage = 0;
    Cross_Timer = 0;
    s_cross_rel = 0;
}

void Cross_Judge(void)
{
    static uint16_t hold = 0;
    static uint16_t gap = 0;
    static uint16_t pat = 0;    /* 特征真正成立的周期数(不含间断容忍) */

    Track_Straight = 0;

    /* 直角/大弯优先: 已确认 90°/110° 或正在转弯时, 绝不强制直行。
     * 这是"右直角弯冲出去"的根治点 —— 直角弯入口的竖置也会抬头,
     * 以前会被 Cross_Pattern 判成十字, 强制直行 120ms, 车就冲过去了。 */
    if ((Curve_Grade_Now >= 2) || Curve_Turn_Now)
    {
        Cross_Stage = 0;
        Cross_Timer = 0;
        hold = 0;
        gap  = 0;
        pat  = 0;
        return;
    }

    /* 环内【绝不允许】十字抢控制:
     * 环内回线让竖置抬头、横置又可能接近对称, 正好满足十字特征。
     * 一旦 Track_Straight=1, PID_Apply 的十字分支会提前 return,
     * 把环状态机的转向偏置整个吞掉 -> 车在环里一直直行, 永远绕不出去。
     * 环外的十字仍照常强制直行, 不受影响。 */
    if (HexDia_Flag != 0)
    {
        Cross_Stage = 0;
        Cross_Timer = 0;
        hold = 0; gap = 0; pat = 0; s_cross_rel = 0;
        return;
    }

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

            /* 特征一出现就先"软直行": 确认要 4 个周期(20ms), 这 20ms 里
             * PD 已经可能被交叉导线打出一个大方向, 等确认完再关门就晚了,
             * 车会斜着穿出路口。先直行, 误检最多损失 20ms。 */
            Track_Straight = 1;

            if (Cross_Pattern())
            {
                gap = 0;
                pat++;
                /* 特征确认 -> 就是路口(宽的起点线也一样), 保持直行通过 */
                if (pat >= CROSS_CONFIRM)
                {
                    Menu_CrossCount++;      /* 只是计数显示, 不参与控制 */
                    hold = CROSS_HOLD;
                    Cross_Stage = 2;
                    Cross_Timer = 0;
                    gap = 0;
                    s_cross_rel = 0;
                }
            }
            else
            {
                gap++;
                if (gap > CROSS_GAP_MAX)
                {
                    /* 特征结束: 成立过就是路口, 没成立过当噪声丢掉 */
                    if (pat >= CROSS_CONFIRM)
                    {
                        Menu_CrossCount++;
                        hold = CROSS_HOLD;
                        Cross_Stage = 2;
                        s_cross_rel = 0;
                    }
                    else
                    {
                        Cross_Stage = 0;
                    }
                    Cross_Timer = 0;
                    gap = 0;
                }
            }
            break;

        case 2:
            /* 路口保持直行: 不许 PD 乱打方向, 直到驶出路口特征。
             * 但无论如何不能超过 CROSS_HOLD_MAX —— 误判时保证车还能拐弯。 */
            if (hold > 0) hold--;
            Cross_Timer++;                        /* 本次直行总时长(含续期) */
            if ((hold > 0 || Cross_Pattern()) && (Cross_Timer < CROSS_HOLD_MAX))
            {
                Track_Straight = 1;
                if (hold == 0) hold = CROSS_HOLD;   /* 还在路口, 续期 */

                /* 提前出路口: 交叉线已甩到车后(竖置连续回落到巡航电平)。
                 * 出口判据 = "路口特征消失", 与入口判据互为镜像;
                 * 至少已直行 CROSS_RELEASE_MIN, 且回落要连续确认, 防止
                 * 路口中间的瞬时凹谷把车提前放出去。 */
                {
                    float rel_th = (V_Base > (float)CROSS_V_BASE_MIN)
                                 ? V_Base * 1.3f : (float)CROSS_V_RELEASE;

                    if ((Cross_Timer >= CROSS_RELEASE_MIN) &&
                        (ADC_V_StrengthF < rel_th))
                    {
                        if (s_cross_rel < 60000) s_cross_rel++;
                        if (s_cross_rel >= CROSS_RELEASE_CNT)
                        {
                            Cross_Stage = 0;
                            Cross_Timer = 0;
                            s_cross_rel = 0;
                            break;
                        }
                    }
                    else
                    {
                        s_cross_rel = 0;
                    }
                }
            }
            else
            {
                Cross_Stage = 0;
                Cross_Timer = 0;
                s_cross_rel = 0;
            }
            break;

        default:
            Cross_Stage = 0;
            Cross_Timer = 0;
            break;
    }
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
    if (HEX_NEAR(ADC_Lf, ADC_Rf) < HEX_FAR(ADC_Lf, ADC_Rf)) return 0;
    rdom = (HEX_NEAR(ADC_Lf, ADC_Rf) - HEX_FAR(ADC_Lf, ADC_Rf)) / hsum * 100.0f;
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
    if (HEX_FAR(ADC_Lf, ADC_Rf) < HEX_NEAR(ADC_Lf, ADC_Rf)) return 0;
    ldom = (HEX_FAR(ADC_Lf, ADC_Rf) - HEX_NEAR(ADC_Lf, ADC_Rf)) / hsum * 100.0f;
    if (ldom < (float)HEX_BACK_DIFF) return 0;
    return 1;
}

/* 出环判据: 导线重新被两路横置电感稳定压住(左右接近对称)。
 * 旧判据用 fabsf(Error)<14 —— Error 是低通后的差比和, 环出口偏差本来就大,
 * 加上低通滞后, 车已经回到主线了 Error 还很大 -> 一直卡在出环状态出不去。
 * 直接看横置两路是否都有信号且基本对称, 不受滤波滞后影响。 */
static uint8_t HexDia_Recovered(void)
{
    float hsum = ADC_Lf + ADC_Rf;

    if (hsum < 2.0f) return 0;
    return (fabsf(ADC_Lf - ADC_Rf) / hsum * 100.0f < 25.0f) ? 1 : 0;
}

/* 环内完全压不到导线 */
static uint8_t HexDia_Lost(void)
{
    return (ADC_StrengthF < (float)ABS_MIN) ? 1 : 0;
}

/* 出环的航向门槛: 接受"对侧导线重现"前, 车必须真的绕了大半圈。
 * 有编码器航向 -> 看航向沿环方向的累计量; 没有 -> 退回时间门槛。 */
static uint8_t HexDia_Yaw_OK(void)
{
    if (Yaw_Valid)
    {
        float prog = (Yaw_Deg - s_hex_yaw0) * (float)HEX_DIR;
        return (prog >= HEX_YAW_EXIT_MIN) ? 1 : 0;
    }
    return (HexDia_Timer >= HEX_IN_MIN) ? 1 : 0;
}

/* 发车/调试时调用: 清空环状态机、出环冷却与航向基准。
 * 上一次跑车若中途停车, 残留的环状态会把入环偏置带进下一次发车。 */
void HexDia_Reset(void)
{
    HexDia_Flag = 0;
    HexDia_Timer = 0;
    HexDia_Cool = 0;
    s_hex_yaw0 = 0.0f;
}

void HexDia_Judge(void)
{
    if (Menu_HexDiaEn == 0)
    {
        HexDia_Flag = 0;
        HexDia_Timer = 0;
        HexDia_Cool = 0;
        return;
    }

    /* 冷却每个周期都在流逝, 与是否处于十字无关 */
    if (HexDia_Cool > 0) HexDia_Cool--;

    /* 十字路口不触发新环; 已进入的环状态仍必须继续计时并走完出环流程。 */
    if ((Cross_Stage != 0) && (HexDia_Flag == 0))
    {
        return;
    }

    switch (HexDia_Flag)
    {
        case 0:
            /* 冷却期内不接受入口特征(防连判) */
            if (HexDia_Cool > 0)
            {
                HexDia_Timer = 0;
                break;
            }
            if (HexDia_Entry_Pattern())
            {
                if (++HexDia_Timer >= HEX_CONFIRM)
                {
                    HexDia_Flag = 1;
                    HexDia_Timer = 0;
                    s_hex_yaw0 = Yaw_Deg;    /* 记下入环航向基准 */
                }
            }
            else
            {
                HexDia_Timer = 0;
            }
            break;

        case 1:
            /* 进环: 强制向环侧转 */
            HexDia_Timer++;
            Track_Steer_Bias = HEX_BIAS_IN * HEX_DIR;
            Track_Speed_Ratio = HEX_SPD_IN;
            if (HexDia_Timer >= HEX_TURN_MAX)
            {
                HexDia_Flag = 2;
                HexDia_Timer = 0;
            }
            break;

        case 2:
            /* 环内: 维持右转; 丢线时转更多去找线; 绕够角度且导线从对侧出现即绕完 */
            HexDia_Timer++;
            if (HexDia_Lost())
            {
                Track_Steer_Bias = HEX_BIAS_SEARCH * HEX_DIR;
                Track_Speed_Ratio = HEX_SPD_SEARCH;
            }
            else
            {
                Track_Steer_Bias = HEX_BIAS_KEEP * HEX_DIR;
                Track_Speed_Ratio = HEX_SPD_KEEP;
            }
            if ((HexDia_Back_Pattern() && HexDia_Yaw_OK()) ||
                (HexDia_Timer >= HEX_IN_MAX))
            {
                HexDia_Flag = 3;
                HexDia_Timer = 0;
            }
            break;

        case 3:
            /* 出环: 减小转向, 等误差回中 */
            HexDia_Timer++;
            Track_Steer_Bias = HEX_BIAS_OUT * HEX_DIR;
            Track_Speed_Ratio = HEX_SPD_OUT;
            if ((HexDia_Timer >= HEX_EXIT_MAX) ||
                (HexDia_Timer > 10 && HexDia_Recovered()))
            {
                HexDia_Flag = 0;
                HexDia_Timer = 0;
                Track_Steer_Bias = 0;
                Track_Speed_Ratio = 1.0f;
                HexDia_Cool = HEX_COOLDOWN;  /* 出环缓冲: 1s 内不再进环 */
            }
            break;

        default:
            HexDia_Flag = 0;
            HexDia_Timer = 0;
            break;
    }
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
    else if (Curve_Grade_Now >= 2)         want = TRK_SHARP;
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
    HexDia_Judge();
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
