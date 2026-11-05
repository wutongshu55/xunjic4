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
 *     kp 15.0 -> 12.0    kd 5.0 -> 4.0
 *     弯道降速拐点 20/60 -> 12/36
 *     急弯饱和门限 55 -> 48, 回中门限 25 -> 15
 *  若以后改 W_OUT/W_IN 权重, 这些常数要按同一比例复查。
 * ============================================================ */

volatile float kp = 12.0f;      /* 菜单第 4 页: 横置(直道) PID 的 Kp
                                 * (11.0 × 1.09: 权重 4:1 -> 4:1.5 后纯横置误差
                                 *  缩小了 9%, 等比放大补偿, 直道手感不变) */
volatile float kd = 4.0f;       /* 菜单第 5 页: 横置(直道) PID 的 Kd (3.7 × 1.09) */
volatile int16_t Speed_Delta = 0;

/* ============ 双 PID: 横置管直道, 竖置管弯道 ============
 * 横置(L/R): 灵敏度高, 直道上反映"离线中心偏多少";
 *            但灵敏度高的代价是邻线经过时也会被拉一下 -> 直道 PID 增益适中。
 * 竖置(ML/MR): 指向性强, 前方有弯道/交叉才抬头, 邻线干扰几乎不响应,
 *            但幅值小 -> 弯道 PID 给更大的增益。
 * 融合: 竖置场强 ADC_V_StrengthF 越高说明越接近弯道,
 *       wv 从 0(纯横置)线性升到 1(纯竖置)。
 */
#define KP_V            16.0f   /* 弯道(竖置) PID 增益, 不进菜单 */
#define KD_V            4.0f
#define V_BLEND_TH      28.0f   /* 竖置场强达到此值(0~100)时完全用弯道 PID。
                                 * 阈值越低弯道 PID 介入越早(竖置比重越大):
                                 * 40=保守, 28=默认, 想再偏竖置就降到 20 */

/* D 项低通系数(横/竖各一路) */
#define D_LPF            0.30f

/* 差速上限("PWM@5000 等价"单位)。基准 20%(2000) 时 900 ≈ 45% 差速,
 * 直角弯转得动; 再大就会一侧轮子接近停转、车头瞬间甩过去。
 * 差速决定"车头转多快", 不是"转多远"。 */
#define DELTA_MAX        900

/* 差速斜率限制(每个 5ms 周期允许的变化量, 单位同上)。
 * PD 算出来的值可能一拍从 +200 跳到 +900, 机械上就是"猛打方向盘";
 * 限制斜率后 0 -> 900 需要 45ms, 车头是"转过去"而不是"甩过去"。
 * 回落比上升快: 松开方向要果断, 防止回正太慢蹭到外侧。 */
#define DELTA_SLEW_UP    100
#define DELTA_SLEW_DN    180

/* 弯道降速的两个拐点(误差绝对值): 调小 = 弯道判定更灵敏, 更早开始降速 */
#define CURVE_TH1        12.0f
#define CURVE_TH2        36.0f

/* ---- 前瞻补偿: 电感装在前轮轴前方约 12cm ----
 * 传感器现在"看到"的偏差, 车轮还要走一段距离才到; 立刻打角就会提前转弯
 * (切内弯、出弯偏外)。开源项目的标准做法是【误差延迟队列】:
 * 把误差存进环形队列, 延迟 K 个控制周期(5ms)后再给 PD:
 *   K = 补偿比例 × 前瞻距离 / (车速 × 0.005s)
 * 修正到达车轮时, 车正好走到当初看到偏差的位置, 车沿赛道本线走。
 * 补偿比例取 0.6 = "延后一点"而非完全几何对齐, 保留部分提前量。 */
#define LOOKAHEAD_MM     120.0f  /* 电感到前轮轴的纵向距离(mm), 按实车量 */
/* 前瞻补偿比例: 0 = 立即转弯, 1 = 完全几何对齐。
 * 【菜单第 7 页可调】默认 0.30 = 原 0.60 的一半。
 * 原因: 20% 车速下 0.60 约等于 360ms 延迟, 车已经冲出弯道、电感离开导线
 * 才把修正打到轮子上 -> 表现为"转弯永远晚半拍、转过头找不到线"。
 * 现场规则: 冲出去/切外弯 -> 调小; 直道来回摆、入弯切内角 -> 调大。 */
volatile float Lag_Ratio = 0.30f;
volatile uint8_t Lag_Cycles = 0;   /* 当前实际延迟周期数(菜单第 7 页显示) */
#define DELAY_BUF_MAX     96     /* 误差队列长度(> DELAY_K_MAX) */
#define DELAY_K_MAX       90     /* 最大延迟周期数(90 × 5ms = 450ms) */

/* ---- 速度调度总开关 ----
 * 0 = 全程恒定 20% PWM(当前要求): 弯道、丢线、出弯都不再加速/减速。
 * 1 = 恢复按误差/元素的自动降速(旧行为)。
 * 关掉调度后, 下面 CURVE_TH / SHARP_SPEED_RATIO / LOST_SPEED_RATIO 都不生效。 */
#define SPEED_SCHEDULE_EN 0

/* 丢线保护 */
#define LOST_CONFIRM     8       /* 连续 8 周期(40ms)低场强才判丢线 */
#define LOST_SPEED_RATIO 0.45f   /* 丢线时速度降到 45% */
#define LOST_SEARCH_DELTA 900    /* 找线时的差速 */
uint8_t  Lost_State = 0;
uint16_t Lost_Timer = 0;
int8_t   Last_Dir = 1;

/* 急弯(直角弯/锯齿)处理 —— 没有陀螺仪的弯道策略:
 *   进入: 横置误差饱和(车明显偏离线中心) 或 竖置误差很大(前方是弯)
 *         -> 锁定转向方向并立即持续转向;
 *   退出: 左右横置电感原始值都不低于 400 且基本相等, 连续确认 3 周期;
 *   保险: 锁太久(2s)强制交还 PD, 防止误判卡死。
 */
#define SHARP_TH         48.0f   /* 横置误差饱和阈值(调大 = 直角判定更严格) */
#define SHARP_V_TH       62.0f   /* 竖置误差达到此值也进入(弯道信号) */
#define SHARP_CONFIRM    3       /* 连续 3 周期满足才进入 */
#define SHARP_EXIT_ADC_MIN 400   /* 两个循迹横置电感都必须有足够信号 */
#define SHARP_EXIT_ADC_DIFF 60   /* 原始 ADC 读数允许的相等误差 */
#define SHARP_EXIT_CONFIRM 3     /* 连续 3 周期(15ms)满足条件才退出 */
#define SHARP_MIN_TURN_TICK 10   /* 至少先锁方向转 50ms, 避免入口对称信号误出弯 */
#define SHARP_MAX_TICK   400     /* 最长锁 400 周期(2s), 防误判卡死 */
#define SHARP_BIAS       450     /* 保持期间的固定转向量(编码器不可用时) */
#define SHARP_SPEED_RATIO 0.45f  /* 急弯降速比例; SPEED_SCHEDULE_EN=0 时不生效 */

/* ---- 直角弯两段式: 前瞻(直行) + 单轮急转 ----
 * 电感装在前轮前方, 会提前看到直角; 立刻打角就是提前转弯、半径大。
 * 第 1 段: 锁方向后先直行走完 SHARP_LEAD_MM, 等弯心到达车轮下方。
 *          这个值越大 = 前瞻越远、越晚打角、转弯半径越小。
 * 第 2 段: 在 Motor.c 里内轮降到接近停转、外轮保持基准, 车几乎绕内轮转。 */
#define SHARP_LEAD_MM    100.0f  /* 直角前瞻距离(mm); 现场按"转晚了/转早了"调 */
#define SHARP_LEAD_MAX   200     /* 前瞻计时上限(周期), 防速度估算异常时锁死 */

/* ---- 直角弯转向力度: 用编码器估算航向, 辅助调节转向 ----
 * 编码器航向只用于调节转向力度, 不再因角度到位而提前停止转向;
 * 最终出弯由横置电感检测, 避免直角弯转角不足。 */
#define SHARP_TURN_DEG   110.0f  /* 转向力度调节目标, 不是提前出弯角度 */
#define SHARP_ANG_KP       5.5f  /* 角度误差(度) -> 差速 */
#define SHARP_ANG_KD       1.2f  /* 角速度阻尼(度/秒) */
#define SHARP_TURN_MIN   450.0f  /* 持续锁方向的最小差速, 增大直角弯转向力度 */

uint8_t  Sharp_State = 0;
uint16_t Sharp_Timer = 0;
uint16_t Sharp_ExitCnt = 0;
uint16_t Sharp_Lead = 0;       /* >0 = 前瞻直行段(每 5ms 减 1) */
int8_t   Sharp_Dir = 1;
float    Sharp_Yaw0 = 0.0f;    /* 进入急弯时的航向基准(度) */

volatile float D_Filt   = 0.0f;   /* 横置 D 项(低通后), 菜单页显示 */
static  float D_V_Filt_ = 0.0f;   /* 竖置 D 项(低通后) */
static  float Dly_Prev_H = 0.0f;  /* 延迟队列输出的上周期横置样本(求 D 用) */
static  float Dly_Prev_V = 0.0f;  /* 延迟队列输出的上周期竖置样本(求 D 用) */

static float Eh_Buf[DELAY_BUF_MAX];   /* 横置误差延迟队列(环形) */
static float Ev_Buf[DELAY_BUF_MAX];   /* 竖置误差延迟队列(环形) */
static uint8_t Eh_Idx = 0;
static uint8_t Ev_Idx = 0;

/* ============================================================
 *  无陀螺仪航向估计 —— 用两个编码器的速度差当"电子陀螺"
 *
 *  差速转向的车满足刚体运动关系:
 *      ω = (vL - vR) / B          (rad/s, B = 左右轮中心距)
 *      ψ = ∫ω dt
 *  正 = 车头向右(顺时针)偏转; 5ms 积分一次, 单次转弯(≤2s)足够精确。
 *
 *  两条限制必须知道:
 *    1) 打滑/内轮悬空时估算会偏小 -> 只用来调节转向力度, 不用它精确对准,
 *       真正的出弯判据仍然靠电感;
 *    2) 没有绝对参考, 直道会漂 -> 非元素期间每 5ms 向 0 衰减 0.2%,
 *       等价于"以当前车头方向为零点"。
 * ============================================================ */
#define WHEEL_TRACK_MM   150.0f   /* 左右轮中心距(mm), 按实车量 */
#define YAW_MIN_SPEED     60.0f   /* 平均轮速低于此值不积分(量化噪声太大) */
#define YAW_LPF            0.45f  /* 角速度一阶低通 */

volatile float   Yaw_Deg       = 0.0f;
volatile float   Yaw_Rate_Dps  = 0.0f;
volatile uint8_t Yaw_Valid     = 0;

void PID_Yaw_Update(void)
{
    static float f = 0.0f;
    float vl  = (float)Speed_L_mms;
    float vr  = (float)Speed_R_mms;
    float avg = (vl + vr) * 0.5f;
    float w;

    if ((fabsf(avg) > YAW_MIN_SPEED) && ((vl * vr) > 0.0f))
    {
        /* 左轮比右轮快 = 车头向右转 */
        w = (vl - vr) / WHEEL_TRACK_MM * 57.29578f;
        f += (w - f) * YAW_LPF;
        Yaw_Rate_Dps = f;
        Yaw_Valid = 1;
    }
    else
    {
        f = 0.0f;
        Yaw_Rate_Dps = 0.0f;
        /* 不清 Yaw_Valid: 转弯中内侧轮瞬时停转是正常现象 */
    }

    Yaw_Deg += Yaw_Rate_Dps * 0.005f;

    /* 非元素期间缓慢回零: 没有绝对参考, 只保留单次转弯内的相对角度 */
    if ((Sharp_State == 0) && (Cross_Stage == 0) && (HexDia_Flag == 0))
    {
        Yaw_Deg *= 0.998f;
    }

    if (Yaw_Deg >  200.0f) Yaw_Deg =  200.0f;
    if (Yaw_Deg < -200.0f) Yaw_Deg = -200.0f;
}

/* 清 D 项: 十字/急弯结束后调用。
 * 否则 D 项带着区间内的跳变冲出尖峰, 表现就是"出路口猛甩一下"。 */
void PID_D_Reset(void)
{
    D_Filt     = 0.0f;
    D_V_Filt_  = 0.0f;
    Dly_Prev_H = 0.0f;
    Dly_Prev_V = 0.0f;
}

/* 差速斜率限制: 让差速"滑"到目标值, 而不是一拍跳到位 */
static int16_t s_delta_prev = 0;

static int16_t Delta_Slew(int16_t want)
{
    int16_t d   = (int16_t)(want - s_delta_prev);
    int16_t lim = (d > 0) ? DELTA_SLEW_UP : DELTA_SLEW_DN;

    if (d >  lim) d =  lim;
    if (d < -lim) d = -lim;
    s_delta_prev = (int16_t)(s_delta_prev + d);
    return s_delta_prev;
}

/* 发车/停车时复位输出整形与 D 项: 否则上一次跑车残留的差速会让车
 * 一起步就"带着方向盘"冲出去。Menu 发车流程里调用。 */
void PID_Apply_Reset(void)
{
    s_delta_prev = 0;
    PID_D_Reset();
}

/* 写入当前误差, 返回 k 个周期前的误差(k=0 时不延迟)。
 * D 项在【同一列延迟样本】上求差分, 保证 P/D 用的是同一时刻的信息。 */
static float Err_Delay(float *buf, uint8_t *idx, float now, uint8_t k)
{
    uint8_t r;
    float   old;

    if (k == 0)
    {
        buf[*idx] = now;
        *idx = (uint8_t)((*idx + 1) % DELAY_BUF_MAX);
        return now;
    }

    /* 先读 k 周期前的样本, 再写入当前样本, 时序才对应得上 */
    r = (uint8_t)((*idx + DELAY_BUF_MAX - k) % DELAY_BUF_MAX);
    old = buf[r];

    buf[*idx] = now;
    *idx = (uint8_t)((*idx + 1) % DELAY_BUF_MAX);
    return old;
}

/* ------------------------------------------------------------
 *  双 PD 计算 + 融合
 *    直道: Δ = kp*E_H + kd*dE_H     (菜单 kp/kd)
 *    弯道: Δ = KP_V*E_V + KD_V*dE_V (宏, 不进菜单)
 *    融合: 竖置场强越大权重越偏弯道
 * ------------------------------------------------------------ */
void PID_Calculate(void)
{
    float d_raw_h, d_raw_v;
    float kp_eff, kd_eff, speed_ratio;
    float dh, dv, wv;
    float eh_use, ev_use, lag, k_float;
    uint8_t k;

    /* ---- 前瞻补偿: 按当前车速算延迟周期数 ----
     * K = 补偿比例(Lag_Ratio) × 前瞻距离 / (车速 × 5ms)
     * 车速 200mm/s、比例 0.30 时 ≈ 36 个周期(约 180ms)。
     * 菜单第 7 页可在线改比例, 不用重新编译。 */
    lag = Lag_Ratio;
    if (lag < 0.0f) lag = 0.0f;
    if (lag > 1.0f) lag = 1.0f;

    if (lag == 0.0f)
    {
        k = 0;
    }
    else if (Speed_Basic > 50)
    {
        k_float = lag * LOOKAHEAD_MM / ((float)Speed_Basic * 0.005f);
        if (k_float >= (float)DELAY_K_MAX) k = DELAY_K_MAX;
        else k = (uint8_t)(k_float + 0.5f);
    }
    else
    {
        k = DELAY_K_MAX;
    }
    if (k > DELAY_K_MAX) k = DELAY_K_MAX;
    Lag_Cycles = k;

    /* ---- 横置(直道) PD: 误差先过前瞻延迟队列 ---- */
    eh_use  = Err_Delay(Eh_Buf, &Eh_Idx, Error_H, k);
    d_raw_h = eh_use - Dly_Prev_H;
    Dly_Prev_H = eh_use;
    D_Filt  = D_Filt + (d_raw_h - D_Filt) * D_LPF;

    /* 速度越低, 比例增益全额生效; Speed_Basic 现在固定 35% PWM 的等效速度 */
    speed_ratio = (float)Speed_Basic / 700.0f;
    if (speed_ratio > 1.0f) speed_ratio = 1.0f;
    if (speed_ratio < 0.0f) speed_ratio = 0.0f;

    kp_eff = kp * (1.0f - 0.15f * speed_ratio);
    kd_eff = kd * (1.0f + 0.35f * speed_ratio) * Track_Kd_Gain;
    dh = kp_eff * eh_use + kd_eff * D_Filt;

    /* ---- 竖置(弯道) PD: 同样延迟, 与横置同节拍 ---- */
    ev_use  = Err_Delay(Ev_Buf, &Ev_Idx, Error_V, k);
    d_raw_v = ev_use - Dly_Prev_V;
    Dly_Prev_V = ev_use;
    D_V_Filt_ = D_V_Filt_ + (d_raw_v - D_V_Filt_) * D_LPF;
    dv = KP_V * ev_use + KD_V * D_V_Filt_;

    /* ---- 融合: 直道全用横置, 弯道逐渐切到竖置 ---- */
    wv = ADC_V_StrengthF / V_BLEND_TH;
    if (wv > 1.0f) wv = 1.0f;

    Speed_Delta = (int16_t)(dh * (1.0f - wv) + dv * wv);
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
    float ah = fabsf(Error_H);
    float av = fabsf(Error_V);

    /* 菜单里可以关掉急弯保持(普通赛道不需要); 六角环内有自己的策略 */
    if (!Menu_ZigEn || (HexDia_Flag != 0))
    {
        Sharp_State = 0;
        Sharp_Timer = 0;
        Sharp_ExitCnt = 0;
        Sharp_Lead = 0;
        return;
    }

    /* 十字/交叉路口必须先直行通过, 急弯状态不能在路口锁住转向。 */
    if (Track_Straight || (Cross_Stage != 0))
    {
        Sharp_State = 0;
        Sharp_Timer = 0;
        Sharp_ExitCnt = 0;
        Sharp_Lead = 0;
        return;
    }

    if (!Sharp_State)
    {
        /* 进入: 横置误差饱和 或 竖置误差很大(前方弯道信号) */
        if ((ah > SHARP_TH) || (av > SHARP_V_TH))
        {
            if (Sharp_Timer < 60000) Sharp_Timer++;
            if (Sharp_Timer >= SHARP_CONFIRM)
            {
                Sharp_State = 1;
                Sharp_Timer = 0;
                Sharp_ExitCnt = 0;
                Sharp_Yaw0 = Yaw_Deg;   /* 记下进入时的航向, 用于调节转向力度 */
                /* 前瞻段: 按距离折算周期数, 弯心到车轮下之前保持直行 */
                if (Speed_Basic > 20)
                {
                    Sharp_Lead = (uint16_t)(SHARP_LEAD_MM /
                                 ((float)Speed_Basic * 0.005f) + 0.5f);
                }
                else
                {
                    Sharp_Lead = SHARP_LEAD_MAX;
                }
                if (Sharp_Lead > SHARP_LEAD_MAX) Sharp_Lead = SHARP_LEAD_MAX;
                /* 锁方向: 优先按竖置误差 —— 竖置看的是"前方弯往哪拐",
                 * 指向性强, 不受直角入口交叉导线干扰;
                 * 横置此时正被交叉线穿过, 符号不可信, 只作竖置无信号时的后备 */
                if (av > 5.0f)     Sharp_Dir = (Error_V > 0.0f) ? 1 : -1;
                else if (ah > 5.0f) Sharp_Dir = (Error_H > 0.0f) ? 1 : -1;
                Beep_On(160);  /* 蜂鸣器标志: 听到长一声 = 进入弯道(锁方向慢转) */
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
        if (Sharp_Lead > 0) Sharp_Lead--;   /* 前瞻直行段倒计时, 到 0 才打角 */

        /* 出弯判据: 两个循迹横置电感读数接近且都超过原始 ADC 400。
         * 传感器有噪声, 用 3 个控制周期确认; 确认期间仍保持转向。
         * 前瞻段不许出弯, 否则刚看到弯就撒手。 */
        if ((Sharp_Lead == 0) &&
            (Sharp_Timer >= SHARP_MIN_TURN_TICK) &&
            (ADC_Value[3] >= SHARP_EXIT_ADC_MIN) &&
            (ADC_Value[0] >= SHARP_EXIT_ADC_MIN) &&
            (abs((int)ADC_Value[3] - (int)ADC_Value[0]) <= SHARP_EXIT_ADC_DIFF))
        {
            if (Sharp_ExitCnt < 60000) Sharp_ExitCnt++;
            if (Sharp_ExitCnt >= SHARP_EXIT_CONFIRM)
            {
                Sharp_State = 0;
                Sharp_Timer = 0;
                Sharp_ExitCnt = 0;
                Sharp_Lead = 0;
                PID_D_Reset();  /* 清 D 项: 出弯瞬间不许带尖峰 */
                Beep_On(110);  /* 短一声 = 横置电感重新可靠循迹 */
            }
        }
        else
        {
            Sharp_ExitCnt = 0;
        }

        /* 安全退出: 锁太久说明判断错了, 交还给 PD */
        if (Sharp_Timer >= SHARP_MAX_TICK)
        {
            Sharp_State = 0;
            Sharp_Timer = 0;
            Sharp_ExitCnt = 0;
            Sharp_Lead = 0;
            PID_D_Reset();
        }
    }
}

/* ------------------------------------------------------------
 *  把 PD 输出、元素偏置、丢线/急弯策略合成为最终差速
 * ------------------------------------------------------------ */
/* 十字路口直行时的航向锁定增益。
 * 只把差速清 0 不够: 车进路口前已经有一点偏航, 清 0 只会让它斜着穿过去。
 * 用编码器航向做小闭环, 把车头锁在"进入路口那一刻的方向"上。
 * 编码器不可用(Yaw_Valid=0)时自动退回纯 0(原来的行为)。 */
#define CROSS_HOLD_KP    8.0f
#define CROSS_HOLD_KD    3.0f
#define CROSS_HOLD_MAX   350   /* 航向修正的最大差速, 别和转向抢戏 */
#define CROSS_LINE_KP    3.0f  /* 十字偏心小修正, 纠正弯道后斜着进路口 */
#define CROSS_LINE_KD    1.0f
#define CROSS_LINE_MAX   120   /* 保持直行的同时只允许小幅横向对中 */

int16_t PID_Apply(int16_t delta)
{
    int16_t out = delta;
    static uint8_t straight_prev = 0;
    static float   straight_yaw0 = 0.0f;

    /* ---- 十字路口: 直行 + 航向锁定 ----
     * 只在环外生效: 环内的回线特征不会被判成十字(见 Cross_Judge),
     * 这里再加一道保险, 防止残留的直行标记吞掉环内转向。 */
    if (Track_Straight && (HexDia_Flag == 0))
    {
        float line_trim = CROSS_LINE_KP * Error_H + CROSS_LINE_KD * D_Filt;
        if (line_trim >  (float)CROSS_LINE_MAX) line_trim =  (float)CROSS_LINE_MAX;
        if (line_trim < -(float)CROSS_LINE_MAX) line_trim = -(float)CROSS_LINE_MAX;

        if (!straight_prev) straight_yaw0 = Yaw_Deg;

        if (Yaw_Valid)
        {
            float u = -(CROSS_HOLD_KP * (Yaw_Deg - straight_yaw0)
                        + CROSS_HOLD_KD * Yaw_Rate_Dps) + line_trim;
            if (u >  (float)CROSS_HOLD_MAX) u =  (float)CROSS_HOLD_MAX;
            if (u < -(float)CROSS_HOLD_MAX) u = -(float)CROSS_HOLD_MAX;
            out = (int16_t)u;
        }
        else
        {
            out = (int16_t)line_trim;
        }

        straight_prev = 1;
        return Delta_Slew(out);
    }
    straight_prev = 0;

    /* ---- 环内: 转向完全由元素状态机接管 ----
     * 环内多边形/丢线时误差噪声极大, PD 输出常与进环方向相反;
     * 旧做法是 out + Track_Steer_Bias 叠加, 两者会互相抵消,
     * 再加上十字直行早退会整个吞掉偏置 -> 车在环里转不动、出不去。
     * 这里直接用状态机偏置作为差速, 不与 PD 叠加。 */
    if (HexDia_Flag != 0)
    {
        out = Track_Steer_Bias;
        if (out >  DELTA_MAX) out =  DELTA_MAX;
        if (out < -DELTA_MAX) out = -DELTA_MAX;
        return Delta_Slew(out);
    }

    if ((HexDia_Flag == 0) && Lost_State && !Sharp_State)
    {
        /* 丢线: 丢线时不能用传感器误差(噪声大), 沿最后方向低速找线
         * (在六角菱形环里不适用, 环内的丢线由环状态机自己处理) */
        out = (int16_t)(Last_Dir * LOST_SEARCH_DELTA);
    }
    else if ((HexDia_Flag == 0) && Sharp_State)
    {
        if (Yaw_Valid)
        {
            /* 航向用于控制转向力度; 达到目标角后仍维持最小差速,
             * 直到横置电感确认重新压在线上。 */
            float turned = (Yaw_Deg - Sharp_Yaw0) * (float)Sharp_Dir;
            float rem    = SHARP_TURN_DEG - turned;
            float u = SHARP_ANG_KP * rem
                    - SHARP_ANG_KD * Yaw_Rate_Dps * (float)Sharp_Dir;
            if (u < SHARP_TURN_MIN)   u = SHARP_TURN_MIN;
            if (u > (float)DELTA_MAX) u = (float)DELTA_MAX;
            out = (int16_t)((float)Sharp_Dir * u);
        }
        else
        {
            /* 编码器不可用: 锁方向持续转, 不让邻线引起反向抽动。 */
            out = (int16_t)(Sharp_Dir * SHARP_BIAS);
        }
    }

    /* 赛道元素(十字/圆环)附加的转向偏置 */
    out = (int16_t)(out + Track_Steer_Bias);

    /* 限幅: 差速量纲是 "PWM@5000 等价", 上限固定 900(≈基准的 26%)。
     * 注意这里不能用 Speed_Basic 相乘 —— Speed_Basic 现在是 mm/s,
     * 单位不同, 乘出来会大 10 倍。真正的 mm/s 换算在 CarMotor_Control() 里做。 */
    if (out >  DELTA_MAX) out =  DELTA_MAX;
    if (out < -DELTA_MAX) out = -DELTA_MAX;

    /* 斜率限制: 转向"转过去"而不是"甩过去" */
    return Delta_Slew(out);
}

/* ------------------------------------------------------------
 *  速度调度
 *
 *  当前要求: 全程恒定 20% PWM —— 弯道不减速、出弯不加速。
 *  SPEED_SCHEDULE_EN = 0 时直接原样返回, 下面的分档逻辑全部旁路。
 *  想恢复"弯道自动降速"只改这一个开关, 不用动其它地方。
 * ------------------------------------------------------------ */
int16_t PID_Speed_Schedule(int16_t basic)
{
#if (SPEED_SCHEDULE_EN == 0)
    (void)basic;
    return basic;                 /* 恒速: 20% PWM 一路到底 */
#else
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
        ratio = SHARP_SPEED_RATIO;    /* 弯道减速: 缓慢通过, 回中后自动恢复 */
    }
    else
    {
        if (a <= CURVE_TH1)
        {
            ratio = 1.0f;
        }
        else if (a <= CURVE_TH2)
        {
            /* 12~36 之间线性降到 70% */
            ratio = 1.0f - (a - CURVE_TH1) / (CURVE_TH2 - CURVE_TH1) * 0.30f;
        }
        else
        {
            /* 36 以上继续降到 50% */
            ratio = 0.70f - (a - CURVE_TH2) / (100.0f - CURVE_TH2) * 0.20f;
        }
    }

    ratio *= Track_Speed_Ratio;   /* 赛道元素的速度系数 */

    if (ratio < 0.20f) ratio = 0.20f;
    return (int16_t)((float)basic * ratio);
#endif
}
