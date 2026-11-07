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

/* 菜单第 4/5 页显示的是 ×10 值: 090 = 9.0
 * 现场要求 P/D 都给 90(显示值), 即 9.0 / 9.0:
 *   P 从 12 降到 9 -> 减小过冲("转弯幅度过大");
 *   D 从 4 升到 9 -> 加大阻尼, 转过了能更快拉回, 直道更稳。 */
volatile float kp = 9.0f;
volatile float kd = 9.0f;
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
#define V_BLEND_TH      22.0f   /* 竖置场强达到此值(0~100)时完全用弯道 PID。
                                 * 阈值越低弯道 PID 介入越早 -> 入弯不晚:
                                 * 40=保守, 28=旧默认, 22=现在(35% 车速下更早看到弯)。
                                 * 竖置对邻线不敏感, 调低比调高安全。 */

/* D 项低通系数(横/竖各一路) */
#define D_LPF            0.30f

/* ---- 直线上降低【中间两个竖置电感 ML/MR】的比重 ----
 * 直道上竖置读数多半是远端/邻线串扰, 让它们少参与打方向,
 * 车主要按外侧横置 L/R 沿主线走; 误差一大(真检测到路线变化)
 * 或竖置场强起来(进弯)立刻恢复全额, 不损失实时反应。 */
#define STRAIGHT_V_TH     20.0f   /* 横/竖误差都小于此值才算直道 */
#define STRAIGHT_V_SCALE   0.60f  /* 直道时竖置融合权重打六折 */

/* 差速总上限。25% 基准(2500)下 900 ≈ 36% 差速, 够转但不甩;
 * 分档限幅见 DELTA_CAP_45/90/110, 那几个才是真正管"转弯幅度"的。 */
#define DELTA_MAX        900

/* 差速斜率: 180 时 0 -> 1100 约 30ms, 路线一变立刻跟上;
 * 回落比上升更快, 转过头能迅速反向回中。 */
#define DELTA_SLEW_UP    180
#define DELTA_SLEW_DN    240

/* 弯道降速: 只保留"直角/大弯瞬间降速"这一条。
 * 45° 及以下不减速(直线/小弯保持流畅), 90°/110° 一确认立刻降到 55%,
 * 降速后按剩余误差一点点转, 转够立即恢复全速。 */
#define CURVE_SPEED_RATIO 0.6f

/* 【前瞻滞后 / 延迟反应 / 速度调度开关 已全部删除】
 * 原 CURVE_TH1/2、Lag_Ratio、Lag_Cycles、LOOKAHEAD_MM、LAG_DISP_MAX、
 * SPEED_SCHEDULE_EN 都不再存在: 误差直接进 PD, 不做任何人为延迟或平滑保持。 */

/* 丢线保护: 用【横置 L+R】判定, 比综合场强准 ——
 * 锯齿/直角中竖置抬头会让 ADC_Strength 很高, 综合场强判不出丢线。 */
#define LOST_CONFIRM     5       /* 连续 5 周期(25ms)横置都没信号才判丢线 */
#define LOST_H_SUM_TH    16.0f   /* 横置归一化 L+R 低于此 = 丢线 */
#define LOST_SPEED_RATIO 0.45f   /* 丢线时速度降到 45% */
#define LOST_SEARCH_DELTA 700    /* 找线差速(比原来的 900 柔和, 不甩) */
#define ZIG_SPEED_RATIO  0.75f   /* 连续锯齿弯: 略降速, 减少丢线 */
uint8_t  Lost_State = 0;
uint16_t Lost_Timer = 0;
int8_t   Last_Dir = 1;

/* 【急弯"锁方向保持"状态机已全部删除】
 * 原 SHARP_TH / Sharp_State / Sharp_Dir / Sharp_Timer / SHARP_EXIT_* /
 * 方向投票 / 保持计时等都不再存在。
 * 直角弯改由 Curve_Update() 的"按角度分级 + 转弯过程实时判断"处理:
 * 不锁方向、不做进入/退出确认、不保持, 转够或转错都当场收敛。 */

volatile float D_Filt   = 0.0f;   /* 横置 D 项(低通后), 菜单页显示 */
static  float D_V_Filt_ = 0.0f;   /* 竖置 D 项(低通后) */
static  float Dly_Prev_H = 0.0f;  /* 延迟队列输出的上周期横置样本(求 D 用) */
static  float Dly_Prev_V = 0.0f;  /* 延迟队列输出的上周期竖置样本(求 D 用) */

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
#define WHEEL_TRACK_MM   135.0f   /* 左右轮中心距(mm), 实测 13~14cm 取中值 */
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
    if ((Curve_Turn_Now == 0) && (Cross_Stage == 0) && (HexDia_Flag == 0))
    {
        Yaw_Deg *= 0.998f;
    }

    if (Yaw_Deg >  200.0f) Yaw_Deg =  200.0f;
    if (Yaw_Deg < -200.0f) Yaw_Deg = -200.0f;
}

/* ============================================================
 *  按角度分级的弯道判定(45° / 90° / 110°) + 转弯过程实时判断
 *
 *  为什么要分级: 以前只用一个阈值 SHARP_TH, 45° 和 110° 用同一套力度,
 *  结果 45° 打死、110° 又转不够。这里按【横置误差 |E_H|】分三档,
 *  并且要求"同方向持续 N 周期"才确认 —— 旁边那根线扫过通常只有一两拍
 *  或符号来回跳, 不会被当成真弯道打出大方向(邻线误判的根治点)。
 *
 *  转的角度够不够, 在【转的过程中】实时判断:
 *    进入 90°/110° 后开 s_turn_active;
 *    误差回落到 TURN_DONE_ERR 并连续 TURN_DONE_CNT 周期 -> 转够了, 立刻收油;
 *    误差反向且够大 -> 转过了, 立刻结束, 交还 PD 反向拉回。
 *  不使用固定时间、固定角度、固定前瞻段, 所以不会"转晚了/转过头回不来"。
 * ============================================================ */
#define CURVE_45_MIN     22.0f   /* >22 = 45° 级弯(轻弯) */
#define CURVE_90_MIN     47.0f   /* >47 = 90° 级弯(现场调过, 更早识别直角) */
#define CURVE_110_MIN    80.0f   /* >80 = 110° 级弯(最急) */
#define CURVE_CONFIRM    2       /* 同方向连续 2 周期(10ms)就确认, 更快进弯 */
#define NEIGH_CAP        350     /* 未确认的大误差(邻线/瞬态)差速上限 */
#define DELTA_CAP_45     600     /* 45° 允许的差速上限 */
#define DELTA_CAP_90     950     /* 90° 允许的差速上限 */
#define DELTA_CAP_110    900     /* 110° 允许的差速上限(与 DELTA_MAX 一致) */
#define STRAIGHT_DEADZONE 3.0f   /* 直道 ±3 误差死区: 只滤噪声, 超过照常实时纠正 */

/* 转弯过程: 起步柔和 -> 逐渐加力 -> 转够/转错立刻收(无任何保持) */
#define TURN_DONE_ERR    12.0f   /* 误差回落到这个以下 = 基本转够了 */
#define TURN_DONE_BAL    12.0f   /* 转弯中 |L-R| 归一化小于此 = 左右电感已一样 -> 停 */
#define TURN_DONE_LR_MIN 18.0f   /* 且两路都要在线, 避免"都没压到线"被当成平衡 */
#define TURN_DONE_CNT    3       /* 连续 3 周期(15ms)确认才收油 */
#define TURN_BIAS_K      4.0f    /* 转弯中按剩余误差补一点方向 */
#define TURN_BIAS_MAX    300
#define TURN_RAMP_TICK   10      /* 前 50ms 从 60% 力渐进到 100%: 一点一点转 */
#define TURN_WRONG_ERR   26.0f   /* 转弯中误差反向且超过此值 -> 可能转错 */
#define TURN_WRONG_CNT   3       /* 连续 3 拍 -> 判定转错, 立刻结束 */
#define TURN_BLOCK_TICK  24      /* 转错后 120ms 内不再进弯, 让 PD 反向拉回 */
#define TURN_MAX_TICK    400     /* 2s 保险, 防误判卡死 */

/* 供菜单"弯道判定页"显示, 拍视频看数据用 */
volatile uint8_t Curve_Grade_Now = 0;   /* 0=直道 1=45° 2=90° 3=110° */
volatile uint8_t Curve_Turn_Now  = 0;   /* 1=转弯进行中(还没判定转够) */
volatile uint8_t Curve_Conf_Now  = 0;   /* 当前同方向确认计数 */
volatile uint8_t Curve_Wrong_Now = 0;   /* 1=刚判定转错(封锁期) */

static uint8_t  s_curve_cnt   = 0;
static uint8_t  s_curve_grade = 0;
static int8_t   s_curve_sign  = 0;
static uint8_t  s_turn_active = 0;
static uint8_t  s_turn_done   = 0;
static uint8_t  s_turn_wrong  = 0;
static uint8_t  s_turn_block  = 0;
static int8_t   s_turn_dir    = 0;
static uint16_t s_turn_tick   = 0;

void PID_Curve_Reset(void)
{
    s_curve_cnt   = 0;
    s_curve_grade = 0;
    s_curve_sign  = 0;
    s_turn_active = 0;
    s_turn_done   = 0;
    s_turn_wrong  = 0;
    s_turn_block  = 0;
    s_turn_dir    = 0;
    s_turn_tick   = 0;
    Curve_Grade_Now = Curve_Turn_Now = Curve_Conf_Now = Curve_Wrong_Now = 0;
}

/* 每 5ms 调用一次。没有"保持/延迟": 每拍都按当前误差重判。 */
static void Curve_Update(float eh)
{
    float   a  = fabsf(eh);
    int8_t  sg = (eh > 1.0f) ? 1 : ((eh < -1.0f) ? -1 : 0);
    uint8_t g  = 0;

    if      (a >= CURVE_110_MIN) g = 3;
    else if (a >= CURVE_90_MIN)  g = 2;
    else if (a >= CURVE_45_MIN)  g = 1;

    if ((g > 0) && (sg != 0))
    {
        if (sg == s_curve_sign)
        {
            if (s_curve_cnt < 250) s_curve_cnt++;
        }
        else if (s_turn_active)
        {
            /* 连续锯齿/连续 90°: 反向是【下一个弯】, 不是噪声。
             * 不重置确认, 否则每个弯都要重新确认 -> 中间掉速、丢线。 */
            s_curve_sign = sg;
            if (s_curve_cnt < CURVE_CONFIRM) s_curve_cnt = CURVE_CONFIRM;
        }
        else
        {
            s_curve_sign = sg;       /* 方向一变就重新数, 邻线抖动进不了确认 */
            s_curve_cnt  = 1;
        }
    }
    else
    {
        s_curve_cnt  = 0;
        s_curve_sign = 0;
    }

    /* 分级直接跟当前误差走, 不做粘性保持 */
    s_curve_grade = (s_curve_cnt >= CURVE_CONFIRM) ? g : 0;

    /* 直角/大弯: 确认后立刻进入"转弯中"(瞬间降速在 Speed_Schedule 里做) */
    if ((s_curve_grade >= 2) && !s_turn_active && (s_turn_block == 0))
    {
        s_turn_active = 1;
        s_turn_tick   = 0;
        s_turn_done   = 0;
        s_turn_wrong  = 0;
        s_turn_dir    = (sg != 0) ? sg : s_curve_sign;
    }

    if (s_turn_active)
    {
        float lr_diff = fabsf(ADC_Lf - ADC_Rf);
        uint8_t lr_bal = 0;

        /* 左右两个电感已经基本一样 = 车头回到线上 = 转够了 */
        if ((lr_diff < TURN_DONE_BAL) &&
            (ADC_Lf > TURN_DONE_LR_MIN) && (ADC_Rf > TURN_DONE_LR_MIN))
        {
            lr_bal = 1;
        }

        if (s_turn_tick < 60000) s_turn_tick++;

        if ((a < TURN_DONE_ERR) || lr_bal)
        {
            if (s_turn_done < 250) s_turn_done++;
        }
        else
        {
            s_turn_done = 0;
        }

        /* ---- 转错保护: 误差反向够大, 且【还没】回中 ----
         * 已回中(lr_bal)的反向属于"下一个锯齿弯", 不算转错。 */
        if ((sg != 0) && (sg != s_turn_dir) && (a > TURN_WRONG_ERR) && !lr_bal)
        {
            if (s_turn_wrong < 250) s_turn_wrong++;
        }
        else
        {
            s_turn_wrong = 0;
        }

        if (s_turn_wrong >= TURN_WRONG_CNT)
        {
            /* 转错: 立刻结束并封锁一小段, PD 反向把车拉回线上 */
            s_turn_active = 0;
            s_turn_done   = 0;
            s_turn_wrong  = 0;
            s_turn_tick   = 0;
            s_turn_block  = TURN_BLOCK_TICK;
        }
        else if ((s_turn_done >= TURN_DONE_CNT) || (s_turn_tick >= TURN_MAX_TICK))
        {
            s_turn_active = 0;
            s_turn_done   = 0;
            s_turn_tick   = 0;
        }
    }
    else if (s_turn_block > 0)
    {
        s_turn_block--;
    }

    Curve_Grade_Now = s_curve_grade;
    Curve_Turn_Now  = s_turn_active;
    Curve_Conf_Now  = s_curve_cnt;
    Curve_Wrong_Now = (s_turn_block > 0) ? 1 : 0;
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
    PID_Curve_Reset();   /* 发车时清掉上一次的弯道分级/转弯中状态 */
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
    float eh_use, ev_use;

    /* 误差直接进 PD: 不延迟、不平滑、不保持 */
    eh_use = Error_H;
    ev_use = Error_V;

    /* ---- 横置(直道) PD ---- */
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

    /* ---- 竖置(弯道) PD ---- */
    d_raw_v = ev_use - Dly_Prev_V;
    Dly_Prev_V = ev_use;
    D_V_Filt_ = D_V_Filt_ + (d_raw_v - D_V_Filt_) * D_LPF;
    dv = KP_V * ev_use + KD_V * D_V_Filt_;

    /* ---- 融合: 直道全用横置, 弯道逐渐切到竖置 ---- */
    wv = ADC_V_StrengthF / V_BLEND_TH;
    if (wv > 1.0f) wv = 1.0f;

    /* 直道上降低中间两个竖置电感的比重: 只按外侧横置 L/R 沿主线走。
     * 误差一大或竖置场强起来(进弯)就不折减, 实时跟着路线变。 */
    if ((fabsf(eh_use) < STRAIGHT_V_TH) && (fabsf(ev_use) < STRAIGHT_V_TH))
    {
        wv *= STRAIGHT_V_SCALE;
    }

    Speed_Delta = (int16_t)(dh * (1.0f - wv) + dv * wv);

    /* ---- 弯道分级 + 转弯过程实时判断 ---- */
    Curve_Update(eh_use);

    /* 直道 ±3 死区: 只滤掉邻线/噪声引起的微抖,
     * 误差超过 3 就照常实时纠正, 不影响跟线。 */
    if ((s_curve_grade == 0) && !Track_Straight &&
        (fabsf(eh_use) < STRAIGHT_DEADZONE))
    {
        Speed_Delta = 0;
    }

    /* 90°/110° 转弯进行中: 按【剩余误差】实时补方向。
     * 误差归零 -> 偏置消失(转够了); 误差反向 -> 偏置清零, PD 反向拉回(转过了)。 */
    if (s_turn_active)
    {
        float bias = TURN_BIAS_K * eh_use;

        if (bias * (float)s_turn_dir < 0.0f) bias = 0.0f;
        if (bias >  TURN_BIAS_MAX) bias =  TURN_BIAS_MAX;
        if (bias < -TURN_BIAS_MAX) bias = -TURN_BIAS_MAX;
        Speed_Delta = (int16_t)((float)Speed_Delta + bias);
    }

    /* 分级限幅: 邻线/瞬态的大误差只给很小的差速, 绝不会"一闪就大范围转弯";
     * 真弯按档位放开, 110° 给足, 45° 收着, 根治"转弯幅度过大"。 */
    {
        int16_t cap;

        if (s_turn_active)
        {
            /* 一点一点转: 前 50ms 从 60% 力渐进到 100%, 之后按剩余误差补 */
            float ramp = 0.60f + 0.40f *
                         (float)((s_turn_tick < TURN_RAMP_TICK) ? s_turn_tick : TURN_RAMP_TICK) /
                         (float)TURN_RAMP_TICK;
            cap = (int16_t)((float)DELTA_CAP_110 * ramp);
        }
        else if (s_curve_grade >= 3)              cap = DELTA_CAP_110;
        else if (s_curve_grade == 2)              cap = DELTA_CAP_90;
        else if (s_curve_grade == 1)              cap = DELTA_CAP_45;
        else if (fabsf(eh_use) > CURVE_45_MIN)    cap = NEIGH_CAP;
        else                                      cap = DELTA_CAP_45;

        if (Speed_Delta >  cap) Speed_Delta =  cap;
        if (Speed_Delta < -cap) Speed_Delta = -cap;
    }
}

/* ------------------------------------------------------------
 *  丢线检测: 总场强(ADC_Strength)过低说明四路都压不到导线
 * ------------------------------------------------------------ */
void PID_LostLine_Check(void)
{
    float hsum = ADC_Lf + ADC_Rf;   /* 横置两路之和: 真正代表"还压不压得住线" */

    if (fabsf(Error) > 11.0f)
    {
        Last_Dir = (Error > 0.0f) ? 1 : -1;    /* 记住最后偏差方向 */
    }

    /* 只认横置: 锯齿/直角中竖置抬头会让 ADC_Strength 很高,
     * 用综合场强会漏判丢线; 用横置和更准, 且竖置强时不会误判。 */
    if (hsum < LOST_H_SUM_TH)
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
        Lost_State = 0;          /* 一压回线就立刻恢复, 不做保持 */
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
#define CROSS_LINE_KP    6.0f  /* 十字偏心修正加强: 进路口前把小弯留下的偏差拉正 */
#define CROSS_LINE_KD    1.5f
#define CROSS_LINE_MAX   250   /* 允许更大的对中修正(仍受 DELTA_MAX 限制) */

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

    if ((HexDia_Flag == 0) && Lost_State)
    {
        /* 丢线: 丢线时不能用传感器误差(噪声大), 沿最后方向低速找线
         * (在六角菱形环里不适用, 环内的丢线由环状态机自己处理) */
        out = (int16_t)(Last_Dir * LOST_SEARCH_DELTA);
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
 *  只做两件事: 直角/大弯瞬间降到 55%, 丢线降到 45%; 其余全程 30% 恒速。
 *  直线与 45° 小弯不减速(保持流畅), 90°/110° 一确认立刻降速,
 *  转够或判转错后立刻恢复 —— 没有任何"保持/延迟"。
 * ------------------------------------------------------------ */
int16_t PID_Speed_Schedule(int16_t basic)
{
    float ratio = 1.0f;

    if (HexDia_Flag != 0)
    {
        ratio = 1.0f;                 /* 环内由环状态机自己给速度系数 */
    }
    else if (Lost_State)
    {
        ratio = LOST_SPEED_RATIO;
    }
    else if (Track_State == TRK_ZIGZAG)
    {
        ratio = ZIG_SPEED_RATIO;      /* 连续锯齿: 略降速, 减少丢线 */
    }
    else if (Curve_Turn_Now || (Curve_Grade_Now >= 2))
    {
        ratio = CURVE_SPEED_RATIO;    /* 直角/大弯: 瞬间降速, 再一点点转 */
    }
    else if (Curve_Grade_Now == 1)
    {
        ratio = 0.90f;                /* 45° 轻弯: 略收一点, 不影响流畅 */
    }

    ratio *= Track_Speed_Ratio;       /* 赛道元素的速度系数 */

    if (ratio < 0.20f) ratio = 0.20f;
    return (int16_t)((float)basic * ratio);
}
