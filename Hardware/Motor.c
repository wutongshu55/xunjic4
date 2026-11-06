#include "Common_headfile.h"

/* ------------------------------------------------------------
 *  电机输出 + 双轮速度闭环
 *
 *  单位约定(重要, 别混):
 *    Speed_Basic      : mm/s          —— 目标基准速度(菜单三档)
 *    Speed_Delta      : "PWM@5000" 等价单位 —— 循迹 PD 输出的差速量
 *    Speed_Target_L/R : mm/s          —— 速度环的左右轮目标
 *    Duty_L/R         : 0~10000 占空比 —— 真正给 PWM 的值
 *
 *  为什么 Speed_Delta 不直接改成 mm/s:
 *    它来自 PID_Apply(), kp/kd 的门限都是按这套量纲调过的。
 *    在这里一次性换算, 好处是【换速度档位时转向比例不变】:
 *      delta_mms = Speed_Delta * (Speed_Basic / 5000)
 *    原开环是 L=Basic+Delta, 差速比 = Delta/Basic, 低速档转向会过冲;
 *    改成按 5000 归一后, 三档的差速比例一致。
 *
 *  速度环结构: 前馈 + PI
 *    duty = FF * 目标速度 + Kp*误差 + 积分项
 *    前馈让它在目标变化时立刻给出大致占空比, PI 只负责修偏,
 *    所以 Kp/Ki 可以取得比较小, 不容易抖。
 * ------------------------------------------------------------ */

#define SPEED_LOOP_DIV    4        /* 每 4 次(20ms)跑一次速度环, 与测速窗口对齐 */
#define SPEED_LOOP_DT     0.02f
#define SPEED_FF_GAIN     12.5f    /* 前馈: 每 mm/s 需要的占空比 ≈ 10000/800mm/s */
#define SPEED_DUTY_DEAD   1200     /* 电机启动死区(占空比), 低于此值电机不转 */
#define SPEED_I_LIMIT     3500.0f  /* 积分限幅(占空比) */

/* 差速随降速收缩的比例(0~1)。0 = 差速不随降速变化(旧行为, 弯道会原地急转),
 * 1 = 完全等比缩放(转向半径恒定, 但内侧轮几乎不转)。
 * 0.45 = 折中: 弯道降到 45% 时, 差速收成原来的 0.75。 */
#define DELTA_SPEED_KEEP  0.45f

/* 急弯/直角: 内轮占空比随【当前误差】实时变化, 不按固定时间/角度。
 *   误差 >= SHARP_INNER_REF -> 内轮降到 SHARP_INNER_DUTY(接近停, 半径最小)
 *   误差收小 -> 内轮自动补速, 车头平滑摆正
 * SHARP_INNER_STEP 是斜坡, 防止瞬间堵转。 */
#define SHARP_INNER_DUTY  150
#define SHARP_INNER_STEP  250.0f
#define SHARP_INNER_REF   56.0f    /* 误差达到此值即"满舵"(提高后 60° 弯不会一上来就打死) */
#define SHARP_INNER_K     0.93f    /* 满舵时内轮只保留 7% 基准 */

/* ---- 直道等速补偿(编码器) ----
 * 直行时两轮目标占空比完全相同, 但电机特性/阻力/轮胎直径不完全一致,
 * 车会慢慢往一侧偏。用实测速度差做慢积分: 跑得快的轮子压一点、慢的补一点,
 * 只调整左右分配, 总占空比不变(车速不变)。
 * 只在"直道 + 不在任何元素里"时积分, 免得弯道/环把补偿带偏。 */
#define STRAIGHT_EQ_EN   1        /* 1=开 0=关(做对比用) */
#define EQ_SIGN          1        /* 补偿方向反了(越补越偏)就改 -1 */
#define EQ_KP            0.8f     /* (mm/s 速度差) -> 占空比 */
#define EQ_KI            3.0f     /* 积分速度 */
#define EQ_LIMIT       250.0f     /* 最大补偿量(占空比), 不许大到大角度转圈 */
#define EQ_ERR_MAX      80.0f     /* 速度差超过此值视为异常, 不积分 */
#define EQ_MIN_SPEED    30.0f     /* 平均速度太低不积分(量化噪声大) */
#define EQ_STRAIGHT_TH  12.0f     /* 横/竖误差都要小于此值才算直道 */
#define EQ_LEAK          0.995f   /* 积分泄漏, 防止把错误偏置固化成转圈 */

volatile int16_t Speed_Basic = 350;      /* mm/s */
volatile int16_t Speed_Target_L = 0;     /* mm/s */
volatile int16_t Speed_Target_R = 0;     /* mm/s */
volatile int16_t Duty_L = 0;             /* 实际输出占空比 */
volatile int16_t Duty_R = 0;

volatile float   Speed_KpS = 12.0f;      /* 速度环比例(占空比 / (mm/s)) */
volatile float   Speed_KiS = 60.0f;      /* 速度环积分(占空比 / (mm/s·s)) */
volatile uint8_t Speed_Loop_En = 1;      /* 0=开环(仅前馈) 1=闭环 */
volatile uint8_t Speed_Fault   = 0;      /* 0=正常 1=测速失效(已降级开环) 2=编码器反接(已自动纠正) */

static float   s_int_l = 0.0f;
static float   s_int_r = 0.0f;
static float   s_eq_int = 0.0f;    /* 直道等速补偿的积分 */
static uint8_t s_div   = 0;

/* 编码器极性自动纠正: 车轮向前, 计数却一直为负 -> 说明 A/B 相接反了。
 * 与其让 PI 把"负速度"当成"跑太慢"一路加到满占空比, 不如直接翻符号。 */
static int8_t   s_meas_sign = 1;
static uint16_t s_rev_ms    = 0;   /* 测速持续为负的时长(ms) */
static uint16_t s_low_ms    = 0;   /* 满占空比仍测不到速度的时长(ms) */
static uint8_t  s_ol_fb     = 0;   /* 1=测速失效, 已强制降级为开环 */
static uint16_t s_grace_ms  = 0;   /* 极性纠正后的观察期, 期间不判失效 */

void Motor_Init(void)
{
    PWM_Init(17000);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
}

/* 左右两个轮子的方向极性整体取反:
 * 1 = 反转(当前: 电感改低/车头方向调整后的接线)
 * 0 = 恢复原来的方向
 * 只改这一个宏就能来回切换, 不用动下面的 GPIO 代码。 */
#define MOTOR_DIR_INVERT 1

#if MOTOR_DIR_INVERT
#define DIR_POS_1  Bit_SET      /* 正转: IN1 */
#define DIR_POS_2  Bit_RESET    /* 正转: IN2 */
#define DIR_NEG_1  Bit_RESET    /* 反转: IN1 */
#define DIR_NEG_2  Bit_SET      /* 反转: IN2 */
#else
#define DIR_POS_1  Bit_RESET
#define DIR_POS_2  Bit_SET
#define DIR_NEG_1  Bit_SET
#define DIR_NEG_2  Bit_RESET
#endif

void Motor_Out(int16_t L_Speed, int16_t R_Speed)
{
    if (L_Speed > 0)
    {
        if (L_Speed > PWM_DUTY_MAX) L_Speed = PWM_DUTY_MAX;
        GPIO_WriteBit(GPIOB, GPIO_Pin_12, DIR_POS_1);   /* 左轮 PB12/PB13 */
        GPIO_WriteBit(GPIOB, GPIO_Pin_13, DIR_POS_2);
        PWMA_SetDuty(L_Speed);
    }
    else
    {
        if (L_Speed < -PWM_DUTY_MAX) L_Speed = -PWM_DUTY_MAX;
        GPIO_WriteBit(GPIOB, GPIO_Pin_12, DIR_NEG_1);
        GPIO_WriteBit(GPIOB, GPIO_Pin_13, DIR_NEG_2);
        PWMA_SetDuty(-L_Speed);
    }

    if (R_Speed > 0)
    {
        if (R_Speed > PWM_DUTY_MAX) R_Speed = PWM_DUTY_MAX;
        GPIO_WriteBit(GPIOB, GPIO_Pin_14, DIR_POS_1);   /* 右轮 PB14/PB15 */
        GPIO_WriteBit(GPIOB, GPIO_Pin_15, DIR_POS_2);
        PWMB_SetDuty(R_Speed);
    }
    else
    {
        if (R_Speed < -PWM_DUTY_MAX) R_Speed = -PWM_DUTY_MAX;
        GPIO_WriteBit(GPIOB, GPIO_Pin_14, DIR_NEG_1);
        GPIO_WriteBit(GPIOB, GPIO_Pin_15, DIR_NEG_2);
        PWMB_SetDuty(-R_Speed);
    }
}

/* 清积分: 停车/重新发车/切换闭环模式时调用, 避免带着旧积分冲出去 */
void Speed_Loop_Reset(void)
{
    s_int_l  = 0.0f;
    s_int_r  = 0.0f;
    s_eq_int = 0.0f;      /* 等速补偿一起清, 重新发车从零学 */
    s_div    = 0;
    Duty_L   = 0;
    Duty_R   = 0;
}

/* 发车时调用: 清故障标志, 重新开始检测(编码器极性纠正结果保留) */
void Speed_Fault_Reset(void)
{
    Speed_Fault = 0;
    s_ol_fb     = 0;
    s_rev_ms    = 0;
    s_low_ms    = 0;
    s_grace_ms  = 0;
}

/* 死区补偿: 目标不为 0 但占空比太小, 电机根本不转 -> 补到启动死区 */
static float Speed_DeadComp(float duty, float target)
{
    if (target > 1.0f)
    {
        if (duty < 0.0f)             duty = 0.0f;          /* 目标向前, 不许倒转 */
        if (duty < (float)SPEED_DUTY_DEAD) duty = (float)SPEED_DUTY_DEAD;
    }
    else if (target < -1.0f)
    {
        if (duty > 0.0f)             duty = 0.0f;
        if (duty > -(float)SPEED_DUTY_DEAD) duty = -(float)SPEED_DUTY_DEAD;
    }
    else
    {
        duty = 0.0f;                                          /* 目标为 0 直接停 */
    }
    return duty;
}

/* 单轮 PI: 前馈 + 比例 + 积分, 带积分限幅与饱和停止积分 */
static float Speed_PI(float target, float actual, float *integ)
{
    float err = target - actual;
    float duty;
    float p_term;

    if (Speed_Loop_En == 0 || s_ol_fb)
    {
        /* 开环: 只用前馈。
         * 手动切到开环 = FF 标定/A-B 对比;
         * s_ol_fb = 测速失效时的安全降级, 不让车因为"测不到速度"而跑满。 */
        *integ = 0.0f;
        return target * SPEED_FF_GAIN;
    }

    p_term = Speed_KpS * err;
    duty   = target * SPEED_FF_GAIN + p_term + *integ;

    /* 抗积分饱和: 输出已顶限且误差还在往同方向推, 就不再积分 */
    if (!((duty >= (float)PWM_DUTY_MAX && err > 0.0f) ||
          (duty <= -(float)PWM_DUTY_MAX && err < 0.0f)))
    {
        *integ += Speed_KiS * err * SPEED_LOOP_DT;
        if (*integ >  SPEED_I_LIMIT) *integ =  SPEED_I_LIMIT;
        if (*integ < -SPEED_I_LIMIT) *integ = -SPEED_I_LIMIT;
    }

    return target * SPEED_FF_GAIN + p_term + *integ;
}

/* 每 5ms 调用: 更新目标, 每 20ms 跑一次速度环
 *
 * 三层保护(为什么要有):
 *   1. 编码器 A/B 反接 -> 测速为负 -> PI 以为"越跑越慢" -> 加到满. 自动翻符号。
 *   2. 编码器没信号   -> 测速恒 0  -> PI 积分顶满   -> 加到满. 降级为开环。
 *   3. 输出上限 = 目标前馈的 1.6 倍 -> 任何单点故障都不可能变成满占空比。
 * 没有这三层时, 症状就是【开低档也全速转】。
 */
void CarMotor_Control(void)
{
    float dl, dr;
    float k, delta, tl, tr, al, ar, cap_l, cap_r;

    /* ---- 方式二: 纯 PWM 开环(车速固定 35%, 现在是唯一入口) ----
     * 基准占空比固定 PWM_BASE_DUTY(35%), 循迹差速按原量纲直接叠加,
     * 完全不进速度环。弯道降速仍然生效: 按调度后的比例缩放基准占空比
     * (Speed_Basic 已被 PID_Speed_Schedule 按 0.20~1.0 缩放过)。 */
    if (Menu_SpdMode == SPD_MODE_PWM)
    {
        float base  = (float)PWM_BASE_DUTY;
        float ratio = 1.0f;
        float d;

        if (SPEED_NOMINAL_MMS > 0)
        {
            ratio = (float)Speed_Basic / (float)SPEED_NOMINAL_MMS;
            if (ratio > 1.0f) ratio = 1.0f;
            if (ratio < 0.2f) ratio = 0.2f;
            base *= ratio;
        }

        /* 差速跟着基准一起收(见 DELTA_SPEED_KEEP 说明):
         * 旧代码这里是 d = Speed_Delta 不缩放。弯道降速到 45% 时,
         * 基准 1575 配上 900 的差速 -> 内轮 675、外轮 2475,
         * 轮速比接近 4:1, 车在小半径里"原地急转"再甩出去。
         * 这就是现场"转弯角度太大 / 十字出口甩尾"的主因。 */
        d = (float)Speed_Delta * (1.0f - DELTA_SPEED_KEEP * (1.0f - ratio));

#if STRAIGHT_EQ_EN
        /* 直道等速: 用编码器把两轮实际速度拉平(只调分配, 不改车速)。
         * 【判直道必须用横/竖各自误差】合成 Error 可能被横竖抵消成 0,
         * 那时车其实还在弯里, 用 Error 判断会把补偿积分带偏并固化成恒定差速 -> 转圈。 */
        if ((fabsf(Error_H) < EQ_STRAIGHT_TH) && (fabsf(Error_V) < EQ_STRAIGHT_TH) &&
            !Sharp_State && !Track_Straight && (HexDia_Flag == 0) && !Lost_State)
        {
            float vr = (float)Speed_R_mms;
            float vl = (float)Speed_L_mms;
            float ev = (vr - vl) * (float)EQ_SIGN;      /* 右快为正 */

            s_eq_int *= EQ_LEAK;                        /* 泄漏: 不给错误偏置留记忆 */

            if ((fabsf(ev) < EQ_ERR_MAX) &&
                (fabsf(vr) + fabsf(vl) > 2.0f * EQ_MIN_SPEED))
            {
                /* ev 与积分反向 = 补过头或 EQ_SIGN 反了 -> 快速释放, 不许越补越偏 */
                if (ev * s_eq_int < 0.0f) s_eq_int *= 0.90f;
                s_eq_int += EQ_KI * ev * 0.005f;
                if (s_eq_int >  EQ_LIMIT) s_eq_int =  EQ_LIMIT;
                if (s_eq_int < -EQ_LIMIT) s_eq_int = -EQ_LIMIT;
            }
            else
            {
                s_eq_int *= 0.98f;                      /* 数据不可信时缓慢释放 */
            }

            d += EQ_KP * ev + s_eq_int;
            if (d >  EQ_LIMIT) d =  EQ_LIMIT;
            if (d < -EQ_LIMIT) d = -EQ_LIMIT;
        }
        else
        {
            s_eq_int *= 0.98f;                          /* 离开直道就把偏置放掉 */
        }
#endif

        dl = base + d;
        dr = base - d;

        /* ---- 急弯/直角: 内轮按当前误差实时缩放 ----
         * 误差大(还在弯里) -> 内轮接近停转, 绕内轮转, 半径最小;
         * 误差收小(快回中) -> 内轮自动补速, 车头平滑摆正。
         * 不按固定时间、不按固定角度, 全靠电感反馈。 */
        if (Sharp_State && (HexDia_Flag == 0) && !Track_Straight)
        {
            static float s_inner = 0.0f;
            /* 用【合成误差】, 与运行页 E 一致: E=0 就不该再压内轮。
             * 旧实现用 max(|E_H|,|E_V|), 横竖抵消成 E=0 时仍会满舵 -> 原地转圈。 */
            float e = fabsf(Error);
            float k = e / SHARP_INNER_REF;
            float want;

            if (k > 1.0f) k = 1.0f;
            if (s_inner <= 0.0f) s_inner = base;      /* 首次进入从基准起步 */

            want = base * (1.0f - SHARP_INNER_K * k);
            if (want < (float)SHARP_INNER_DUTY) want = (float)SHARP_INNER_DUTY;

            /* 斜坡: 每 5ms 最多变化 STEP, 防止瞬间堵转/打滑 */
            if (want < s_inner)
            {
                s_inner -= SHARP_INNER_STEP;
                if (s_inner < want) s_inner = want;
            }
            else
            {
                s_inner += SHARP_INNER_STEP;
                if (s_inner > want) s_inner = want;
            }

            if (Sharp_Dir > 0) { dl = base; dr = s_inner; }  /* 左轮走、右轮停 */
            else               { dl = s_inner; dr = base; }  /* 右轮走、左轮停 */
        }

        if (dl > (float)PWM_DUTY_MAX) dl = (float)PWM_DUTY_MAX;
        if (dr > (float)PWM_DUTY_MAX) dr = (float)PWM_DUTY_MAX;
        if (dl < 0.0f) dl = 0.0f;                 /* 循迹阶段不倒车 */
        if (dr < 0.0f) dr = 0.0f;

        Duty_L = (int16_t)dl;
        Duty_R = (int16_t)dr;
        /* 运行页 T 仍显示"等效 mm/s", 方便和实测 A 对照 */
        Speed_Target_L = (int16_t)(dl / SPEED_FF_GAIN);
        Speed_Target_R = (int16_t)(dr / SPEED_FF_GAIN);

        s_int_l = 0.0f;
        s_int_r = 0.0f;
        Motor_Out(Duty_L, Duty_R);
        return;
    }

    /* ---- 方式一: 编码器闭环 / 前馈开环(目标 mm/s) ---- */
    k     = (float)Speed_Basic / 5000.0f;   /* 差速量纲换算 */
    delta = (float)Speed_Delta * k;
    tl    = (float)Speed_Basic + delta;
    tr    = (float)Speed_Basic - delta;
    al    = 0.0f;
    ar    = 0.0f;

    if (tl < 0.0f) tl = 0.0f;                     /* 循迹阶段不倒车 */
    if (tr < 0.0f) tr = 0.0f;

    Speed_Target_L = (int16_t)tl;
    Speed_Target_R = (int16_t)tr;

    /* 实测速度(经极性纠正) */
    al = (float)Speed_L_mms * (float)s_meas_sign;
    ar = (float)Speed_R_mms * (float)s_meas_sign;

    /* 输出上限: 目标速度对应前馈的 1.6 倍 + 死区, 留够带载余量但不许失控 */
    cap_l = tl * SPEED_FF_GAIN * 1.6f + (float)SPEED_DUTY_DEAD;
    cap_r = tr * SPEED_FF_GAIN * 1.6f + (float)SPEED_DUTY_DEAD;

    /* ---- 纠错 1: 编码器 A/B 反接 ----
     * 目标向前, 两轮测量却持续为负 -> 翻符号。 */
    if (tl > 50.0f && al < -20.0f && ar < -20.0f)
    {
        if (s_rev_ms < 60000) s_rev_ms += 5;
    }
    else
    {
        s_rev_ms = 0;
    }
    if (s_rev_ms >= 200)
    {
        s_meas_sign = (int8_t)(-s_meas_sign);
        s_rev_ms    = 0;
        s_int_l     = 0.0f;
        s_int_r     = 0.0f;
        s_low_ms    = 0;            /* 纠正后重新给 800ms 机会, 别马上判成失效 */
        s_grace_ms  = 800;
        Speed_Fault = 2;
    }

    if (s_grace_ms > 0 && s_grace_ms >= 5) s_grace_ms -= 5;

    s_div++;
    if (s_div < SPEED_LOOP_DIV) return;
    s_div = 0;

    /* ---- 纠错 2: 测不到速度 ----
     * 已经顶到输出上限(说明给了 1.6 倍前馈仍不够), 实测却不到目标 30%:
     * 编码器没接/没信号/断线 -> 降级为开环, 车按前馈估算跑, 不会失控。 */
    if (!s_ol_fb && s_grace_ms == 0 && tl > 50.0f)
    {
        if (Duty_L >= (int16_t)(cap_l - 60.0f) && al < tl * 0.30f)
        {
            if (s_low_ms < 60000) s_low_ms += 20;
        }
        else
        {
            s_low_ms = 0;
        }
        if (s_low_ms >= 400)
        {
            s_ol_fb     = 1;
            Speed_Fault = 1;
            s_int_l     = 0.0f;
            s_int_r     = 0.0f;
        }
    }

    dl = Speed_PI(tl, al, &s_int_l);
    dr = Speed_PI(tr, ar, &s_int_r);

    /* ---- 保护 3: 输出上限 ---- */
    if (dl > cap_l) dl = cap_l;
    if (dr > cap_r) dr = cap_r;

    dl = Speed_DeadComp(dl, tl);
    dr = Speed_DeadComp(dr, tr);

    if (dl >  (float)PWM_DUTY_MAX) dl =  (float)PWM_DUTY_MAX;
    if (dl < -(float)PWM_DUTY_MAX) dl = -(float)PWM_DUTY_MAX;
    if (dr >  (float)PWM_DUTY_MAX) dr =  (float)PWM_DUTY_MAX;
    if (dr < -(float)PWM_DUTY_MAX) dr = -(float)PWM_DUTY_MAX;

    Duty_L = (int16_t)dl;
    Duty_R = (int16_t)dr;

    Motor_Out(Duty_L, Duty_R);
}