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

volatile int16_t Speed_Basic = 300;      /* mm/s */
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

void Motor_Out(int16_t L_Speed, int16_t R_Speed)
{
    if (L_Speed > 0)
    {
        if (L_Speed > PWM_DUTY_MAX) L_Speed = PWM_DUTY_MAX;
        GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_SET);
        PWMA_SetDuty(L_Speed);
    }
    else
    {
        if (L_Speed < -PWM_DUTY_MAX) L_Speed = -PWM_DUTY_MAX;
        GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET);
        PWMA_SetDuty(-L_Speed);
    }

    if (R_Speed > 0)
    {
        if (R_Speed > PWM_DUTY_MAX) R_Speed = PWM_DUTY_MAX;
        GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_SET);
        PWMB_SetDuty(R_Speed);
    }
    else
    {
        if (R_Speed < -PWM_DUTY_MAX) R_Speed = -PWM_DUTY_MAX;
        GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_SET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET);
        PWMB_SetDuty(-R_Speed);
    }
}

/* 清积分: 停车/重新发车/切换闭环模式时调用, 避免带着旧积分冲出去 */
void Speed_Loop_Reset(void)
{
    s_int_l = 0.0f;
    s_int_r = 0.0f;
    s_div   = 0;
    Duty_L  = 0;
    Duty_R  = 0;
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
    float k     = (float)Speed_Basic / 5000.0f;   /* 差速量纲换算 */
    float delta = (float)Speed_Delta * k;
    float tl    = (float)Speed_Basic + delta;
    float tr    = (float)Speed_Basic - delta;
    float dl, dr;
    float al, ar;
    float cap_l, cap_r;

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
        Beep_On(300);
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
            Beep_On(400);
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