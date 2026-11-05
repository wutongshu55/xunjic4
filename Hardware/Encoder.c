#include "Common_headfile.h"

/* ===================== 机械参数(换车/换电机改这里) =====================
 * 减速比 20:1, 轮子外径 48mm -> 周长 = 48 x PI = 150.80mm
 */
#define WHEEL_DIAMETER_MM   48.0f
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER_MM * 3.14159265f)  /* 150.80mm */
#define GEAR_RATIO          20    /* 减速比: 电机转 20 圈, 轮子转 1 圈 */
#define ENCODER_PPR         11    /* 编码器单相每转脉冲数, 按实物改 */

/* ===================== 测速方式(最重要, 决定测不测得到) =====================
 *
 * ENCODER_MODE = 1 【定时器外部时钟计数】默认
 *     TIM2/TIM4 退化成"纯脉冲计数器": A 相来一个边沿, CNT 就 +1,
 *     CPU 只在 5ms 中断里读一下差值。不依赖正交解码硬件,
 *     【只接 A 相一根线也能测速】—— 这是"速度始终为 0"时最该用的方案。
 *
 * ENCODER_MODE = 0 【正交解码】TI12 四倍频, 需要 A/B 两相都正常,
 *     少一相(断线/虚焊/单相编码器)计数器就永远不动 -> 速度恒 0。
 */
#define ENCODER_MODE   1

/* 计数边沿(ENCODER_MODE = 1 时有效)
 *   2 = A 相上升沿 + 下降沿都数(TI1ED)【默认】一圈 440 计数, 分辨率翻倍
 *   1 = 只数上升沿(TI1FP1)             一圈 220 计数
 */
#define ENC_EDGE       2

/* 方向来源(ENCODER_MODE = 1 时有效)
 *   0 = 不看 B 相, 方向恒为正【默认】—— 循迹车不倒车, 方向本来就不重要。
 *   1 = B 相投票: A 相每个边沿进 EXTI 中断采一次 B 相。
 *
 * 【为什么默认 0】若 PA0/PB6 悬空或接触不良, 上拉+双沿会把噪声当脉冲,
 * EXTI 中断风暴会抢光 CPU, TIM1(按键扫描/电机控制)被饿死 —— 表现就是
 * "所有按键突然都没反应"。等编码器信号确认干净(EncPin 页电平稳定跳变)后
 * 再改回 1; 即使开 1 也有熔断保护(见下方 s_exti_cnt)。
 */
#define ENC_DIR_BY_B   0

/* 显示出来是负速度(A/B 两根线接反)就把这个改成 1 */
#define ENC_DIR_INVERT 0

/* 输入滤波: 0x0F 约 3.5us, 能滤掉毛刺又不影响 kHz 级脉冲。
 * 若编码器线很长、干扰大, 保持 0x0F; 若是高速编码器(>50kHz)可减到 0x04。 */
#define ENC_IC_FILTER  0x0F

/* ===================== 每转计数(自动算, 不用手改) ===================== */
#if (ENCODER_MODE == 1)
  #define ENC_MULT       (ENC_EDGE)        /* 1 = 220/转, 2 = 440/转 */
  #if (ENC_EDGE == 2)
    #define ENC_TRIG     TIM_TS_TI1F_ED    /* 双边沿 */
  #else
    #define ENC_TRIG     TIM_TS_TI1FP1     /* 单边沿(上升) */
  #endif
#else
  #define ENC_MULT       4                 /* TI12 四倍频 = 880/转 */
#endif

#define COUNTS_PER_REV   (ENCODER_PPR * ENC_MULT * GEAR_RATIO)

/* 测速窗口: 计数每 5ms 累加一次, 每 20ms 才算一次速度。
 * 窗口越长量化误差越小, 但速度环滞后越大; 20ms(50Hz)对直流电机正好。
 * 以 440 计数/转为例: 1 个计数 = 150.8/440/0.02 = 17.1 mm/s,
 *   300mm/s 时窗口内有 17 个计数 -> 量化误差约 6%, 够速度环用。 */
#define SPEED_WIN_MS     20    /* 必须是 5 的整数倍 */
#define SPEED_LPF        0.50f /* 测速一阶低通(0=不滤波, 1=不更新) */

volatile int16_t  Encoder_L_Count = 0;   /* 左编码器计数(本次5ms, 带方向) */
volatile int16_t  Encoder_R_Count = 0;   /* 右编码器计数(本次5ms, 带方向) */
volatile int32_t  Encoder_L_Total = 0;   /* 左累计(菜单 KEY3 可清零) */
volatile int32_t  Encoder_R_Total = 0;
volatile int32_t  Enc_Pulse_L     = 0;   /* 左累计(上电起, 不清零, 诊断用) */
volatile int32_t  Enc_Pulse_R     = 0;
volatile int8_t   Enc_Dir_L       = 1;   /* 当前方向 +1 / -1 */
volatile int8_t   Enc_Dir_R       = 1;
volatile int32_t  Encoder_CPR     = COUNTS_PER_REV;  /* 每转计数, 显示用 */
volatile int32_t  Encoder_Edge    = ENC_EDGE;        /* 1=只数上升沿 2=双沿 */
volatile int32_t  Encoder_Method  = ENCODER_MODE;    /* 1=外部时钟计数 0=正交解码 */
volatile int32_t  Speed_L_RPM     = 0;
volatile int32_t  Speed_R_RPM     = 0;
volatile int32_t  Speed_L_mms     = 0;   /* 左轮线速度 mm/s */
volatile int32_t  Speed_R_mms     = 0;
volatile int32_t  Speed_Avg_mms   = 0;

/* 20ms 窗口累加 */
static uint8_t  s_div   = 0;
static int32_t  s_acc_l = 0;
static int32_t  s_acc_r = 0;
static float    s_flt_l = 0.0f;
static float    s_flt_r = 0.0f;

/* 硬件计数器上一次的读数(用差值, 不再每 5ms 清零, 避免漏掉恰好在清零瞬间的脉冲) */
static uint16_t s_prev_l = 0;
static uint16_t s_prev_r = 0;

/* 方向投票(EXTI 里累加, 5ms 里结算) */
static volatile int16_t s_vote_l = 0;
static volatile int16_t s_vote_r = 0;

/* EXTI 风暴熔断: 5ms 窗口内中断超过 400 次(>80kHz)说明信号是噪声不是编码器,
 * 永久关掉 EXTI 降级为"方向恒正", 把 CPU 还给 TIM1(按键/电机控制) */
static volatile uint16_t s_exti_cnt    = 0;
volatile uint8_t         Enc_Dir_Fault = 0;

/* ------------------------------------------------------------------
 * 定时器配置: 上拉输入 + 外部时钟计数
 * ------------------------------------------------------------------ */
static void Enc_CounterInit(TIM_TypeDef *TIMx, uint32_t tim_clk,
                            GPIO_TypeDef *port, uint32_t gpio_clk,
                            uint16_t pin_a, uint16_t pin_b)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_ICInitTypeDef       ic;

    RCC_APB2PeriphClockCmd(gpio_clk, ENABLE);
    RCC_APB1PeriphClockCmd(tim_clk,  ENABLE);

    /* 上拉输入: 循迹板/霍尔编码器多为集电极开路(LM393 比较器)输出,
     * 只能拉低不能拉高, 浮空输入会一直读到 0 或乱跳 -> 计数恒 0。
     * 推挽输出的编码器用上拉输入也完全没问题。 */
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Pin   = pin_a | pin_b;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &gpio);

    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Period    = 0xFFFF;
    tb.TIM_Prescaler = 0;
    TIM_TimeBaseInit(TIMx, &tb);

    TIM_ICStructInit(&ic);
    ic.TIM_Channel     = TIM_Channel_1;
    ic.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter    = ENC_IC_FILTER;
    TIM_ICInit(TIMx, &ic);

#if (ENCODER_MODE == 1)
    /* 外部时钟模式 1: TI1(PA0/PB6) 每来一个有效边沿, CNT 加 1 */
    TIM_SelectInputTrigger(TIMx, ENC_TRIG);
    TIM_SelectSlaveMode(TIMx, TIM_SlaveMode_External1);
#else
    /* 正交解码: 需要 A/B 两相 */
    TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
#endif

    TIM_SetCounter(TIMx, 0);
    TIM_Cmd(TIMx, ENABLE);
}

#if (ENCODER_MODE == 1 && ENC_DIR_BY_B == 1)
/* ------------------------------------------------------------------
 * 方向判定: A 相边沿进外部中断, 顺手看一眼 B 相
 *   正转时 A 超前 B -> A 跳变后 A 与 B 总是【不同】
 *   反转时 B 超前 A -> A 跳变后 A 与 B 总是【相同】
 * ------------------------------------------------------------------ */
static void Enc_Vote(uint8_t side)
{
    uint8_t a, b;
    int8_t  v;

    if (side == 0)
    {
        a = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);   /* 左 A */
        b = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);   /* 左 B */
    }
    else
    {
        a = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);   /* 右 A */
        b = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);   /* 右 B */
    }

    v = (a != b) ? 1 : -1;
#if (ENC_DIR_INVERT == 1)
    v = (int8_t)(-v);
#endif

    if (side == 0)
    {
        if (s_vote_l > -50 && s_vote_l < 50) s_vote_l += v;
    }
    else
    {
        if (s_vote_r > -50 && s_vote_r < 50) s_vote_r += v;
    }
}

void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        Enc_Vote(0);
        if (s_exti_cnt < 60000) s_exti_cnt++;
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line6) != RESET)
    {
        Enc_Vote(1);
        if (s_exti_cnt < 60000) s_exti_cnt++;
        EXTI_ClearITPendingBit(EXTI_Line6);
    }
}

/* 在 Encoder_Update(5ms 周期)里调用: 检查 EXTI 频率, 超限就熔断 */
static void Enc_DirWatchdog(void)
{
    if (Enc_Dir_Fault) { s_exti_cnt = 0; return; }

    if (s_exti_cnt > 400)      /* 5ms 内 >400 次 = 平均 >80kHz, 必是噪声 */
    {
        NVIC_DisableIRQ(EXTI0_IRQn);
        NVIC_DisableIRQ(EXTI9_5_IRQn);
        Enc_Dir_Fault = 1;
        Enc_Dir_L     = 1;
        Enc_Dir_R     = 1;
    }
    s_exti_cnt = 0;
}

static void Enc_DirExtiInit(void)
{
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);  /* 左 A */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);  /* 右 A */

    /* 先定分组再填优先级, 免得被 Timer_Init 里的分组设置改掉含义 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti.EXTI_LineCmd = ENABLE;

    exti.EXTI_Line = EXTI_Line0;
    EXTI_Init(&exti);
    exti.EXTI_Line = EXTI_Line6;
    EXTI_Init(&exti);

    nvic.NVIC_IRQChannelCmd            = ENABLE;
    /* 抢占优先级必须【低于】TIM1(2): 就算编码器线是坏的、EXTI 疯狂触发,
     * TIM1 的按键扫描和电机控制也永远能插进去, 车和按键不会瘫。 */
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority    = 0;

    nvic.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_Init(&nvic);
    nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_Init(&nvic);
}
#endif

void Encoder_Init(void)
{
    /* 左编码器: PA0 = A 相(计数), PA1 = B 相(方向) */
    Enc_CounterInit(TIM2, RCC_APB1Periph_TIM2, GPIOA, RCC_APB2Periph_GPIOA,
                    GPIO_Pin_0, GPIO_Pin_1);

    /* 右编码器: PB6 = A 相(计数), PB7 = B 相(方向) */
    Enc_CounterInit(TIM4, RCC_APB1Periph_TIM4, GPIOB, RCC_APB2Periph_GPIOB,
                    GPIO_Pin_6, GPIO_Pin_7);

#if (ENCODER_MODE == 1 && ENC_DIR_BY_B == 1)
    Enc_DirExtiInit();
#endif
}

/* 读四个编码器脚的实时电平, 位图: bit0=PA0 bit1=PA1 bit2=PB6 bit3=PB7
 * 手推车时四个位都应该跳变; 哪一位不动, 就是那一根线没接/没焊/没供电。 */
uint8_t Encoder_ReadLevel(void)
{
    uint8_t lv = 0;

    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) != 0) lv |= 0x01;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) != 0) lv |= 0x02;
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) != 0) lv |= 0x04;
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) != 0) lv |= 0x08;

    return lv;
}

/* 在 TIM1 的 5ms 中断里调用, dt = 0.005
 * 每 5ms 取一次计数差值, 每 SPEED_WIN_MS(20ms) 才算一次速度。 */
void Encoder_Update(float dt)
{
    uint16_t cur;
    int16_t  dl, dr;

    cur = (uint16_t)TIM_GetCounter(TIM2);
    dl  = (int16_t)((uint16_t)(cur - s_prev_l));   /* 16 位回绕自动处理 */
    s_prev_l = cur;

    cur = (uint16_t)TIM_GetCounter(TIM4);
    dr  = (int16_t)((uint16_t)(cur - s_prev_r));
    s_prev_r = cur;

#if (ENCODER_MODE == 1 && ENC_DIR_BY_B == 1)
    Enc_DirWatchdog();   /* EXTI 频率过高就熔断, 保住按键与控制中断 */

    /* 结算方向: 20ms 内 B 相票数取符号; 没有票(只接了 A 相)就保持上次的方向 */
    if      (s_vote_l > 0) { Enc_Dir_L =  1; }
    else if (s_vote_l < 0) { Enc_Dir_L = -1; }
    s_vote_l = 0;

    if      (s_vote_r > 0) { Enc_Dir_R =  1; }
    else if (s_vote_r < 0) { Enc_Dir_R = -1; }
    s_vote_r = 0;

    dl = (int16_t)(dl * (int16_t)Enc_Dir_L);
    dr = (int16_t)(dr * (int16_t)Enc_Dir_R);
#endif

    Encoder_L_Count = dl;
    Encoder_R_Count = dr;
    Encoder_L_Total += dl;
    Encoder_R_Total += dr;
    Enc_Pulse_L     += dl;
    Enc_Pulse_R     += dr;

    s_acc_l += dl;
    s_acc_r += dr;

    s_div++;
    if (s_div < (SPEED_WIN_MS / 5)) return;
    if (dt <= 0.0001f) { s_div = 0; s_acc_l = 0; s_acc_r = 0; return; }

    {
        float dt_win = dt * (float)s_div;
        float mms_l  = ((float)s_acc_l / (float)COUNTS_PER_REV)
                       * WHEEL_CIRCUMFERENCE / dt_win;
        float mms_r  = ((float)s_acc_r / (float)COUNTS_PER_REV)
                       * WHEEL_CIRCUMFERENCE / dt_win;

        /* 一阶低通: 抑制单个窗口的量化跳变, 代价是约 20ms 滞后 */
        s_flt_l = s_flt_l + SPEED_LPF * (mms_l - s_flt_l);
        s_flt_r = s_flt_r + SPEED_LPF * (mms_r - s_flt_r);

        Speed_L_mms   = (int32_t)s_flt_l;
        Speed_R_mms   = (int32_t)s_flt_r;
        Speed_Avg_mms = (Speed_L_mms + Speed_R_mms) / 2;

        Speed_L_RPM = (int32_t)((float)Speed_L_mms / WHEEL_CIRCUMFERENCE * 60.0f);
        Speed_R_RPM = (int32_t)((float)Speed_R_mms / WHEEL_CIRCUMFERENCE * 60.0f);

        s_div   = 0;
        s_acc_l = 0;
        s_acc_r = 0;
    }
}
