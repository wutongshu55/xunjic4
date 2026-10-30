#include "Common_headfile.h"

/* 减速比20, 轮子外径48mm
 * 周长 = 48 * PI = 150.80mm
 * 编码器一圈计数(4倍频后) = 线数 * 4 * 减速比 = 11*4*20 = 880
 *
 * 测速窗口说明(重要):
 *   若每 5ms 算一次速度, 1 个计数 = 150.8/880/0.005 = 34.3 mm/s,
 *   300mm/s 时只有 9 个计数, 量化误差 >10%, 速度环会抖。
 *   所以这里把计数累加到 20ms 再算: 1 个计数 = 8.6 mm/s,
 *   300mm/s 时有 35 个计数, 量化误差约 3%, 够用了。
 *   20ms 对应 50Hz 速度环带宽, 对直流电机(机械时间常数几十 ms)足够。
 */

#define WHEEL_DIAMETER_MM   48.0f
#define WHEEL_CIRCUMFERENCE  (WHEEL_DIAMETER_MM * 3.14159265f)  /* 约150.80mm */
#define GEAR_RATIO           20   /* 减速比 */
#define ENCODER_PPR          11   /* 编码器单相每转脉冲数, 按实际修改 */

/* =====================================================================
 *  测速方式选择(最重要, 决定能不能测到速度):
 *
 *  ENCODER_MODE = 1  【单相计数】只用 TI1 外部时钟, 一根信号线就测速
 *     适用: 编码器只有 A 相输出(很多霍尔编码器只有单相),
 *           或 A/B 里有一根没接/断线。
 *     每转计数 = PPR * GR (无 4 倍频)
 *     缺点: 不分辨方向(后退也计为正), 对循迹不倒车没有影响。
 *
 *  ENCODER_MODE = 0  【正交解码】TI12 四倍频, 需要 A/B 两相都接
 *     每转计数 = PPR * 4 * GR
 *     优点: 能分辨方向; 缺点: 少一相就永远测不到。
 * ===================================================================== */
#define ENCODER_MODE         1

#if (ENCODER_MODE == 1)
  #define ENC_MULT          1
  #define ENC_TIM_MODE      TIM_SlaveMode_External1
#else
  #define ENC_MULT          4
  #define ENC_TIM_MODE      TIM_EncoderMode_TI12
#endif

#define COUNTS_PER_REV       (ENCODER_PPR * ENC_MULT * GEAR_RATIO)

#define SPEED_WIN_MS         20   /* 测速窗口(ms), 必须是 5ms 的整数倍 */
#define SPEED_LPF            0.50f /* 测速一阶低通系数(0=不滤波, 1=不更新) */

volatile int16_t  Encoder_L_Count = 0;   /* 左编码器计数(本次5ms) */
volatile int16_t  Encoder_R_Count = 0;   /* 右编码器计数(本次5ms) */
volatile int32_t  Encoder_L_Total = 0;  /* 左编码器总累计(用于里程) */
volatile int32_t  Encoder_R_Total = 0;  /* 右编码器总累计 */
volatile int32_t  Speed_L_RPM = 0;      /* 左轮转速 RPM */
volatile int32_t  Speed_R_RPM = 0;      /* 右轮转速 RPM */
volatile int32_t  Speed_L_mms = 0;      /* 左轮速度 mm/s */
volatile int32_t  Speed_R_mms = 0;      /* 右轮速度 mm/s */
volatile int32_t  Speed_Avg_mms = 0;   /* 平均速度 mm/s */

/* 20ms 窗口累加 */
static uint8_t s_div   = 0;
static int32_t s_acc_l = 0;
static int32_t s_acc_r = 0;
static float   s_flt_l = 0.0f;
static float   s_flt_r = 0.0f;

void Encoder_Init(void)
{
    /* 左编码器: PA0(TIM2_CH1) —— 单相计数只用 CH1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    /* 上拉输入: 很多循迹板/编码器的输出是集电极开路(LM393 是比较器),
     * 浮空输入没有上拉电阻就一直是 0 或乱跳 -> 计数恒 0。
     * 推挽输出的编码器用上拉输入也完全没问题。 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_Period        = 0xFFFF;
    TIM_TimeBaseInitStructure.TIM_Prescaler     = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter    = 0x0F;

#if (ENCODER_MODE == 1)
    /* 单相计数: 每个 TI1 上升沿 CNT++ (外部时钟模式 1) */
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM_SelectInputTrigger(TIM2, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_External1);
    TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
#else
    /* 正交解码: A/B 两相 */
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
#endif
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);

    /* 右编码器: PB6(TIM4_CH1) —— 单相计数只用 CH1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_Period    = 0xFFFF;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter    = 0x0F;

#if (ENCODER_MODE == 1)
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_External1);
    TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
#else
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
#endif
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}

/* 在 TIM1 的 5ms 中断中调用, dt = 调用周期(秒) = 0.005
 * 每 SPEED_WIN_MS 才真正算一次速度, 中间只累加计数。 */
void Encoder_Update(float dt)
{
    int16_t cnt_l, cnt_r;

    cnt_l = (int16_t)TIM_GetCounter(TIM2);
    cnt_r = (int16_t)TIM_GetCounter(TIM4);

    TIM_SetCounter(TIM2, 0);
    TIM_SetCounter(TIM4, 0);

    Encoder_L_Count = cnt_l;
    Encoder_R_Count = cnt_r;
    Encoder_L_Total += cnt_l;
    Encoder_R_Total += cnt_r;

    s_acc_l += cnt_l;
    s_acc_r += cnt_r;

    s_div++;
    if (s_div < (SPEED_WIN_MS / 5)) return;
    if (dt <= 0.0001f) { s_div = 0; s_acc_l = 0; s_acc_r = 0; return; }

    {
        float dt_win = dt * (float)s_div;
        float mms_l  = ((float)s_acc_l / (float)COUNTS_PER_REV)
                       * WHEEL_CIRCUMFERENCE / dt_win;
        float mms_r  = ((float)s_acc_r / (float)COUNTS_PER_REV)
                       * WHEEL_CIRCUMFERENCE / dt_win;

        /* 一阶低通: 抑制单个窗口的量化跳变, 但会引入约 20ms 滞后 */
        s_flt_l = s_flt_l + SPEED_LPF * (mms_l - s_flt_l);
        s_flt_r = s_flt_r + SPEED_LPF * (mms_r - s_flt_r);

        Speed_L_mms = (int32_t)s_flt_l;
        Speed_R_mms = (int32_t)s_flt_r;
        Speed_Avg_mms = (Speed_L_mms + Speed_R_mms) / 2;

        Speed_L_RPM = (int32_t)((float)Speed_L_mms / WHEEL_CIRCUMFERENCE * 60.0f);
        Speed_R_RPM = (int32_t)((float)Speed_R_mms / WHEEL_CIRCUMFERENCE * 60.0f);

        s_div   = 0;
        s_acc_l = 0;
        s_acc_r = 0;
    }
}
