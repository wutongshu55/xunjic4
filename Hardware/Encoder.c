#include "Common_headfile.h"

/* 减速比20, 轮子外径48mm
 * 周长 = 48 * PI = 150.80mm = 0.15080m
 * 编码器一圈脉冲数(假设11PPR单相) * 减速比20 = 220脉冲/圈(4倍频后880)
 * 这里用4倍频计数, 实际一圈计数 = 编码器线数 * 4 * 减速比
 * 若编码器为11PPR: 一圈 = 11*4*20 = 880
 * 速度(mm/s) = (计数差 * 周长mm) / (一圈计数 * 采样周期s)
 */

#define WHEEL_DIAMETER_MM   48.0f
#define WHEEL_CIRCUMFERENCE  (WHEEL_DIAMETER_MM * 3.14159265f)  /* 约150.80mm */
#define GEAR_RATIO           20   /* 减速比 */
#define ENCODER_PPR          11   /* 编码器单相每转脉冲数, 按实际修改 */
#define COUNTS_PER_REV       (ENCODER_PPR * 4 * GEAR_RATIO)  /* 4倍频, 一圈计数 */

volatile int16_t  Encoder_L_Count = 0;   /* 左编码器累计计数(本采样周期) */
volatile int16_t  Encoder_R_Count = 0;   /* 右编码器累计计数(本采样周期) */
volatile int32_t  Encoder_L_Total = 0;  /* 左编码器总累计(用于里程) */
volatile int32_t  Encoder_R_Total = 0;  /* 右编码器总累计 */
volatile int32_t  Speed_L_RPM = 0;      /* 左轮转速 RPM */
volatile int32_t  Speed_R_RPM = 0;      /* 右轮转速 RPM */
volatile int32_t  Speed_L_mms = 0;      /* 左轮速度 mm/s */
volatile int32_t  Speed_R_mms = 0;      /* 右轮速度 mm/s */
volatile int32_t  Speed_Avg_mms = 0;   /* 平均速度 mm/s */

void Encoder_Init(void)
{
    /* 左编码器: PA0(TIM2_CH1) / PA1(TIM2_CH2) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
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
    TIM_ICInitStructure.TIM_ICFilter = 0x0F;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);

    /* 右编码器: PB6(TIM4_CH1) / PB7(TIM4_CH2) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_Period    = 0xFFFF;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 0x0F;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}

/* 在定时中断中调用, dt为采样周期(秒) */
void Encoder_Update(float dt)
{
    int16_t cnt_l, cnt_r;

    cnt_l = (int16_t)TIM_GetCounter(TIM2);
    cnt_r = (int16_t)TIM_GetCounter(TIM4);

    TIM_SetCounter(TIM2, 0);
    TIM_SetCounter(TIM4, 0);

    Encoder_L_Count  = cnt_l;
    Encoder_R_Count  = cnt_r;
    Encoder_L_Total += cnt_l;
    Encoder_R_Total += cnt_r;

    if (dt > 0.001f)
    {
        /* RPM = (计数差 / 一圈计数) * (60 / 采样周期s) */
        Speed_L_RPM = (int32_t)((float)cnt_l / (float)COUNTS_PER_REV / dt * 60.0f);
        Speed_R_RPM = (int32_t)((float)cnt_r / (float)COUNTS_PER_REV / dt * 60.0f);
        /* mm/s = (计数差 / 一圈计数) * 周长mm / 采样周期s */
        Speed_L_mms = (int32_t)((float)cnt_l / (float)COUNTS_PER_REV * WHEEL_CIRCUMFERENCE / dt);
        Speed_R_mms = (int32_t)((float)cnt_r / (float)COUNTS_PER_REV * WHEEL_CIRCUMFERENCE / dt);
        Speed_Avg_mms = (Speed_L_mms + Speed_R_mms) / 2;
    }
}
