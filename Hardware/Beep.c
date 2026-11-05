#include "Common_headfile.h"

/* ============================================================
 *  无源蜂鸣器驱动 (BEEP = PA8, 依原理图)
 *
 *  【为什么不能直接拉高/拉低】
 *  无源蜂鸣器内部没有振荡源, 必须给它 2~5kHz 的方波才会响;
 *  普通电平只会"嗒"一声甚至无声。
 *
 *  【方波从哪来】
 *  PA8 硬件上只属于 TIM1_CH1, 而 TIM1 时基被 5ms 控制中断占死,
 *  不能改成音频频率 -> 只能软件翻转。
 *  方案: 借用 TIM3(电机 PWM, 17000Hz) 的更新中断翻转 PA8,
 *  只有蜂鸣期间才打开 TIM3 更新中断, 空闲时零开销。
 *
 *  音色(TIM3 更新 ≈ 17kHz):
 *    每 1 次更新翻转 -> 8.5kHz 方波(高于谐振区, 偏轻)
 *    每 2 次         -> 4.25kHz  平时用, 接近谐振, 最响
 *    每 4 次         -> 2.13kHz  低音, 长鸣(故障告警)专用, 一听就不同
 *
 *  中断优先级: 抢占 3, 低于 TIM1(2, 控制环), 与编码器 EXTI 同级,
 *  蜂鸣绝不影响 5ms 控制周期。
 * ============================================================ */

volatile uint16_t Beep_Timer = 0;   /* 蜂鸣剩余时间(ms), 由 Beep_Tick(5ms) 递减 */
volatile uint8_t Beep_Mode = 1;     /* 本车是【有源】蜂鸣器: 1=高电平驱动(默认)
                                     * 若上电就一直响 / 全程没声, 在菜单第 6 页
                                     * 把 Beep 切成 2(低电平) 或 0(无源方波) */

static uint8_t s_div = 2;           /* 每 s_div 次 TIM3 更新翻转一次引脚 */
static uint8_t s_cnt = 0;           /* 翻转分频计数 */
static uint8_t s_pin = 0;           /* 当前引脚电平镜像 */

void Beep_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOA, GPIO_Pin_8);

    /* 按当前模式输出"静默"电平: 低电平触发型必须先拉高,
     * 否则上电就长响不停(带三极管的蜂鸣器模块最常见) */
    if (Beep_Mode == 2) GPIO_SetBits(GPIOA, GPIO_Pin_8);

    /* TIM3 的 NVIC 通道在这里配好, 中断源由 Beep_On/Off 动态开关 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;   /* 低于 TIM1(2) */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_InitStructure);
}

/* 开启蜂鸣器持续 time_ms 毫秒
 * 三种驱动模式见 Beep.h —— 现场"不响"最常见的原因不是代码,
 * 而是蜂鸣器类型/极性不对(有源当无源驱、低电平触发当高电平使能)。
 * 菜单第 6 页 Beep 条目切换模式, 每种切换后立即试鸣 300ms。 */
void Beep_On(uint16_t time_ms)
{
    if (time_ms == 0)
    {
        Beep_Off();
        return;
    }

    s_cnt      = 0;
    s_pin      = 0;
    Beep_Timer = time_ms;

    if (Beep_Mode == 0)
    {
        /* 无源: 2.8kHz 方波(17kHz / 3 / 2)。原来的 4.25kHz 对很多蜂鸣器
         * 已高出谐振点, 听起来只有很轻的"嘶"——这也是"不响"的常见原因。
         * 长鸣(≥300ms)降到 2.1kHz, 更低沉, 一听就分得清。 */
        s_div = (time_ms >= 300) ? 4 : 3;
        GPIO_ResetBits(GPIOA, GPIO_Pin_8);
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    }
    else
    {
        /* 有源: 给恒定电平, 不能再翻转 */
        TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
        if (Beep_Mode == 1) GPIO_SetBits(GPIOA, GPIO_Pin_8);
        else                GPIO_ResetBits(GPIOA, GPIO_Pin_8);
    }
}

void Beep_Off(void)
{
    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
    if (Beep_Mode == 2) GPIO_SetBits(GPIOA, GPIO_Pin_8);   /* 低有效: 静默为高 */
    else                GPIO_ResetBits(GPIOA, GPIO_Pin_8);
}

/* 在 5ms 控制中断中调用: 只管时长, 方波由 TIM3 中断出 */
void Beep_Tick(void)
{
    if (Beep_Timer > 0)
    {
        Beep_Timer -= 5;
        if (Beep_Timer < 5)
        {
            Beep_Timer = 0;
            Beep_Off();
        }
    }
}

/* TIM3 更新中断: 蜂鸣期间按分频翻转 PA8 产生方波 */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        if ((Beep_Timer == 0) || (Beep_Mode != 0))
        {
            Beep_Off();
            return;
        }

        s_cnt++;
        if (s_cnt >= s_div)
        {
            s_cnt = 0;
            s_pin = !s_pin;
            GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)s_pin);
        }
    }
}
