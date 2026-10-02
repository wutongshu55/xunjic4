#include "Common_headfile.h"

/* 原理图: BEEP=PA8 */

volatile uint16_t Beep_Timer = 0;   /* 蜂鸣剩余时间(ms) */

void Beep_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOA, GPIO_Pin_8);
}

/* 开启蜂鸣器持续 time_ms 毫秒 */
void Beep_On(uint16_t time_ms)
{
    Beep_Timer = time_ms;
    GPIO_SetBits(GPIOA, GPIO_Pin_8);
}

void Beep_Off(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_8);
}

/* 在定时中断中调用, 每1ms减1 */
void Beep_Tick(void)
{
    if (Beep_Timer > 0)
    {
        Beep_Timer--;
        if (Beep_Timer == 0)
        {
            Beep_Off();
        }
    }
}
