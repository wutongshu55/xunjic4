#include "Delay.h"

/* 简单延时函数，基于72MHz主频 */
void DelayMs(uint32_t ms)
{
    uint32_t i, j;
    for(i = 0; i < ms; i++)
        for(j = 0; j < 8000; j++);  /* 约1ms @ 72MHz */
}

void DelayUs(uint32_t us)
{
    uint32_t i;
    while(us--)
    {
        for(i = 0; i < 8; i++);  /* 约1us @ 72MHz */
    }
}

/* 兼容旧代码中的 Delay_ms 命名 */
void Delay_ms(uint32_t ms)
{
    DelayMs(ms);
}
