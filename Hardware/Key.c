#include "Common_headfile.h"

/* 原理图: KEY1=PA9, KEY2=PA10, KEY3=PA11, KEY4=PA12 */

uint8_t Key_Num = 0;            /* 当前按下的键号 1~4, 0无按键 */
uint8_t Key_Last = 0;           /* 上次键号(去抖) */

static uint8_t Key_ReadPin(void)
{
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9)  == 0) return 1;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10) == 0) return 2;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == 0) return 3;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == 0) return 4;
    return 0;
}

void Key_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/* 在主循环中周期调用(约10ms), 返回按键事件 */
uint8_t Key_GetNum(void)
{
    uint8_t cur = Key_ReadPin();
    if (cur != 0 && cur == Key_Last)
    {
        /* 持续按下, 仅在第一次稳定时返回 */
        if (Key_Num == 0)
        {
            Key_Num = cur;
            return cur;
        }
        return 0;
    }
    else
    {
        Key_Last = cur;
        if (cur == 0) Key_Num = 0;
        return 0;
    }
}
