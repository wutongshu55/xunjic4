#include "Common_headfile.h"

uint32_t Period_Data = 0;

void PWM_Init(uint32_t Freq)
{
    uint32_t Prescaler_Data;

    Prescaler_Data = (72000000 / Freq) >> 16;
    Period_Data = 72000000 / Freq;
    Period_Data = Period_Data / (Prescaler_Data + 1) - 1;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_InternalClockConfig(TIM3);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = Period_Data;
    TIM_TimeBaseInitStructure.TIM_Prescaler = Prescaler_Data;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);

    TIM_Cmd(TIM3, ENABLE);

    TIM_SetCompare3(TIM3, 0);
    TIM_SetCompare4(TIM3, 0);
}

void PWMA_SetDuty(int16_t Duty)
{
    uint16_t CCR3_Data;

    Duty = Duty < 0 ? 0 : Duty;
    Duty = Duty > PWM_DUTY_MAX ? PWM_DUTY_MAX : Duty;
    CCR3_Data = (uint16_t)(Period_Data * ((float)Duty / PWM_DUTY_MAX));
    TIM_SetCompare3(TIM3, CCR3_Data);
}

void PWMB_SetDuty(int16_t Duty)
{
    uint16_t CCR4_Data;

    Duty = Duty < 0 ? 0 : Duty;
    Duty = Duty > PWM_DUTY_MAX ? PWM_DUTY_MAX : Duty;
    CCR4_Data = (uint16_t)((Period_Data + 1) * ((float)Duty / PWM_DUTY_MAX));
    TIM_SetCompare4(TIM3, CCR4_Data);
}
