#include "Common_headfile.h"

volatile int16_t Speed_Basic = 2000;

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

void CarMotor_Control(void)
{
    int16_t L_Speed, R_Speed;

    L_Speed = Speed_Basic + Speed_Delta;
    R_Speed = Speed_Basic - Speed_Delta;
    Motor_Out(L_Speed, R_Speed);
}
