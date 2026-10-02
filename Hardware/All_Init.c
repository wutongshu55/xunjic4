#include "Common_headfile.h"

void All_Init(void)
{
    /* 电感标定值默认(需实际标定后修改) */
    ADC_MAX[0] = 4000;  ADC_MIN[0] = 200;
    ADC_MAX[1] = 4000;  ADC_MIN[1] = 200;
    ADC_MAX[2] = 4000;  ADC_MIN[2] = 200;
    ADC_MAX[3] = 4000;  ADC_MIN[3] = 200;

    Timer_Init();
    OLED_Init();
    Beep_Init();
    Key_Init();
    Motor_Init();
    Encoder_Init();
    ADC_MyInit();

    OLED_Clear();
}
