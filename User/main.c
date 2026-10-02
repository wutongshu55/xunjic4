#include "Common_headfile.h"

/* 显示模式: 0=菜单 1=运行 2=标定 3=编码器测试 */
uint8_t Display_Mode = 0;
uint8_t Display_Refresh = 1;

int main(void)
{
    uint8_t key;
    uint16_t tick = 0;

    All_Init();

    OLED_ShowString(1, 1, "SmartCar v1.0");
    OLED_ShowString(2, 1, "TianKeDa EM");
    OLED_ShowString(3, 1, "K1/K2:Page");
    OLED_ShowString(4, 1, "K3- K4+");
    Delay_ms(1500);

    while (1)
    {
        key = Key_GetNum();

        /* 全局: 长按KEY1+KEY4 切换显示模式(简化: KEY2在菜单页切换到运行显示) */
        if (Display_Mode == 0 && Menu_Page == 5 && Menu_Running == 1)
        {
            /* 发车后自动切到运行显示 */
            Display_Mode = 1;
            Display_Refresh = 1;
        }
        if (Display_Mode == 1 && Menu_Running == 2)
        {
            /* 完成停车后切回菜单 */
            Display_Refresh = 1;
        }

        if (key != 0)
        {
            if (Display_Mode == 1)
            {
                /* 运行中按任意键回到菜单(暂停) */
                Display_Mode = 0;
                Menu_Running = 0;
                Display_Refresh = 1;
            }
            else
            {
                Menu_KeyHandler(key);
                Display_Refresh = 1;
            }
        }

        /* 显示刷新 */
        if (Display_Refresh)
        {
            if (Display_Mode == 0)
                Menu_Display();
            else
                Menu_RunDisplay();
            Display_Refresh = 0;
        }

        /* 运行中周期刷新运行显示(约200ms) */
        if (Display_Mode == 1)
        {
            tick++;
            if (tick >= 20)
            {
                tick = 0;
                Menu_RunDisplay();
            }
        }

        Delay_ms(10);
    }
}

/* TIM1 中断: 5ms控制周期
 * Prescaler=3600-1, Period=100-1 => 72M/3600/100 = 200Hz = 5ms
 */
void TIM1_UP_IRQHandler(void)
{
    uint8_t i;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        /* 1. ADC采样 */
        for (i = 0; i < 20; i++)
        {
            ADC_GetValue();
        }
        ADC_Calibrate();
        Get_Error();

        /* 2. 编码器更新(采样周期5ms=0.005s) */
        Encoder_Update(0.005f);

        /* 3. 蜂鸣器计时 */
        Beep_Tick();

        /* 4. 运行控制 */
        if (Menu_Running == 1)
        {
            PID_Calculate();
            Track_Judge();

            /* 根据车速档位设置基础速度 */
            Speed_Basic = Speed_Level_Table[Menu_SpeedLevel];

            CarMotor_Control();
        }
        else if (Menu_Running == 2)
        {
            /* 完成停车 */
            Motor_Out(0, 0);
            Speed_Basic = 0;
        }
        else
        {
            /* 待发车, 电机停止 */
            Motor_Out(0, 0);
        }

        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
