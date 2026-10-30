#include "Common_headfile.h"

/* 显示模式: 0=电感调试 1=菜单 2=运行 */
uint8_t Display_Mode = 0;
uint8_t Debug_Inited = 0;

/* 系统毫秒时基: 由 TIM1 中断(5ms)累加。
 * 按键长按判定用它, 不受主循环周期(刷屏约 50ms/帧)影响。 */
volatile uint32_t Sys_Tick_ms = 0;

static void Debug_ShowSensor(uint8_t line, uint8_t value_column,
                             uint8_t status_column, uint8_t adc_index)
{
    uint16_t value = ADC_Value[adc_index];

    OLED_ShowNum(line, value_column, value, 4);

    if (value < ADC_MIN[adc_index])
    {
        OLED_ShowString(line, status_column, "MIN-");
    }
    else if (value > ADC_MAX[adc_index])
    {
        OLED_ShowString(line, status_column, "MAX+");
    }
    else
    {
        OLED_ShowString(line, status_column, " OK ");
    }
}

int main(void)
{
    uint8_t key;

    All_Init();

    while (1)
    {
        key = Key_GetNum();

        /* KEY4 长按(约 0.8 秒): 调试页 <-> 菜单 切换。
         * 短按 KEY4 仍然是正常功能键(菜单里加数值), 不会一按就跳界面。 */
        if (key == KEY_EV_LONG4)
        {
            if (Display_Mode == 0)
            {
                Display_Mode = 1;          /* 调试页 -> 菜单 */
                Menu_Inited = 0;
            }
            else if (Display_Mode == 1)
            {
                Display_Mode = 0;          /* 菜单 -> 调试页 */
                Debug_Inited = 0;
            }
            else
            {
                Display_Mode = 0;          /* 运行中 -> 暂停并回调试页 */
                Menu_Running = 0;
                Debug_Inited = 0;
            }
            key = 0;
        }

        /* 发车后自动切到运行显示 */
        if (Display_Mode == 1 && Menu_Page == MENU_PAGE_START && Menu_Running == 1)
        {
            Display_Mode = 2;
            Menu_RunInited = 0;
        }
        /* 完成停车后切回调试 */
        if (Display_Mode == 2 && Menu_Running == 2)
        {
            Display_Mode = 0;
            Debug_Inited = 0;
        }
        /* 运行中被 KEY4 停车(中断里置 Running=0) -> 回发车页, 等待重新发车。
         * 停车本身在 Key_Scan_5ms() 里做, 这里只负责把界面切过去。 */
        else if (Display_Mode == 2 && Menu_Running == 0)
        {
            Menu_Page    = MENU_PAGE_START;
            Menu_Inited  = 0;
            Display_Mode = 1;
        }

        /* 菜单模式按键处理 */
        if (Display_Mode == 1 && key != 0)
        {
            Menu_KeyHandler(key);
            /* 发车页按KEY4后, 本次按键事件已消费, 清空避免立刻触发暂停 */
            if (Menu_Page == MENU_PAGE_START && Menu_Running == 1) key = 0;
        }
        /* 运行页按键:
         *   KEY4      = 停车, 并回到发车页(显示 IDLE), 再按一次即可重新发车
         *   KEY1~KEY3 = 应急暂停(保留圈数), 回调试页
         * 停车统一走发车页的 Menu_KeyHandler, 避免两处对 Running 的判断不一致。 */
        if (Display_Mode == 2 && key != 0)
        {
            if (key == 4)
            {
                Menu_Page = MENU_PAGE_START;
                Menu_KeyHandler(4);        /* Running 1 -> 0 */
                Menu_Inited  = 0;
                Display_Mode = 1;
            }
            else
            {
                Display_Mode = 0;
                Menu_Running = 0;
                Debug_Inited = 0;
            }
            key = 0;
        }

        /* ---- 显示刷新 ---- */

        /* 调试界面: 首次画标签, 之后更新读数与标定提示 */
        if (Display_Mode == 0)
        {
            if (!Debug_Inited)
            {
                OLED_Clear();
                OLED_ShowString(1, 1, "L  PA7:");
                OLED_ShowString(2, 1, "ML PA6:");
                OLED_ShowString(3, 1, "MR PA5:");
                OLED_ShowString(4, 1, "R  PA4:");
                Debug_Inited = 1;
            }
            Debug_ShowSensor(1, 8, 13, 3);
            Debug_ShowSensor(2, 8, 13, 2);
            Debug_ShowSensor(3, 8, 13, 1);
            Debug_ShowSensor(4, 8, 13, 0);
        }

        /* 菜单界面 */
        if (Display_Mode == 1)
        {
            Menu_Display();
        }

        /* 运行界面 */
        if (Display_Mode == 2)
        {
            Menu_RunDisplay();
        }

        Delay_ms(50);
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
        /* 0. 系统毫秒时基 + 按键扫描(含运行中的紧急停车) */
        Sys_Tick_ms += 5;
        Key_Scan_5ms();

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
            /* 基础速度先定下来, 后面调度用 */
            Speed_Basic = Speed_Level_Table[Menu_SpeedLevel];

            /* 4.1 异常检测: 丢线 / 急弯(锯齿线、直角弯) */
            PID_LostLine_Check();
            PID_Sharp_Check();

            /* 4.2 PD 循迹 */
            PID_Calculate();

            /* 4.3 赛道元素识别(十字路口/圆环), 会给出额外偏置与速度系数 */
            Track_Judge();

            /* 4.4 合成最终差速: 十字路口强制直行, 再叠加元素偏置与限幅 */
            if (Track_Straight)
            {
                Speed_Delta = 0;
            }
            Speed_Delta = PID_Apply(Speed_Delta);

            /* 4.5 速度调度: 弯道降速, 元素降速 */
            Speed_Basic = PID_Speed_Schedule(Speed_Basic);

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
