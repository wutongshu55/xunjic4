#include "Common_headfile.h"

void All_Init(void)
{
    /* 电感标定值默认(需实际标定后修改)
     *
     * 复合布局标定要点:
     *   横置 L/R : MAX = 正常巡航姿态下直道居中的读数(调好后 1500~2500)
     *   竖置 ML/MR: MAX = 十字/环岛交叉点处的峰值(3200~3600)
     * 两者绝对值【不要求一致】, 每路 100 代表各自的峰值。
     *
     * 这里先统一填 3600 作为占位: 与"峰值上限 3700"配套,
     * 归一化 100 对应峰值区间上沿, 饱和时不会溢出。
     */
    ADC_MAX[0] = 3600;  ADC_MIN[0] = 200;   /* R  右横置 */
    ADC_MAX[1] = 3600;  ADC_MIN[1] = 200;   /* MR 中右竖置 */
    ADC_MAX[2] = 3600;  ADC_MIN[2] = 200;   /* ML 中左竖置 */
    ADC_MAX[3] = 3600;  ADC_MIN[3] = 200;   /* L  左横置 */

    /* OLED 必须在 Timer_Init 之前初始化,
       否则 TIM1 中断会打断 I2C 时序导致屏幕不亮 */
    OLED_Init();
    Key_Init();
    Motor_Init();
    Encoder_Init();
    ADC_MyInit();
    /* 最后配置 PA8, 确保其他外设初始化不会覆盖蜂鸣器 GPIO。 */
    Beep_Init();
    Timer_Init();       /* 定时中断最后开启 */

    /* ---- 开机自检: 依次试 高电平 / 低电平 / 方波, 每段 200ms(间隔 150ms) ----
     * 按"哪一段响"直接确定蜂鸣器类型, 然后在菜单第 6 页设对应的 Beep 值:
     *   第 1 段响(高电平) -> 有源高有效, 设 1(默认)
     *   第 2 段响(低电平) -> 有源低有效, 设 2
     *   第 3 段响(方波)   -> 其实是无源, 设 0
     *   三段都不响        -> 硬件问题: PA8 焊点 / Q1(8050) / 蜂鸣器 5V 供电
     * 自检期间屏幕还是黑的, 专心听即可; 结束后 PA8 回到静默(低)。 */
    GPIO_SetBits(GPIOA, GPIO_Pin_8);    Delay_ms(200);   /* 1) 高电平 */
    GPIO_ResetBits(GPIOA, GPIO_Pin_8);  Delay_ms(150);
    Delay_ms(200);                                        /* 2) 低电平 */
    GPIO_SetBits(GPIOA, GPIO_Pin_8);    Delay_ms(150);
    Beep_Mode = 0;  Beep_On(200);       Delay_ms(320);   /* 3) 无源方波 */
    Beep_Mode = 1;                                        /* 恢复默认高电平 */
    GPIO_ResetBits(GPIOA, GPIO_Pin_8);

    OLED_Clear();
}
