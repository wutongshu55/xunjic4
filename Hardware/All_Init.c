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
    Beep_Init();
    Key_Init();
    Motor_Init();
    Encoder_Init();
    ADC_MyInit();
    Timer_Init();       /* 定时中断最后开启 */

    OLED_Clear();
}
