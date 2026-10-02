#include "Common_headfile.h"

/* ============================================================
 *  赛道元素判断: 十字路口 / 菱形环 / 圆环 / 圈数
 *  原理图四路电感: L(PA4) ML(PA5) MR(PA6) R(PA7)
 *  归一化后 ADC_L/ADC_ML/ADC_MR/ADC_R 范围 0~100
 * ============================================================ */

uint8_t Ring_Flag = 0;        /* 圆环状态机 */
uint16_t Ring_CountNum = 0;   /* 圆环计时计数 */
uint8_t Cross_Stage = 0;      /* 十字路口检测阶段 */
uint16_t Cross_Timer = 0;     /* 十字路口计时 */
uint8_t Diamond_Stage = 0;   /* 菱形环状态机 */
uint16_t Diamond_Timer = 0;  /* 菱形环计时 */

/* 阈值定义 */
#define TH_HIGH        60    /* 电感高于此值视为"在线" */
#define TH_LOW         15    /* 电感低于此值视为"离线" */
#define TH_ALL_HIGH    70    /* 四路都很高 = 路口/铁皮集中 */
#define RING_DELAY     150   /* 环内循迹延时(控制周期数) */
#define RING_TURN      200   /* 环入弯转弯延时 */
#define RING_EXIT      120   /* 环出弯延时 */
#define CROSS_DELAY    80    /* 十字路口直行延时 */

/* ============================================================
 *  十字路口判断
 *  特征: 四路电感同时很高(都>=TH_ALL_HIGH)
 *  动作: 直行不转弯, 计数+1
 * ============================================================ */
void Cross_Judge(void)
{
    switch (Cross_Stage)
    {
        case 0:
            if (ADC_L >= TH_ALL_HIGH && ADC_ML >= TH_ALL_HIGH &&
                ADC_MR >= TH_ALL_HIGH && ADC_R >= TH_ALL_HIGH)
            {
                Cross_Stage = 1;
                Cross_Timer = 0;
                Menu_CrossCount++;
                Beep_On(30);
            }
            break;

        case 1:
            Cross_Timer++;
            if (Cross_Timer >= CROSS_DELAY)
            {
                Cross_Stage = 0;
                Cross_Timer = 0;
            }
            break;

        default:
            Cross_Stage = 0;
            break;
    }
}

/* ============================================================
 *  菱形环判断 (赛道右侧, 固定右转进环)
 * ============================================================ */
void Diamond_Judge(void)
{
    if (Menu_DiamondEn == 0)
    {
        Diamond_Stage = 0;
        Diamond_Timer = 0;
        return;
    }

    switch (Diamond_Stage)
    {
        case 0:
            if (ADC_R >= TH_HIGH && ADC_MR >= TH_HIGH)
            {
                Diamond_Stage = 1;
                Diamond_Timer = 0;
            }
            break;

        case 1:
            Diamond_Timer++;
            Speed_Delta = -1200;   /* 右转进环 */

            if (Diamond_Timer >= RING_TURN)
            {
                Diamond_Stage = 2;
                Diamond_Timer = 0;
            }
            break;

        case 2:
            Diamond_Timer++;
            if (Diamond_Timer >= RING_DELAY)
            {
                Diamond_Stage = 3;
                Diamond_Timer = 0;
            }
            break;

        case 3:
            Diamond_Timer++;
            Speed_Delta = -1000;   /* 右转出环 */

            if (Diamond_Timer >= RING_EXIT)
            {
                Diamond_Stage = 0;
                Diamond_Timer = 0;
                Beep_On(50);
            }
            break;

        default:
            Diamond_Stage = 0;
            break;
    }
}

/* ============================================================
 *  圆环判断 (赛道右侧, 固定右转进环)
 * ============================================================ */
void Ring_Judge(void)
{
    if (Menu_RingEn == 0)
    {
        Ring_Flag = 0;
        Ring_CountNum = 0;
        return;
    }

    switch (Ring_Flag)
    {
        case 0:
            if (ADC_R >= TH_HIGH && ADC_MR < TH_LOW)
            {
                Ring_Flag = 1;
                Ring_CountNum = 0;
            }
            break;

        case 1:
            Ring_CountNum++;
            Speed_Delta = -1050;   /* 右转进环 */

            if (Ring_CountNum >= RING_TURN)
            {
                Ring_Flag = 2;
                Ring_CountNum = 0;
            }
            break;

        case 2:
            Ring_CountNum++;
            if (Ring_CountNum >= RING_DELAY)
            {
                Ring_Flag = 3;
                Ring_CountNum = 0;
            }
            break;

        case 3:
            if (ADC_R >= TH_HIGH && ADC_MR >= TH_HIGH)
            {
                Ring_Flag = 4;
                Ring_CountNum = 0;
            }
            break;

        case 4:
            Ring_CountNum++;
            Speed_Delta = -1000;   /* 右转出环 */

            if (Ring_CountNum >= RING_EXIT)
            {
                Ring_Flag = 0;
                Ring_CountNum = 0;
                Beep_On(50);
            }
            break;

        default:
            Ring_Flag = 0;
            Ring_CountNum = 0;
            break;
    }
}

/* ============================================================
 *  圈数判断: 通过起点(四路全高)
 * ============================================================ */
void Lap_Judge(void)
{
    static uint8_t lap_lock = 0;

    if (ADC_L >= TH_ALL_HIGH && ADC_ML >= TH_ALL_HIGH &&
        ADC_MR >= TH_ALL_HIGH && ADC_R >= TH_ALL_HIGH)
    {
        if (lap_lock == 0)
        {
            lap_lock = 1;
            Menu_DoneLaps++;
            Beep_On(100);

            if (Menu_DoneLaps >= Menu_TargetLaps)
            {
                Menu_Running = 2;
            }
        }
    }
    else if (ADC_L < TH_LOW && ADC_R < TH_LOW)
    {
        lap_lock = 0;
    }
}

/* 总判断函数, 在中断中调用 */
void Track_Judge(void)
{
    Cross_Judge();
    Diamond_Judge();
    Ring_Judge();
    Lap_Judge();
}
