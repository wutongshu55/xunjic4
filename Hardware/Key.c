#include "Common_headfile.h"

/* 原理图: KEY1=PA9, KEY2=PA10, KEY3=PA11, KEY4=PA12 */

/* 去抖时间: 电平稳定这么久才认可(单位 ms, 按 5ms 一次采样) */
#define KEY_DEBOUNCE_MS  20

uint8_t Key_Num  = 0;       /* 兼容旧接口: 当前认可的键号 1~4, 0=无 */
uint8_t Key_Last = 0;       /* 兼容旧接口: 上一次采样的原始键号 */

/* ---- 扫描状态(只在 5ms 中断里改) ---- */
static uint8_t  s_raw_last  = 0;   /* 上次采样电平 */
static uint8_t  s_deb_ms    = 0;   /* 去抖计时 */
static uint8_t  s_stable    = 0;   /* 已认可的按下键号 */
static uint32_t s_hold_ms   = 0;   /* 已按住时长 */
static uint8_t  s_long_sent = 0;   /* 本次按下是否已发过长按事件 */
static uint8_t  s_consumed  = 0;   /* 本次按下是否已被"紧急停车"消费 */

/* ---- 事件队列(中断写, 主循环读) ---- */
#define KEY_EVQ_SIZE  8
static volatile uint8_t s_ev_head = 0;
static volatile uint8_t s_ev_tail = 0;
static volatile uint8_t s_ev_buf[KEY_EVQ_SIZE];

static void Key_Push(uint8_t ev)
{
    uint8_t nxt = (uint8_t)((s_ev_head + 1u) & (KEY_EVQ_SIZE - 1u));
    if (nxt != s_ev_tail)              /* 队列满就丢最新的, 不覆盖未读事件 */
    {
        s_ev_buf[s_ev_head] = ev;
        s_ev_head = nxt;
    }
}

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

/* ------------------------------------------------------------
 *  扫描函数: 必须由 TIM1 的 5ms 中断周期调用
 *
 *  为什么不能在主循环里扫:
 *    运行页每轮要刷 32 个数字(软件 I2C), 再加 Delay_ms(50),
 *    主循环一轮 100~200ms。按键只在主循环扫一次的话,
 *    短于一整轮的点按会整个被漏掉 —— 这就是"跑起来按 KEY4 没反应"。
 *    放进 5ms 中断后, 任何超过 20ms 的按下都必然被采到。
 *
 *  紧急停车在这里做, 而且"按下即停"不等松开:
 *    跑车时停车要快, 松手时间不该算进反应时间。
 *    这种按下被消费掉(不再产生事件), 避免主循环又把它当"发车"处理。
 * ------------------------------------------------------------ */
void Key_Scan_5ms(void)
{
    uint8_t raw = Key_ReadPin();

    Key_Last = raw;

    /* 1) 去抖: 电平变了就重新计时, 稳定够 20ms 才认可 */
    if (raw != s_raw_last)
    {
        s_raw_last = raw;
        s_deb_ms   = 0;
        return;
    }
    if (s_deb_ms < KEY_DEBOUNCE_MS)
    {
        s_deb_ms += 5;
        return;
    }

    /* 2) 按住 */
    if (raw != 0)
    {
        if (s_stable != raw)
        {
            /* 新的一次按下 */
            s_stable    = raw;
            s_hold_ms   = 0;
            s_long_sent = 0;
            s_consumed  = 0;
            Key_Num     = raw;

            /* 运行中按 KEY4 -> 立即停车(按下即停) */
            if ((raw == 4) && (Menu_Running == 1))
            {
                Menu_Running = 0;
                Beep_On(100);
                s_consumed = 1;      /* 这次按下到此为止, 不再产生事件 */
            }
        }
        else
        {
            s_hold_ms += 5;

            /* 长按: 只有 KEY4 用得上(进/出调试页), 到时间只发一次 */
            if ((raw == 4) && (!s_consumed) && (!s_long_sent) &&
                (s_hold_ms >= KEY_LONG_MS))
            {
                s_long_sent = 1;
                Key_Push(KEY_EV_LONG4);
            }
        }
        return;
    }

    /* 3) 松开: 没发过长按、也没被停车消费 -> 补一个短按事件 */
    Key_Num = 0;
    if (s_stable != 0)
    {
        if ((!s_consumed) && (!s_long_sent))
        {
            Key_Push(s_stable);
        }
        s_stable = 0;
    }
}

/* 主循环取事件(不阻塞): 0=无事件, 1~4=短按, KEY_EV_LONG4=KEY4长按 */
uint8_t Key_GetNum(void)
{
    uint8_t ev = 0;

    if (s_ev_tail != s_ev_head)
    {
        ev = s_ev_buf[s_ev_tail];
        s_ev_tail = (uint8_t)((s_ev_tail + 1u) & (KEY_EVQ_SIZE - 1u));
    }
    return ev;
}