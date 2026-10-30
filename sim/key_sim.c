/*
 * 按键状态机主机端仿真 —— 验证长短按 / 紧急停车 / 事件队列
 *
 * Key_Scan_5ms() 设计为由 TIM1 的 5ms 中断调用, 这里直接按 5ms 步进模拟。
 *
 * 编译:
 *   gcc -O1 -Wall -Isim -IHardware -o sim/key_sim.exe sim/key_sim.c Hardware/Key.c -lm
 */
#include "Common_headfile.h"

/* ---- 被测代码依赖的全局量/桩 ---- */
static GPIO_TypeDef g_gpioa, g_gpiob;
GPIO_TypeDef *GPIOA = &g_gpioa;
GPIO_TypeDef *GPIOB = &g_gpiob;

uint8_t  Menu_Running = 0;
volatile uint32_t Sys_Tick_ms = 0;

static int g_beep_ms = 0;
void Beep_On(uint16_t time_ms) { g_beep_ms += time_ms; }

/* 引脚桩: g_pin = 0 无键, 1~4 表示对应按键按下 */
static uint8_t g_pin = 0;

uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *p, uint16_t pin)
{
    (void)p;
    if (pin == GPIO_Pin_9  && g_pin == 1) return 0;
    if (pin == GPIO_Pin_10 && g_pin == 2) return 0;
    if (pin == GPIO_Pin_11 && g_pin == 3) return 0;
    if (pin == GPIO_Pin_12 && g_pin == 4) return 0;
    return 1;
}
void GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *s) { (void)p; (void)s; }
void RCC_APB2PeriphClockCmd(uint32_t a, uint32_t b) { (void)a; (void)b; }

/* ---- 测试脚手架 ---- */
static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...) do { g_checks++; if (!(cond)) { g_fail++; \
    printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* 事件收集: 每 5ms 轮询一次, 记录产生的事件 */
#define MAX_EV 16
static uint8_t g_ev[MAX_EV];
static int     g_ev_n = 0;

static void Tick(int ms)
{
    int i;
    uint8_t e;
    for (i = 0; i < ms; i += 5)
    {
        Sys_Tick_ms += 5;
        Key_Scan_5ms();
        e = Key_GetNum();
        if (e != 0 && g_ev_n < MAX_EV) g_ev[g_ev_n++] = e;
    }
}

static void Ev_Reset(void) { g_ev_n = 0; memset(g_ev, 0, sizeof(g_ev)); }

/* 按住 ms 毫秒后松开, 再等 60ms 让松开事件产生 */
static void Press(uint8_t key, int ms)
{
    g_pin = key;
    Tick(ms);
    g_pin = 0;
    Tick(60);
}

static void Reset_All(void)
{
    g_pin = 0;
    Menu_Running = 0;
    Ev_Reset();
    Tick(100);
    Ev_Reset();
}

int main(void)
{
    printf("=========================================\n");
    printf(" 按键状态机仿真 (5ms 扫描)\n");
    printf("=========================================\n");

    /* ---- 1. 空闲不产生事件 ---- */
    printf("\n[1] 空闲 500ms 不应产生事件\n");
    Reset_All();
    Tick(500);
    CHECK(g_ev_n == 0, "空闲时不应有事件, 实际 %d 个", g_ev_n);

    /* ---- 2. KEY4 短按 -> 事件 4 ---- */
    printf("\n[2] KEY4 短按 40ms -> 短按事件\n");
    Reset_All();
    Press(4, 40);
    printf("  事件数=%d 首个=%d\n", g_ev_n, g_ev_n ? g_ev[0] : -1);
    CHECK(g_ev_n == 1, "应产生 1 个事件, 实际 %d", g_ev_n);
    CHECK(g_ev[0] == 4, "应是短按事件 4, 实际 %d", g_ev[0]);

    /* ---- 3. KEY1 短按 -> 事件 1 ---- */
    printf("\n[3] KEY1 短按 40ms -> 短按事件\n");
    Reset_All();
    Press(1, 40);
    printf("  事件数=%d 首个=%d\n", g_ev_n, g_ev_n ? g_ev[0] : -1);
    CHECK(g_ev_n == 1 && g_ev[0] == 1, "应产生短按事件 1");

    /* ---- 4. KEY4 长按 -> 长按事件, 且不产生短按 ---- */
    printf("\n[4] KEY4 按住 900ms -> 长按事件(不产生短按)\n");
    Reset_All();
    Press(4, 900);
    printf("  事件数=%d 首个=0x%X (KEY_EV_LONG4=0x%X)\n",
           g_ev_n, g_ev_n ? g_ev[0] : 0, KEY_EV_LONG4);
    CHECK(g_ev_n == 1, "长按应只产生 1 个事件, 实际 %d", g_ev_n);
    CHECK(g_ev[0] == KEY_EV_LONG4, "应是长按事件, 实际 0x%X", g_ev[0]);

    /* ---- 5. 按住 300ms(不够长按) -> 仍是短按 ---- */
    printf("\n[5] KEY4 按住 300ms(不足 %d) -> 仍算短按\n", KEY_LONG_MS);
    Reset_All();
    Press(4, 300);
    printf("  事件数=%d 首个=%d\n", g_ev_n, g_ev_n ? g_ev[0] : -1);
    CHECK(g_ev_n == 1 && g_ev[0] == 4, "300ms 应仍算短按");

    /* ---- 6. 运行中按 KEY4 -> 立即停车, 不产生事件 ---- */
    printf("\n[6] 运行中按 KEY4 -> 立即停车(按下即停, 不等松开)\n");
    Reset_All();
    Menu_Running = 1;
    g_pin = 4;
    Tick(50);                       /* 按住 50ms, 还没松开 */
    printf("  按住 50ms 时: Running=%d (应为 0)  蜂鸣累计=%d\n", Menu_Running, g_beep_ms);
    CHECK(Menu_Running == 0, "按下 50ms 内就应停车, 实际 Running=%d", Menu_Running);

    g_pin = 0;
    Tick(60);
    printf("  松开后事件数=%d (应为 0, 否则会被当成短按又去发车)\n", g_ev_n);
    CHECK(g_ev_n == 0, "停车那次按下不应再产生事件, 实际 %d 个", g_ev_n);

    /* ---- 7. 运行中长按 KEY4 也不应切成"进调试页" ---- */
    printf("\n[7] 运行中一直按住 KEY4 900ms -> 不应产生长按事件\n");
    Reset_All();
    Menu_Running = 1;
    Press(4, 900);
    printf("  Running=%d 事件数=%d\n", Menu_Running, g_ev_n);
    CHECK(Menu_Running == 0, "长按也应已停车");
    CHECK(g_ev_n == 0, "停车后不应再产生长按事件, 实际 %d", g_ev_n);

    /* ---- 8. 未运行时 KEY4 长按仍可用(进菜单) ---- */
    printf("\n[8] 未运行时 KEY4 长按 -> 仍能进菜单\n");
    Reset_All();
    Menu_Running = 0;
    Press(4, 900);
    printf("  事件数=%d 首个=0x%X\n", g_ev_n, g_ev_n ? g_ev[0] : 0);
    CHECK(g_ev_n == 1 && g_ev[0] == KEY_EV_LONG4, "未运行时长按应产生长按事件");

    /* ---- 9. 极短点按(30ms) 也能采到(原来在主循环里会漏) ---- */
    printf("\n[9] 极短点按 30ms -> 仍应采到(旧实现在主循环里会漏掉)\n");
    Reset_All();
    Press(4, 30);
    printf("  事件数=%d 首个=%d\n", g_ev_n, g_ev_n ? g_ev[0] : -1);
    CHECK(g_ev_n == 1 && g_ev[0] == 4, "30ms 的点按应被采到");

    /* ---- 10. 连续两次短按应产生两个事件(队列不丢) ---- */
    printf("\n[10] 连续两次短按 -> 两个事件\n");
    Reset_All();
    Press(3, 40);
    Press(3, 40);
    printf("  事件数=%d\n", g_ev_n);
    CHECK(g_ev_n == 2, "两次短按应产生 2 个事件, 实际 %d", g_ev_n);

    /* ---- 11. 按住不放超过 800ms 只发一次长按 ---- */
    printf("\n[11] 长按 1500ms -> 长按事件只发一次\n");
    Reset_All();
    Press(4, 1500);
    printf("  事件数=%d\n", g_ev_n);
    CHECK(g_ev_n == 1, "长按只应产生 1 个事件, 实际 %d", g_ev_n);

    printf("\n=========================================\n");
    printf(" checks=%d  failures=%d  -> %s\n", g_checks, g_fail, g_fail ? "FAIL" : "ALL PASS");
    printf("=========================================\n");
    return g_fail ? 1 : 0;
}