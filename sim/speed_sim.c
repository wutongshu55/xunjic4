/*
 * 速度闭环仿真 —— 用一个一阶电机模型验证 PI + 前馈 + 死区补偿
 *
 * 编译:
 *   gcc -O1 -Wall -Isim -IHardware -o sim/speed_sim.exe sim/speed_sim.c Hardware/Motor.c -lm
 */
#include "Common_headfile.h"

/* ---------------- 电机 + 编码器模型 ---------------- */
#define PLANT_KV      0.091f    /* 每单位占空比对应的稳态速度 mm/s (10000档 ≈ 800mm/s) */
#define PLANT_DEAD    1200.0f   /* 低于此占空比电机不转 */
#define PLANT_TAU     0.08f     /* 机械时间常数 s */
#define ENC_RES       8.57f     /* 20ms 测速窗口的量化步长 mm/s */
#define ENC_PERIOD_MS 20

static float s_v = 0.0f;            /* 真实轮速 */
static int   s_enc_mode = 0;        /* 0=正常 1=无信号 2=反接 */
static int16_t s_duty_a = 0, s_duty_b = 0;

static void Plant_Step(float dt)
{
    float u, eff;

    u = (float)s_duty_a;
    if (fabsf(u) <= PLANT_DEAD) eff = 0.0f;
    else                        eff = (u > 0.0f ? 1.0f : -1.0f) * (fabsf(u) - PLANT_DEAD) * PLANT_KV;

    s_v += (eff - s_v) * (dt / PLANT_TAU);
    if (s_v < 0.0f) s_v = 0.0f;
}

/* ---------------- 被测代码需要的桩 ---------------- */
static int s_beep_cnt = 0;

void Beep_On(uint16_t ms) { (void)ms; s_beep_cnt++; }

static GPIO_TypeDef g_gpioa, g_gpiob;
GPIO_TypeDef *GPIOA = &g_gpioa;
GPIO_TypeDef *GPIOB = &g_gpiob;

volatile int16_t Speed_Delta = 0;
volatile int32_t Speed_L_mms = 0;
volatile int32_t Speed_R_mms = 0;
volatile int32_t Speed_Avg_mms = 0;

void PWM_Init(uint32_t freq) { (void)freq; }
void PWMA_SetDuty(int16_t duty) { s_duty_a = duty; }
void PWMB_SetDuty(int16_t duty) { s_duty_b = duty; }
void GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *s) { (void)p; (void)s; }
void GPIO_WriteBit(GPIO_TypeDef *p, uint16_t pin, uint8_t v) { (void)p; (void)pin; (void)v; }
void GPIO_ResetBits(GPIO_TypeDef *p, uint16_t pin) { (void)p; (void)pin; }
void GPIO_SetBits(GPIO_TypeDef *p, uint16_t pin) { (void)p; (void)pin; }
void RCC_APB2PeriphClockCmd(uint32_t a, uint32_t b) { (void)a; (void)b; }

/* ---------------- 测试脚手架 ---------------- */
static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...) do { g_checks++; if (!(cond)) { g_fail++; \
    printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static int s_t = 0;
static int g_sim_t = 0;

/* 跑 ms 毫秒, 返回期间的真实速度峰值 */
static float Run(int ms, float *peak_out)
{
    float peak = 0.0f;
    int i;

    for (i = 0; i < ms; i += 5)
    {
        /* 每 20ms 更新一次"测到的"速度(带量化) */
        if ((s_t % ENC_PERIOD_MS) == 0)
        {
            float meas = s_v;
            if (s_enc_mode == 1)      meas = 0.0f;    /* 编码器断线: 永远测到 0 */
            else if (s_enc_mode == 2) meas = -s_v;    /* A/B 反接: 测到负速度 */

            Speed_L_mms = (int32_t)((float)(int)(meas / ENC_RES + 0.5f) * ENC_RES);  /* 量化(四舍五入) */
            Speed_R_mms = Speed_L_mms;
            Speed_Avg_mms = Speed_L_mms;
        }

        CarMotor_Control();
        Plant_Step(0.005f);
        if (s_v > peak) peak = s_v;
        s_t += 5;
    }
    if (peak_out) *peak_out = peak;
    return s_v;
}

static void Reset_Plant(void)
{
    s_v = 0.0f;
    s_duty_a = 0;
    s_duty_b = 0;
    Speed_Delta = 0;
    Speed_Loop_Reset();
    Speed_Fault_Reset();
    s_enc_mode = 0;
    Speed_Loop_En = 1;
    Speed_KpS = 12.0f;      /* 与 Motor.c 的默认值保持一致 */
    Speed_KiS = 60.0f;
    Speed_L_mms = 0;
    Speed_R_mms = 0;
}

/* 参数扫描: 给定 (Kp,Ki), 从静止阶跃到 480, 返回 (调节时间, 超调%%) */
static void Sweep_One(float kp_s, float ki_s, float *settle, float *overshoot)
{
    int t;
    float peak = 0.0f;

    Reset_Plant();
    Speed_KpS = kp_s;
    Speed_KiS = ki_s;
    Speed_Basic = 480;
    g_sim_t = 0;
    Run(2500, &peak);

    *overshoot = (peak - 480.0f) / 480.0f * 100.0f;
    if (*overshoot < 0.0f) *overshoot = 0.0f;

    /* 二次跑一遍, 记录第一次进入 ±3%% 且不再出去的时间 */
    Reset_Plant();
    Speed_KpS = kp_s;
    Speed_KiS = ki_s;
    Speed_Basic = 480;
    *settle = -1.0f;
    for (t = 0; t < 2500; t += 5)
    {
        Run(5, NULL);
        if (fabsf(s_v - 480.0f) <= 480.0f * 0.03f)
        {
            if (*settle < 0.0f) *settle = (float)t;
        }
        else
        {
            *settle = -1.0f;   /* 又跑出去了, 重新计时 */
        }
    }
}

static void Sweep(void)
{
    float kps[] = {4.0f, 8.0f, 12.0f, 20.0f};
    float kis[] = {20.0f, 60.0f, 120.0f, 240.0f};
    int i, j;
    float st, ov;

    printf("\n--- 参数扫描 (阶跃 0->480mm/s) ---\n");
    printf(" Kp   Ki   调节时间   超调\n");
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            Sweep_One(kps[i], kis[j], &st, &ov);
            printf("%4.0f %4.0f   %6.0fms  %5.1f%%\n", kps[i], kis[j], st, ov);
        }
    }
    printf("\n");
}
int main(void)
{
    float v, peak;

    printf("=========================================\n");
    printf(" 速度闭环仿真 (前馈 + PI + 死区补偿)\n");
    Sweep();

    printf("=========================================\n");

    /* ---- 1. 阶跃到 480mm/s: 收敛 + 超调 ---- */
    printf("\n[1] 目标 480mm/s 阶跃响应\n");
    Reset_Plant();
    Speed_Basic = 480;
    Run(1500, &peak);
    v = s_v;
    printf("  1.5s 后实际 %.1f mm/s, 峰值 %.1f, 占空比 %d\n", v, peak, s_duty_a);
    CHECK(fabsf(v - 480.0f) <= 25.0f, "稳态误差过大: %.1f (目标 480)", v);
    CHECK(peak <= 480.0f * 1.25f, "超调过大: 峰值 %.1f", peak);

    /* ---- 2. 换档 480 -> 700 ---- */
    printf("\n[2] 换高速档 480 -> 700mm/s\n");
    Speed_Basic = 700;
    Run(1200, NULL);
    v = s_v;
    printf("  1.2s 后实际 %.1f mm/s\n", v);
    CHECK(fabsf(v - 700.0f) <= 30.0f, "高速档跟随失败: %.1f", v);

    /* ---- 3. 抗积分饱和: 目标超出电机能力后再回落 ---- */
    printf("\n[3] 目标 1400mm/s(超出能力) 1s 后回落到 480\n");
    Reset_Plant();
    Speed_Basic = 1400;
    Run(1000, NULL);
    printf("  饱和时占空比 %d, 实际 %.1f\n", s_duty_a, s_v);
    CHECK(s_duty_a <= PWM_DUTY_MAX, "占空比越限: %d", s_duty_a);

    Speed_Basic = 480;
    Run(1000, &peak);
    printf("  回落 1s 后实际 %.1f mm/s, 峰值 %.1f\n", s_v, peak);
    CHECK(fabsf(s_v - 480.0f) <= 30.0f, "积分饱和后回不来: %.1f", s_v);

    /* ---- 4. 开环对照: 只走前馈 ---- */
    printf("\n[4] 开环(仅前馈) 对比: 目标 480\n");
    Reset_Plant();
    Speed_Loop_En = 0;
    Speed_Basic = 480;
    Run(1500, NULL);
    printf("  开环 1.5s 后实际 %.1f mm/s (前馈系数不准会有稳态差, 正常)\n", s_v);
    CHECK(s_duty_a == (int16_t)(480.0f * 12.5f), "开环占空比应为 目标*12.5: %d", s_duty_a);

    /* ---- 5. 低速: 死区补偿是否生效 ---- */
    printf("\n[5] 低速 60mm/s: 死区补偿\n");
    Reset_Plant();
    Speed_Basic = 60;
    Run(1000, NULL);
    printf("  占空比 %d (应高于死区 1200), 实际 %.1f mm/s\n", s_duty_a, s_v);
    CHECK(s_duty_a >= 1200, "低速时应补偿到死区以上: %d", s_duty_a);

    /* ---- 6. 停车: 目标 0 立即输出 0 ---- */
    printf("\n[6] 目标 0 -> 占空比归零\n");
    Speed_Basic = 0;
    Run(200, NULL);
    printf("  占空比 %d, 实际 %.2f mm/s\n", s_duty_a, s_v);
    CHECK(s_duty_a == 0, "停车时占空比应为 0: %d", s_duty_a);

    /* ---- 7. 编码器断线: 低档也必须受限, 不许跑满 ---- */
    printf("\n[7] 编码器断线(测速恒 0), 低档 300mm/s\n");
    Reset_Plant();
    s_enc_mode   = 1;
    Speed_Basic  = 300;
    Run(1500, &peak);
    printf("  故障标志 F%u, 占空比 %d, 实际 %.1f mm/s (峰值 %.1f)\n",
           Speed_Fault, s_duty_a, s_v, peak);
    CHECK(Speed_Fault == 1,            "应检测到测速失效(F1), 实际 F%u", Speed_Fault);
    CHECK(s_duty_a <= 7400,            "降级后占空比应受限(<=7400), 实际 %d", s_duty_a);
    CHECK(s_v <= 420.0f,               "降级后速度应受限(<=420), 实际 %.1f", s_v);

    /* ---- 8. 编码器 A/B 反接: 自动纠正后仍要闭环到目标 ---- */
    printf("\n[8] 编码器 A/B 反接(测速为负), 低档 300mm/s\n");
    Reset_Plant();
    s_enc_mode  = 2;
    Speed_Basic = 300;
    Run(2000, &peak);
    printf("  故障标志 F%u, 占空比 %d, 实际 %.1f mm/s (峰值 %.1f)\n",
           Speed_Fault, s_duty_a, s_v, peak);
    CHECK(Speed_Fault == 2,            "应识别为编码器反接(F2), 实际 F%u", Speed_Fault);
    CHECK(fabsf(s_v - 300.0f) <= 40.0f, "纠正后应闭环到 300, 实际 %.1f", s_v);
    CHECK(peak <= 300.0f * 1.8f,       "纠正过程不应冲到满速: 峰值 %.1f", peak);

    printf("\n=========================================\n");
    printf(" checks=%d  failures=%d  -> %s\n", g_checks, g_fail, g_fail ? "FAIL" : "ALL PASS");
    printf("=========================================\n");
    return g_fail ? 1 : 0;
}