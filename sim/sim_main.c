/*
 * 电磁循迹算法主机端仿真验证 —— 复合(2横+2竖)电感布局版
 *
 * 传感器分工(与实物一致):
 *   两侧【横置】 L / R    —— 循迹主力(80%), 直道上就有 1500~2500 的读数
 *   中间【竖置】 ML / MR  —— 特殊元素探测器(20%), 直道上接近 0,
 *                            到十字/环岛磁场叠加处才抬头
 *
 * 验证内容:
 *   S1  差比和误差是否与"离导线距离"无关
 *   S2  十字路口在 10/20/40cm 下能否识别
 *   S2b 路口一律只直行(含起点线) + 过再多路口也不自动停车
 *   S3  直线/直角弯/普通右弯 不误判
 *   S4  六角菱形环在三个距离下的入口识别
 *   S4b 六角菱形环闭环(进环/环内丢线/左侧重现出环)
 *   S5  丢线保护
 *   S6  急弯保持(锯齿线) + 横置电感回线判据
 *   S7  弯道降速调度
 *   S8  峰值保持与饱和检测(调电位器用)
 *
 * 编译:
 *   gcc -O1 -Wall -Isim -IHardware -o sim/sim.exe sim_main.c ADC.c PID.c Ring_Judge.c -lm
 */
#include "Common_headfile.h"

/* ================= 仿真用的空外设 ================= */
static GPIO_TypeDef       g_gpioa, g_gpiob;
static ADC_TypeDef        g_adc1;
static DMA_Channel_TypeDef g_dma1_ch1;
GPIO_TypeDef *GPIOA = &g_gpioa;
GPIO_TypeDef *GPIOB = &g_gpiob;
ADC_TypeDef  *ADC1  = &g_adc1;
DMA_Channel_TypeDef *DMA1_Channel1 = &g_dma1_ch1;

void RCC_APB2PeriphClockCmd(uint32_t a, uint32_t b) { (void)a; (void)b; }
void RCC_AHBPeriphClockCmd(uint32_t a, uint32_t b) { (void)a; (void)b; }
void RCC_ADCCLKConfig(uint32_t a) { (void)a; }
void GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *s) { (void)p; (void)s; }
void ADC_RegularChannelConfig(ADC_TypeDef *p, uint8_t c, uint8_t r, uint8_t t) { (void)p; (void)c; (void)r; (void)t; }
void ADC_Init(ADC_TypeDef *p, ADC_InitTypeDef *s) { (void)p; (void)s; }
void ADC_DMACmd(ADC_TypeDef *p, uint32_t c) { (void)p; (void)c; }
void ADC_Cmd(ADC_TypeDef *p, uint32_t c) { (void)p; (void)c; }
void ADC_ResetCalibration(ADC_TypeDef *p) { (void)p; }
uint8_t ADC_GetResetCalibrationStatus(ADC_TypeDef *p) { (void)p; return 0; }
void ADC_StartCalibration(ADC_TypeDef *p) { (void)p; }
uint8_t ADC_GetCalibrationStatus(ADC_TypeDef *p) { (void)p; return 0; }
void ADC_SoftwareStartConvCmd(ADC_TypeDef *p, uint32_t c) { (void)p; (void)c; }
void DMA_Init(DMA_Channel_TypeDef *p, DMA_InitTypeDef *s) { (void)p; (void)s; }
void DMA_Cmd(DMA_Channel_TypeDef *p, uint32_t c) { (void)p; (void)c; }
void DMA_SetCurrDataCounter(DMA_Channel_TypeDef *p, uint16_t n) { (void)p; (void)n; }
uint8_t DMA_GetFlagStatus(uint32_t f) { (void)f; return 1; }
void DMA_ClearFlag(uint32_t f) { (void)f; }

/* ================= 菜单/电机等全局量 ================= */
uint8_t  Menu_HexDiaEn = 1;
uint8_t  Menu_SpeedLevel = 1;
uint8_t  Menu_Running = 1;      /* 只有 0=停 / 1=跑(圈数与自动停车已删除) */
uint8_t  Menu_CrossCount = 0;
uint8_t  Menu_Page = 0;
uint8_t  Menu_CalibMode = 0;
uint8_t  Menu_ZigEn = 1;
uint8_t  Menu_DistRaw = 0;
const int16_t Speed_Level_Table[3] = {3000, 5000, 7500};
volatile int16_t Speed_Basic = 5000;
/* PID.c 的航向估计读这两个量(宿主仿真里由测试用例直接写) */
volatile int32_t Speed_L_mms = 0;
volatile int32_t Speed_R_mms = 0;
volatile int32_t Speed_Avg_mms = 0;

/* ================= 测试脚手架 ================= */
static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...) do { g_checks++; if (!(cond)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* 标定基准: 与 All_Init.c 默认一致(峰值上限 3700 配套) */
#define SIM_MAX 3600
#define SIM_MIN 200

/*
 * 直接注入原始 ADC 值。
 * 通道顺序: L(PA7) ML(PA6) MR(PA5) R(PA4) -> ADC_Value[3],[2],[1],[0]
 */
static void FeedRaw(uint16_t rL, uint16_t rML, uint16_t rMR, uint16_t rR)
{
    ADC_Value[3] = rL;
    ADC_Value[2] = rML;
    ADC_Value[1] = rMR;
    ADC_Value[0] = rR;
    Get_Error();
}

/*
 * 注入一组"归一化波形"(0~100)并乘以距离衰减系数 k。
 * k=1.00 相当于 10cm; k=0.25 相当于 20cm; k=0.0625 相当于 40cm
 * (电磁场随距离衰减, 经验上接近 1/r^2)
 *
 * 参数顺序: 横置(L, R) 与 竖置(ML, MR) 一起给, 由调用者按布局填写。
 */
static void Feed(int16_t nL, int16_t nML, int16_t nMR, int16_t nR, float k)
{
    float rL, rML, rMR, rR;
    rL  = (float)ADC_MIN[3] + (float)(ADC_MAX[3] - ADC_MIN[3]) * (float)nL  * k / 100.0f;
    rML = (float)ADC_MIN[2] + (float)(ADC_MAX[2] - ADC_MIN[2]) * (float)nML * k / 100.0f;
    rMR = (float)ADC_MIN[1] + (float)(ADC_MAX[1] - ADC_MIN[1]) * (float)nMR * k / 100.0f;
    rR  = (float)ADC_MIN[0] + (float)(ADC_MAX[0] - ADC_MIN[0]) * (float)nR  * k / 100.0f;
    FeedRaw((uint16_t)rL, (uint16_t)rML, (uint16_t)rMR, (uint16_t)rR);
}

/* ---------- 各种赛道路况的"归一化波形"(k=1 时) ---------- */
#define STRAIGHT(KK)   Feed(55,  2,  2, 55, (KK))   /* 直道居中: 横置对称, 竖置接近 0 */
#define OFF_LEFT(KK)   Feed(85,  3,  0,  5, (KK))   /* 车偏右/导线偏左: 左横置强 */
#define CROSS(KK)      Feed(50, 88, 88, 50, (KK))   /* 十字: 横置仍在 + 竖置猛升 */
#define CORNER90(KK)   Feed(75, 40, 40, 15, (KK))   /* 直角弯: 竖置也升, 但横置明显偏一侧 */
#define CURVE_R(KK)    Feed( 5,  0,  0, 85, (KK))   /* 普通右弯: 竖置始终接近 0 */
#define HEX_ENTRY(KK)  Feed( 8, 20, 30, 90, (KK))   /* 六角菱形环入口: 强右偏 + 竖置抬头 */
#define HEX_INSIDE(KK) Feed( 5, 10, 40, 80, (KK))   /* 环内 */
#define HEX_BACK(KK)   Feed(90,  5,  5,  8, (KK))   /* 绕完: 导线从左侧重现 */

/* 复位所有状态机 */
static void Reset_State(void)
{
    HexDia_Flag = 0; HexDia_Timer = 0;
    Cross_Stage = 0; Cross_Timer = 0;
    Track_Straight = 0; Track_Steer_Bias = 0; Track_Speed_Ratio = 1.0f;
    Lost_State = 0; Lost_Timer = 0;
    Sharp_State = 0; Sharp_Timer = 0; Sharp_ExitCnt = 0;
    D_Filt = 0.0f; Error = 0.0f; Error_Last = 0.0f;
    Error_H = 0.0f; Error_V = 0.0f;
    Menu_CrossCount = 0;
    Menu_Running = 1;
    H_Base = 0.0f; V_Base = 0.0f;
    Yaw_Deg = 0.0f; Yaw_Rate_Dps = 0.0f; Yaw_Valid = 0;
    PID_Apply_Reset();
    Track_StateMachine_Reset();
    ADC_MAX[0] = ADC_MAX[1] = ADC_MAX[2] = ADC_MAX[3] = SIM_MAX;
    ADC_MIN[0] = ADC_MIN[1] = ADC_MIN[2] = ADC_MIN[3] = SIM_MIN;
    ADC_Peak_Reset();
}

/* 模拟"车高变化 + 标定跟随": 信号整体衰减 k 倍时, 量程也按 k 缩放。
 * 真实系统里这件事由【自动标定】完成(MAX 跟随实际峰值),
 * 这是"按各自峰值归一化"的元素判据能在不同高度都工作的前提。 */
static void Set_Range(float k)
{
    uint16_t mx = (uint16_t)((float)SIM_MIN + (float)(SIM_MAX - SIM_MIN) * k);
    ADC_MAX[0] = ADC_MAX[1] = ADC_MAX[2] = ADC_MAX[3] = mx;
    ADC_MIN[0] = ADC_MIN[1] = ADC_MIN[2] = ADC_MIN[3] = SIM_MIN;
}

/* 让横置/竖置基线收敛到"直道巡航"电平 */
static void Prime_Base(void)
{
    int i;
    for (i = 0; i < 2000; i++)     /* 2000 周期 = 10 秒 */
    {
        STRAIGHT(1.0f);
        Track_Judge();
    }
}

/* ================= S1 误差与距离无关 ================= */
static void S1_Error_Distance_Independent(void)
{
    float e10, e20, e40;
    int i;

    printf("\n[S1] 差比和误差: 同一形状、不同距离 -> 误差应一致\n");
    Reset_State();

    for (i = 0; i < 30; i++) { OFF_LEFT(1.0f); }
    e10 = Error;
    for (i = 0; i < 30; i++) { OFF_LEFT(0.25f); }
    e20 = Error;
    for (i = 0; i < 30; i++) { OFF_LEFT(0.0625f); }
    e40 = Error;

    printf("  10cm: 横置 L=85 R=5, 竖置 ML=3 MR=0   Error=%+.2f\n", e10);
    printf("  20cm: 同形状整体衰减到 25%%            Error=%+.2f\n", e20);
    printf("  40cm: 同形状整体衰减到 6.25%%          Error=%+.2f\n", e40);
    CHECK(fabsf(e10 - e20) < 1.0f, "10cm 与 20cm 误差应一致: %.2f vs %.2f", e10, e20);
    CHECK(fabsf(e10 - e40) < 1.0f, "10cm 与 40cm 误差应一致: %.2f vs %.2f", e10, e40);
    CHECK(e10 > 10.0f, "车偏左(左横置更强)时误差应为正: %.2f", e10);
    /* 80/20 加权: 纯横置偏摆时竖置无信号, 误差上限就是 80 */
    CHECK(e10 <= 80.5f, "纯横置偏摆时误差不应超过 80(竖置无贡献): %.2f", e10);
}

/* ================= S2 十字路口: 三种距离 ================= */
static void S2_Cross_At_Distances(void)
{
    float k_list[3];
    const char *names[3];
    int i, j;

    printf("\n[S2] 十字路口识别(靠竖置电感, 三个距离)\n");
    k_list[0] = 1.00f;  names[0] = "10cm";
    k_list[1] = 0.25f;  names[1] = "20cm";
    k_list[2] = 0.0625f;names[2] = "40cm";

    for (i = 0; i < 3; i++)
    {
        float k = k_list[i];
        Reset_State();
        Set_Range(k);
        Prime_Base();

        for (j = 0; j < 30; j++) { CROSS(1.0f); Track_Judge(); }

        /* 驶出路口, 让计数落下 */
        for (j = 0; j < 40; j++) { STRAIGHT(1.0f); Track_Judge(); }
        printf("  %s(k=%.4f) 横置 %3d/%3d 竖置 %3d/%3d  V场强=%d 十字计数=%d\n",
               names[i], k, ADC_L, ADC_R, ADC_ML, ADC_MR,
               (int)ADC_V_StrengthF, Menu_CrossCount);
        CHECK(Menu_CrossCount == 1, "%s 十字路口应计 1 次, 实际 %d", names[i], Menu_CrossCount);
    }
}

/* ================= S2b 任何路口都只直行, 且绝不自动停车 =================
 * 需求: 十字路口、起点线、折返线一律"保持直行通过";
 *       不再计圈, 不存在"跑满圈数自动停车", 停车只能靠按 KEY4。
 * 这里锁死: ① 路口期间 Track_Straight=1 ② 跑再多路口 Menu_Running 恒为 1。 */
static void S2b_Cross_Always_Straight(void)
{
    int j, m;
    float k = 1.0f;

    printf("\n[S2b] 路口一律只直行 + 无自动停车\n");

    /* 窄路口: 20 周期 */
    Reset_State();
    Prime_Base();
    for (j = 0; j < 20; j++) { CROSS(k); Track_Judge(); }
    printf("  窄路口(20周期): 十字=%d 直行=%d\n", Menu_CrossCount, Track_Straight);
    CHECK(Menu_CrossCount == 1, "窄路口应计 1 次十字, 实际 %d", Menu_CrossCount);
    CHECK(Track_Straight == 1,  "窄路口应保持直行");
    for (j = 0; j < 60; j++) { STRAIGHT(k); Track_Judge(); }
    CHECK(Track_Straight == 0,  "驶出路口后应解除直行");

    /* 宽起点线(70 周期): 同样只直行, 不做任何计圈 */
    Reset_State();
    Prime_Base();
    for (j = 0; j < 70; j++) { CROSS(k); Track_Judge(); }
    printf("  宽起点(70周期): 十字=%d 直行=%d Running=%d\n",
           Menu_CrossCount, Track_Straight, Menu_Running);
    CHECK(Track_Straight == 1, "宽起点线也应保持直行");
    CHECK(Menu_Running == 1,   "路口不应改变运行状态, 实际 %d", Menu_Running);

    /* 连过 10 个路口: 运行状态必须始终是 1(不存在自动停车) */
    Reset_State();
    Prime_Base();
    for (j = 0; j < 10; j++)
    {
        for (m = 0; m < 20; m++) { CROSS(k);    Track_Judge(); }
        for (m = 0; m < 40; m++) { STRAIGHT(k); Track_Judge(); }
    }
    printf("  连过 10 个路口: 十字=%d Running=%d (应恒为 1)\n",
           Menu_CrossCount, Menu_Running);
    CHECK(Menu_Running == 1, "无论过多少路口都不许自动停车, 实际 Running=%d", Menu_Running);
}

/* ================= S3 不该误判的情况 ================= */
static void S3_No_False_Positive(void)
{
    int i;
    float k = 1.0f;

    printf("\n[S3] 直线 / 直角弯 / 普通右弯 不应被误判\n");

    /* 直道居中 */
    Reset_State();
    Prime_Base();
    for (i = 0; i < 60; i++) { STRAIGHT(k); Track_Judge(); }
    printf("  直道居中  横置 %2d/%2d 竖置 %2d/%2d  十字=%d 环=%d\n",
           ADC_L, ADC_R, ADC_ML, ADC_MR, Menu_CrossCount, HexDia_Flag);
    CHECK(Menu_CrossCount == 0, "直道不应判成十字");
    CHECK(HexDia_Flag == 0, "直道不应判成环");

    /* 直角弯: 竖置也抬头, 但横置明显偏一侧 -> 必须靠对称性排除 */
    Reset_State();
    Prime_Base();
    for (i = 0; i < 60; i++) { CORNER90(k); Track_Judge(); }
    printf("  直角弯    横置 %2d/%2d 竖置 %2d/%2d  十字=%d\n",
           ADC_L, ADC_R, ADC_ML, ADC_MR, Menu_CrossCount);
    CHECK(Menu_CrossCount == 0, "直角弯不应判成十字(横置左右不对称)");

    /* 普通右弯: 横置强右偏, 但竖置始终接近 0 */
    Reset_State();
    Prime_Base();
    for (i = 0; i < 60; i++) { CURVE_R(k); Track_Judge(); }
    printf("  普通右弯  横置 %2d/%2d 竖置 %2d/%2d  环=%d\n",
           ADC_L, ADC_R, ADC_ML, ADC_MR, HexDia_Flag);
    CHECK(HexDia_Flag == 0, "普通右弯不应判成六角菱形环(竖置没抬头)");

    /* 六角菱形环入口: 强右偏 + 竖置抬头 */
    Reset_State();
    Prime_Base();
    for (i = 0; i < 60; i++) { HEX_ENTRY(k); Track_Judge(); }
    printf("  环入口    横置 %2d/%2d 竖置 %2d/%2d  环=%d\n",
           ADC_L, ADC_R, ADC_ML, ADC_MR, HexDia_Flag);
    CHECK(HexDia_Flag >= 1, "六角菱形环入口应被识别, 实际 %d", HexDia_Flag);
}

/* ================= S4 六角菱形环在三个距离下 ================= */
static void S4_HexDia_At_Distances(void)
{
    float k_list[3];
    const char *names[3];
    int i, j;

    printf("\n[S4] 六角菱形环入口识别(三个距离)\n");
    k_list[0] = 1.00f;  names[0] = "10cm";
    k_list[1] = 0.25f;  names[1] = "20cm";
    k_list[2] = 0.0625f;names[2] = "40cm";

    for (i = 0; i < 3; i++)
    {
        float k = k_list[i];
        Reset_State();
        Set_Range(k);
        Prime_Base();
        for (j = 0; j < 60; j++) { HEX_ENTRY(1.0f); Track_Judge(); }
        printf("  %s(k=%.4f) 横置 %2d/%2d 竖置 %2d/%2d  环=%d\n",
               names[i], k, ADC_L, ADC_R, ADC_ML, ADC_MR, HexDia_Flag);
        CHECK(HexDia_Flag >= 1, "%s 六角菱形环入口未识别", names[i]);
    }
}

/* ================= S4b 六角菱形环闭环行为 ================= */
static void S4b_HexDia_ClosedLoop(void)
{
    int i;
    float k = 1.0f;
    int16_t bias_in, bias_keep, bias_search;
    float spd_in, spd_keep, spd_search;

    printf("\n[S4b] 六角菱形环: 进环 / 环内丢线找线 / 左侧重现出环\n");

    Reset_State();
    Prime_Base();

    /* 1) 进环: 强制右转 + 降速 (入口确认 10 周期, 进环最长 40 周期) */
    for (i = 0; i < 15; i++) { HEX_ENTRY(k); Track_Judge(); }
    bias_in = Track_Steer_Bias;
    spd_in = Track_Speed_Ratio;
    printf("  进环      状态=%d 偏置=%d 速度系数=%.2f\n", HexDia_Flag, bias_in, spd_in);
    CHECK(HexDia_Flag == 1, "进环阶段应为状态 1, 实际 %d", HexDia_Flag);
    CHECK(bias_in < 0, "进环应为右转(负偏置): %d", bias_in);
    CHECK(spd_in < 1.0f, "进环应降速: %.2f", spd_in);

    /* 2) 环内: 轻偏置维持方向 */
    for (i = 0; i < 60; i++) { HEX_INSIDE(k); Track_Judge(); }
    bias_keep = Track_Steer_Bias;
    spd_keep = Track_Speed_Ratio;
    printf("  环内      状态=%d 偏置=%d 速度系数=%.2f\n", HexDia_Flag, bias_keep, spd_keep);
    CHECK(HexDia_Flag == 2, "应进入环内状态 2, 实际 %d", HexDia_Flag);

    /* 3) 环内丢线(尖角处压不到导线): 转更多去找线, 速度再降 */
    for (i = 0; i < 10; i++) { Feed(0, 0, 0, 0, k); Track_Judge(); }
    bias_search = Track_Steer_Bias;
    spd_search = Track_Speed_Ratio;
    printf("  环内丢线  偏置=%d 速度系数=%.2f (应比环内转更多、更慢)\n", bias_search, spd_search);
    CHECK(HexDia_Flag == 2, "丢线时不应退出环: %d", HexDia_Flag);
    CHECK(bias_search < bias_keep, "丢线时偏置应更大(转更多): %d vs %d", bias_search, bias_keep);
    CHECK(spd_search < spd_keep, "丢线时应更慢: %.2f vs %.2f", spd_search, spd_keep);

    /* 4) 绕完一圈: 导线从左侧重新出现 */
    for (i = 0; i < 10; i++) { HEX_BACK(k); Track_Judge(); }
    printf("  左侧重现  状态=%d (应为 3 出环)\n", HexDia_Flag);
    CHECK(HexDia_Flag == 3, "导线从左侧重现应转入出环状态, 实际 %d", HexDia_Flag);

    /* 5) 回中后回到待命 */
    for (i = 0; i < 80; i++) { STRAIGHT(k); Track_Judge(); }
    printf("  回中后    状态=%d\n", HexDia_Flag);
    CHECK(HexDia_Flag == 0, "回中后应回到待命状态, 实际 %d", HexDia_Flag);

    /* 十字期间禁止新进环识别, 但活动中的环状态必须继续计时并退出。 */
    Reset_State();
    Cross_Stage = 1;
    for (i = 0; i < 15; i++) { HEX_ENTRY(k); HexDia_Judge(); }
    CHECK(HexDia_Flag == 0, "十字状态下不得新进入六角环");

    HexDia_Flag = 2;
    HexDia_Timer = 219;
    STRAIGHT(k);
    HexDia_Judge();
    CHECK(HexDia_Flag == 3, "十字检测不能冻结已进入环的超时出环流程");
    for (i = 0; i < 11; i++) HexDia_Judge();
    CHECK(HexDia_Flag == 0, "活动环流程应能在十字检测期间完成出环, 状态=%d Error=%+.2f",
          HexDia_Flag, Error);
}

/* ================= S5 丢线保护 ================= */
static void S5_Lost_Line(void)
{
    int i;
    float k = 1.0f;

    printf("\n[S5] 丢线保护\n");
    Reset_State();
    for (i = 0; i < 20; i++) { Feed(85, 3, 0, 5, k); PID_LostLine_Check(); PID_Calculate(); }
    {
        int16_t before = 0;
        int16_t after  = 0;
        for (i = 0; i < 20; i++) before = PID_Apply(Speed_Delta);
        printf("  在线时 Error=%+.2f 差速=%d\n", Error, before);
        for (i = 0; i < 20; i++) { Feed(0, 0, 0, 0, k); PID_LostLine_Check(); PID_Calculate(); }
        {
            /* 差速现在有斜率限制, 单拍到不了 ±900, 多跑几拍看稳态 */
            for (i = 0; i < 20; i++) after = PID_Apply(Speed_Delta);
            printf("  丢线后 场强=%d 丢线状态=%d 差速=%d (应沿最后方向找线)\n",
                   ADC_Strength, Lost_State, after);
            CHECK(Lost_State == 1, "全丢线应置丢线状态");
            CHECK(after == 900 || after == -900, "丢线应按最后方向给固定找线差速: %d", after);
        }
        {
            int16_t slow = PID_Speed_Schedule(Speed_Basic);
            printf("  丢线时调度后基础速度=%d (原 %d, 应相等: 全程恒速)\n", slow, Speed_Basic);
            CHECK(slow == Speed_Basic, "SPEED_SCHEDULE_EN=0 时应全程恒速, 实际 %d", slow);
        }
    }
}

/* ================= S6 急弯保持(锯齿线) ================= */
static void S6_Sharp_Turn(void)
{
    int i;
    float k = 1.0f;
    int16_t d1 = 0, d2 = 0;

    printf("\n[S6] 急弯(锯齿线/直角弯)保持\n");
    Reset_State();
    /* 导线跑到最左边 -> 误差饱和 */
    for (i = 0; i < 10; i++) { Feed(100, 10, 0, 0, k); PID_Sharp_Check(); PID_Calculate(); }
    printf("  饱和时 Error=%+.2f 急弯状态=%d 方向=%d\n", Error, Sharp_State, Sharp_Dir);
    CHECK(Sharp_State == 1, "误差饱和应进入急弯保持");

    d1 = PID_Apply(Speed_Delta);
    /* 锯齿线: 误差反向但还没回中, 保持方向不许抽动 */
    Feed(0, 0, 10, 100, k);
    PID_Sharp_Check();
    PID_Calculate();
    d2 = PID_Apply(Speed_Delta);
    printf("  反向瞬间 原始差速=%d 保持后差速=%d (饱和时差速=%d, 符号应保持一致)\n",
           Speed_Delta, d2, d1);
    CHECK((d1 > 0) == (d2 > 0), "急弯保持期间转向方向不应反转: %d vs %d", d1, d2);

    Lost_State = 1;
    Last_Dir = (int8_t)-Sharp_Dir;
    d2 = PID_Apply(0);
    CHECK((d2 > 0) == (Sharp_Dir > 0), "急弯锁定期间丢线搜索不能抢走锁定方向");
    Lost_State = 0;

    /* 两个电感虽然相等, 但均低于 400 时仍未可靠重新压线, 不能出弯。 */
    for (i = 0; i < 10; i++) { FeedRaw(300, 200, 200, 300); PID_Sharp_Check(); }
    CHECK(Sharp_State == 1, "横置电感低于 400 时不能退出急弯");

    FeedRaw(500, 200, 200, 600); PID_Sharp_Check();
    CHECK(Sharp_State == 1, "L/R 差值超过容差时不能退出急弯");

    /* 达到阈值后连续确认 2 周期即退出, 不再直行等待 */
    FeedRaw(500, 200, 200, 520); PID_Sharp_Check();
    CHECK(Sharp_State == 1, "出弯判据第 1 周期仍应保持转向");
    FeedRaw(500, 200, 200, 520); PID_Sharp_Check();
    CHECK(Sharp_State == 0, "两路横置电感均不低于 400 且接近时应立即退出");

    /* 十字状态下禁止进入或保持急弯状态, 路口优先直行。 */
    Reset_State();
    Cross_Stage = 1;
    Track_Straight = 1;
    for (i = 0; i < 10; i++) { Feed(100, 10, 0, 0, k); PID_Sharp_Check(); }
    CHECK(Sharp_State == 0, "十字直行状态不得进入急弯锁定");

    Cross_Stage = 0;
    Track_Straight = 0;
    for (i = 0; i < 3; i++) { Feed(100, 10, 0, 0, k); PID_Sharp_Check(); }
    CHECK(Sharp_State == 1, "驶出十字后应能识别紧随其后的直角弯");
}

/* ================= S7 弯道降速调度 ================= */
static void S7_Speed_Schedule(void)
{
    int i;
    float k = 1.0f;

    printf("\n[S7] 恒速(弯道不再降速)\n");
    Speed_Basic = 700;      /* mm/s */
    Reset_State();
    for (i = 0; i < 20; i++) { STRAIGHT(k); }
    printf("  直线   Error=%+6.2f -> 速度 %d\n", Error, PID_Speed_Schedule(Speed_Basic));
    {
        int16_t straight = PID_Speed_Schedule(Speed_Basic);
        for (i = 0; i < 20; i++) { Feed(80, 5, 0, 8, k); }  /* 偏得多(弯道) */
        printf("  大偏差 Error=%+6.2f -> 速度 %d\n", Error, PID_Speed_Schedule(Speed_Basic));
        {
            int16_t curve = PID_Speed_Schedule(Speed_Basic);
            CHECK(curve == straight, "全程恒速: 弯道不该降速 %d vs %d", curve, straight);
        }
    }
}

/* ================= S8 峰值保持与饱和检测 ================= */
static void S8_Peak_And_Saturation(void)
{
    printf("\n[S8] 峰值保持 / 饱和检测(调电位器用)\n");

    Reset_State();

    /* 先在直道上跑一段: 横置约 2000, 竖置约 200 */
    FeedRaw(2000, 200, 200, 1950);
    ADC_Calibrate();
    printf("  直道: 横置峰值 %4d/%4d 竖置峰值 %4d/%4d  SAT=%d\n",
           ADC_Peak[3], ADC_Peak[0], ADC_Peak[2], ADC_Peak[1], ADC_SatFlag);
    CHECK(ADC_SatFlag == 0, "直道不应饱和");
    CHECK(ADC_Peak[3] == 2000, "峰值应记住 2000, 实际 %d", ADC_Peak[3]);

    /* 推到十字路口: 峰值 3400, 落在推荐区间 3200~3600 */
    FeedRaw(2100, 3400, 3380, 2050);
    ADC_Calibrate();
    printf("  十字: 横置峰值 %4d/%4d 竖置峰值 %4d/%4d  SAT=%d  (目标 3200~3600)\n",
           ADC_Peak[3], ADC_Peak[0], ADC_Peak[2], ADC_Peak[1], ADC_SatFlag);
    CHECK(ADC_Peak[2] == 3400, "峰值应更新到 3400, 实际 %d", ADC_Peak[2]);
    CHECK(ADC_SatFlag == 0, "3400 未超 3700, 不应报饱和");

    /* 增益过大: 峰值顶到 4095 -> 必须报饱和 */
    FeedRaw(2200, 4095, 4060, 2100);
    ADC_Calibrate();
    printf("  过增益: 竖置 %4d/%4d  SAT=0x%X (bit2/bit1 应置位)  SatSeen=%d\n",
           ADC_Value[2], ADC_Value[1], ADC_SatFlag, ADC_SatSeen);
    CHECK((ADC_SatFlag & 0x06) == 0x06, "顶到 4095 必须报饱和, 实际 0x%X", ADC_SatFlag);
    CHECK(ADC_SatSeen == 1, "饱和粘滞标志应置位");

    /* 顶到 4095 时归一化必须限在 100, 不能溢出 */
    printf("  饱和时归一化: 竖置 %d/%d (应 <=100)\n", ADC_ML, ADC_MR);
    CHECK(ADC_ML <= 100 && ADC_MR <= 100, "饱和时归一化应限幅到 100");

    /* 清零峰值后重新开始 */
    ADC_Peak_Reset();
    printf("  清零后峰值: %d/%d/%d/%d\n",
           ADC_Peak[3], ADC_Peak[0], ADC_Peak[2], ADC_Peak[1]);
    CHECK(ADC_Peak[3] == 0 && ADC_Peak[2] == 0, "清零后峰值应为 0");
}

/* ================= S9 赛道状态机 ================= */
/* 连续跑 n 个 5ms 周期 */
static void Cycle(int n, int16_t nL, int16_t nML, int16_t nMR, int16_t nR)
{
    int i;
    for (i = 0; i < n; i++)
    {
        Feed(nL, nML, nMR, nR, 1.0f);
        PID_Sharp_Check();
        PID_Calculate();
        Track_Judge();
    }
}

static void S9_Track_State(void)
{
    int i;

    printf("\n[S9] 赛道状态机分类\n");

    Reset_State();
    Cycle(60, 55, 2, 2, 55);
    printf("  直道   -> 状态 %d (期望 0)  系数 %.2f\n", Track_State, Track_Speed_Ratio);
    CHECK(Track_State == TRK_STRAIGHT, "直道应为 0, 实际 %d", Track_State);

    Reset_State();
    Cycle(60, 75, 2, 2, 35);
    printf("  小弯   -> 状态 %d (期望 1)\n", Track_State);
    CHECK(Track_State == TRK_CURVE, "小弯应为 1, 实际 %d", Track_State);

    /* 半圆弧: 误差稳在 45~80 且持续 400ms 以上 */
    Reset_State();
    Cycle(200, 70, 3, 3, 20);
    printf("  半圆弧 -> 状态 %d (期望 6)  E=%+.1f 系数 %.2f\n",
           Track_State, Error, Track_Speed_Ratio);
    CHECK(Track_State == TRK_ARC, "半圆弧应为 6, 实际 %d", Track_State);
    CHECK(Track_Speed_Ratio < 0.95f, "圆弧应略降速");

    /* 直角弯: 误差直接顶到 90 以上 */
    Reset_State();
    Cycle(60, 95, 5, 5, 2);
    printf("  直角弯 -> 状态 %d (期望 2 或 1)\n", Track_State);
    CHECK((Track_State == TRK_SHARP) || (Track_State == TRK_CURVE),
          "直角弯判据异常: %d", Track_State);

    /* S弯/两条很近的直道: 每 400ms 换向 */
    Reset_State();
    Cycle(80, 70, 3, 3, 20);
    Cycle(80, 20, 3, 3, 70);
    Cycle(80, 70, 3, 3, 20);
    printf("  S弯    -> 状态 %d (期望 3) 系数 %.2f D倍数 %.2f\n",
           Track_State, Track_Speed_Ratio, Track_Kd_Gain);
    CHECK(Track_State == TRK_S_CURVE, "S弯应为 3, 实际 %d", Track_State);
    CHECK(Track_Kd_Gain > 1.2f, "S弯应加大 D 阻尼: %.2f", Track_Kd_Gain);

    /* 锯齿: 每 60ms 换向 */
    Reset_State();
    for (i = 0; i < 3; i++)
    {
        Cycle(12, 75, 3, 3, 15);
        Cycle(12, 15, 3, 3, 75);
    }
    printf("  锯齿   -> 状态 %d (期望 7) 系数 %.2f D倍数 %.2f\n",
           Track_State, Track_Speed_Ratio, Track_Kd_Gain);
    CHECK(Track_State == TRK_ZIGZAG, "锯齿应为 7, 实际 %d", Track_State);
    CHECK(Track_Speed_Ratio < 0.60f, "锯齿应明显降速: %.2f", Track_Speed_Ratio);

    /* 十字 */
    Reset_State();
    Cycle(60, 50, 88, 88, 50);
    printf("  十字   -> 状态 %d (期望 4)\n", Track_State);
    CHECK(Track_State == TRK_CROSS, "十字应为 4, 实际 %d", Track_State);

    Track_StateMachine_Reset();
    CHECK(Track_State == TRK_STRAIGHT, "复位后应回到直道");
    CHECK(Track_Kd_Gain == 1.0f, "复位后 D 倍数应回到 1.0");
}

/* ================= S10 差速整形 + 无陀螺仪航向 ================= */
static void S10_Delta_Shape_And_Yaw(void)
{
    int i;
    int16_t d;

    printf("\n[S10] 差速斜率/限幅 + 十字航向锁定 + 急弯持续锁方向\n");

    /* Lag=0 在低速也必须立即使用当前误差; 非零 Lag 的低速保护仍受上限约束。 */
    Reset_State();
    Speed_Basic = 0;
    Lag_Ratio = 0.0f;
    Error_H = 20.0f;
    PID_Calculate();
    CHECK(Lag_Cycles == 0, "Lag=0 时低速分支也不得引入延迟, 实际 %u 周期", Lag_Cycles);
    Lag_Ratio = 0.30f;
    PID_Calculate();
    CHECK(Lag_Cycles == 90, "非零 Lag 的低速延迟仍应受 90 周期上限约束, 实际 %u", Lag_Cycles);
    Speed_Basic = 51;
    Lag_Ratio = 1.0f;
    PID_Calculate();
    CHECK(Lag_Cycles == 90, "低速高比例 Lag 应先饱和再转整数, 实际 %u", Lag_Cycles);
    Lag_Ratio = 0.30f;
    Speed_Basic = 5000;

    /* 1) 斜率: 一拍不许从 0 跳到满舵 */
    Reset_State();
    d = PID_Apply(3000);
    printf("  满舵请求: 第一拍=%d (应 <=180)\n", d);
    CHECK(d > 0 && d <= 180, "差速应受斜率限制, 实际 %d", d);
    for (i = 0; i < 20; i++) d = PID_Apply(3000);
    printf("  满舵请求: 稳态=%d (应 1000)\n", d);
    CHECK(d == 1000, "稳态应被限幅到 1000, 实际 %d", d);

    /* 2) 十字直行: 没有编码器航向时, 老实压成 0 */
    Reset_State();
    Yaw_Valid = 0;
    Track_Straight = 1;
    for (i = 0; i < 20; i++) d = PID_Apply(900);
    printf("  十字(编码器无效) 差速=%d (应 0)\n", d);
    CHECK(d == 0, "十字直行应把差速压到 0, 实际 %d", d);
    Error_H = 20.0f;
    for (i = 0; i < 5; i++) d = PID_Apply(0);
    CHECK(d > 0 && d <= 120, "十字入口轻微偏心应只产生有限的对中修正, 实际 %d", d);
    Track_Straight = 0;

    /* 3) 十字直行: 有航向时把偏航拉回来(不能让它斜穿) */
    Reset_State();
    Yaw_Valid = 1;
    Track_Straight = 1;
    Yaw_Deg = 0.0f;
    PID_Apply(0);               /* 建立航向基准 */
    Yaw_Deg = 10.0f;            /* 车头已经往右偏 10 度 */
    for (i = 0; i < 5; i++) d = PID_Apply(0);
    printf("  十字(偏航+10度) 差速=%d (应为负: 往左修)\n", d);
    CHECK(d < 0, "偏航时应反向修正, 实际 %d", d);
    Track_Straight = 0;

    /* 4) 直角弯在满足电感出弯条件前持续锁方向, 不因估算角度提前停转。 */
    Reset_State();
    Yaw_Valid = 1;
    Sharp_State = 1;
    Sharp_Dir = 1;
    Yaw_Deg = 0.0f;
    for (i = 0; i < 6; i++) d = PID_Apply(0);
    printf("  直角弯 0度: 差速=%d (应 >=380 保底)\n", d);
    CHECK(d >= 380, "刚进弯应有保底转向量, 实际 %d", d);

    Yaw_Deg = 45.0f;
    for (i = 0; i < 3; i++) d = PID_Apply(0);
    printf("  直角弯 45度: 差速=%d\n", d);
    CHECK(d < 900, "半程不应顶到限幅");

    Yaw_Deg = 90.0f;
    for (i = 0; i < 20; i++) d = PID_Apply(0);
    printf("  直角弯 90度: 差速=%d (仍应锁方向)\n", d);
    CHECK(d >= 450, "未满足电感出弯条件前应继续转向, 实际 %d", d);

    Yaw_Deg = 120.0f;
    for (i = 0; i < 20; i++) d = PID_Apply(0);
    printf("  转过 120度: 差速=%d (不允许反向)\n", d);
    CHECK(d >= 450, "角度超过辅助目标后仍需锁方向, 实际 %d", d);

    Sharp_Timer = 10;
    FeedRaw(500, 200, 200, 520);
    PID_Sharp_Check();
    CHECK(Sharp_State == 1, "达到电感出弯条件第 1 周期仍保持锁定");
    FeedRaw(500, 200, 200, 520);
    PID_Sharp_Check();
    CHECK(Sharp_State == 0, "电感满足阈值并确认(2 周期)后立即出弯");

    Sharp_State = 0;
    Yaw_Valid = 0;
    Yaw_Deg = 0.0f;
}

/* ================= S12 弯道分级 / 邻线抑制 / 转够了实时收油 ================= */
static void S12_Curve_Grade(void)
{
    int i;

    printf("\n[S12] 按角度分级 + 邻线瞬态抑制 + 转弯过程实时判断\n");
    Reset_State();

    /* 1) 旁边那根线只扫过一拍: 不该确认成弯, 也不该大范围转弯 */
    CURVE_R(1.0f);
    PID_Calculate();
    printf("  单拍大误差: G=%d C=%d d=%d\n", Curve_Grade_Now, Curve_Conf_Now, Speed_Delta);
    CHECK(Curve_Grade_Now == 0, "邻线瞬态不应确认成弯, G=%d", Curve_Grade_Now);
    CHECK(abs(Speed_Delta) <= 350, "未确认的大误差差速应 <=350, 实际 %d", Speed_Delta);

    /* 2) 同方向持续 = 真弯: 分级确认并放开差速 */
    Reset_State();
    for (i = 0; i < 10; i++) { CURVE_R(1.0f); PID_Calculate(); }
    printf("  持续右弯: G=%d T=%d d=%d\n", Curve_Grade_Now, Curve_Turn_Now, Speed_Delta);
    CHECK(Curve_Grade_Now >= 2, "持续大误差应判 90/110 级, G=%d", Curve_Grade_Now);
    CHECK(abs(Speed_Delta) > 350, "确认后的弯应放开差速, 实际 %d", Speed_Delta);

    /* 3) 误差回落 = 转够了, 转弯中状态当场释放 */
    for (i = 0; i < 10; i++) { STRAIGHT(1.0f); PID_Calculate(); }
    printf("  回中: G=%d T=%d d=%d\n", Curve_Grade_Now, Curve_Turn_Now, Speed_Delta);
    CHECK(Curve_Turn_Now == 0, "误差回落应判定转够并收油");
    CHECK(abs(Speed_Delta) < 350, "回中后不该还有大差速, 实际 %d", Speed_Delta);
}

int main(void)
{
    printf("==============================================\n");
    printf(" 电磁循迹算法仿真 (复合 2横+2竖 布局版)\n");
    printf("==============================================\n");

    S1_Error_Distance_Independent();
    S2_Cross_At_Distances();
    S2b_Cross_Always_Straight();
    S3_No_False_Positive();
    S4_HexDia_At_Distances();
    S4b_HexDia_ClosedLoop();
    S5_Lost_Line();
    S6_Sharp_Turn();
    S7_Speed_Schedule();
    S8_Peak_And_Saturation();
    S9_Track_State();
    S10_Delta_Shape_And_Yaw();
    S12_Curve_Grade();

    printf("\n==============================================\n");
    printf(" checks=%d  failures=%d  -> %s\n", g_checks, g_fail, g_fail ? "FAIL" : "ALL PASS");
    printf("==============================================\n");
    return g_fail ? 1 : 0;
}