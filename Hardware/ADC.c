#include "Common_headfile.h"

/* ============================================================
 *  电磁采集与误差计算 —— 复合(十字)型电感布局版
 *
 *  传感器布局(2 横 + 2 竖):
 *    两侧【横置】 L(PA7) / R(PA4)  —— 循迹主力, 权重 80%
 *    中间【竖置】 ML(PA6)/ MR(PA5) —— 特殊元素探测器, 权重 20%
 *
 *  为什么横置与竖置"不能互比":
 *    横置线圈轴线垂直于导线, 导线从侧方经过时耦合最强 -> 直道上读数就很大
 *    (调好后 1500~2500), 是判断左右偏差的主力。
 *    竖置线圈轴线竖直, 直道上导线的磁场几乎全是水平方向, 耦合接近 0
 *    -> 直道上读数很小(接近 0 属正常现象)。只有到十字/环岛这类
 *    "两根导线交叉、磁场矢量叠加" 的地方才显著起来。
 *
 *    两者电平量纲完全不同, 因此本文件遵守两条铁律:
 *      a) 归一化【各自用自己的峰值】(ADC_MAX[i] 逐通道), 不要求横竖一致;
 *      b) 误差与判据【各类型先各自归一化, 再加权合成】, 绝不做绝对值互比。
 *
 *  通道顺序: ADC_Value[0]=PA4=R [1]=PA5=MR [2]=PA6=ML [3]=PA7=L
 * ============================================================ */

volatile uint16_t ADC_TempDat[4];
volatile uint16_t ADC_Dat[20][4];
volatile uint16_t ADC_Value[4];
volatile int16_t ADC_L = 0, ADC_ML = 0, ADC_MR = 0, ADC_R = 0;
/* 浮点归一化值: 8 位整数在远距离(归一化值只有个位数)量化误差很大,
 * 误差计算必须用浮点, 整数版本只给显示和状态机用 */
volatile float ADC_Lf = 0.0f, ADC_MLf = 0.0f, ADC_MRf = 0.0f, ADC_Rf = 0.0f;
volatile float Error = 0.0f, Error_Last = 0.0f, Error_Raw = 0.0f;
/* 两路【独立】误差(各带低通), 由 PID 层分别算 PD 再融合:
 *   Error_H: 左右横置(L/R)的差比和 —— 直道主力, 反映"车离线中心偏多少"
 *   Error_V: 中左/中右竖置(ML/MR)的差比和 —— 弯道信号, 竖置指向性强,
 *            前方有弯/交叉时才抬头, 邻线从旁边经过时几乎不响应 */
volatile float Error_H = 0.0f;
volatile float Error_V = 0.0f;

uint16_t ADC_MAX[4] = {0, 0, 0, 0};
uint16_t ADC_MIN[4] = {0, 0, 0, 0};

/* 误差权重: A=横置总权重微调, B=竖置总权重微调 (默认 1.0, 一般不用改)
 * C 保留但已不参与运算(旧版的分母修正项, 复合布局下无意义) */
float A = 1.0f;
float B = 1.0f;
float C = 0.0f;

/* ------------------------------------------------------------
 *  布局权重: 横置 72.7% / 竖置 27.3%
 *  W_OUT : W_IN = 4 : 1.5 -> 4/5.5 ≈ 73%
 *  横置仍是循迹主力, 但竖置比重已从 20% 提到 27%:
 *  竖置指向性强、不受邻线干扰, 弯道/交叉时给合成误差更大的话语权,
 *  弯道降速和状态分类都会更早响应。
 *  注意: 直道上竖置接近 0 且被 V_ACT_MIN 旁路, 这个权重不影响直线。
 * ------------------------------------------------------------ */
/* 中右竖置通道硬件增益偏大且无法调节, 控制与显示统一按实际值的 75% 使用。 */
#define MR_GAIN_SCALE  0.75f

/* 直道零点自适应: 消除左右横置的安装/增益不对称(车一直往一边偏的根因)。
 * 只在"竖置 <500(直道) + 横置有信号 + 误差本身很小"时学习,
 * 弯道上误差大, 不学习, 所以不会把真实转向吃掉。 */
#define ZERO_LEARN       0.0015f  /* 每 5ms 学习 0.15%, 约 3 秒收敛(太快会掩盖真实偏差) */
#define ZERO_GATE        5.0f     /* 只学习 ±5 以内: 更大的偏差是车真的偏了, 必须由 PD 修 */
#define ZERO_LIMIT       8.0f     /* 最多补偿 ±8, 防止把真实位置偏差当成零点学掉 */

volatile float ADC_Zero_H = 0.0f;   /* 横置零点(百分数) */
volatile float ADC_Raw_MR = 0.0f;   /* MR 原始值 ×0.75 */
static uint8_t  s_zero_lock = 0;    /* 1 = 零点已锁定(发车 3 秒后不再学) */
static uint16_t s_zero_ms   = 0;

#define W_OUT 4.0f
#define W_IN  1.5f

/* 误差一阶低通系数(0~1), 越大越灵敏但越抖 */
#define ERROR_LPF 0.5f

/* 各类型参与误差运算的最低门限。
 * 对于竖置，现场实测中间两路在直道上通常只有几十到几百 ADC,
 * 低于 500 时就不再视为弯道特征，避免直道被误判成急弯。
 * 这里作为“直道误差旁路门限”，不直接抹掉它被判定弯道/十字的能力。 */
#define H_ACT_MIN      4.0f    /* 横置 (L+R): 低于此视为丢线, 误差交给丢线保护 */
#define V_RAW_IGNORE  500.0f    /* 中间两个竖置在 500 以下时不参与直道误差 */
#define V_ACT_MIN     18.0f    /* 竖置归一化和低于此 -> 直接不参与 */

/* 硬件峰值上限: 电位器调好后十字/环岛处的峰值应落在 3200~3550,
 * 超过 3700 必须逆时针降增益, 绝对不能顶到 4095(饱和后完全失去区分度)。
 * 门限值统一定义在 ADC.h 的 ADC_SAT_TH_VALUE, 便于菜单与文档引用。 */

/* 总场强(0~100): 横竖加权平均, 反映"离导线有多远" */
volatile uint8_t ADC_Strength = 0;
volatile float   ADC_StrengthF = 0.0f;
/* 横置/竖置分开的场强: 判据必须按类型各自比较(横竖不互比) */
volatile float   ADC_H_StrengthF = 0.0f;   /* 横置平均 (L,R)   */
volatile float   ADC_V_StrengthF = 0.0f;   /* 竖置平均 (ML,MR) */
/* 是否压在线上(总场强高于阈值) */
volatile uint8_t ADC_Online = 0;

/* 自动标定开关: 1=运行中自动跟踪每路的实际最大/最小值 */
uint8_t ADC_AutoCalib = 0;
/* 标定状态: 0=未标定 1=标定中 2=标定完成 */
uint8_t Calib_State = 0;

/* 手动标定持续时长: 按 KEY3 后采集这么久自动锁定, 不用再按一次 */
#define ADC_CALIB_MS   3000
static uint16_t s_calib_ms = 0;

/* 峰值保持: 调电位器时用它读"十字/环岛处的峰值"(见调车手册第二章)。
 * 按菜单 Dist 页的 KEY3 清零。 */
volatile uint16_t ADC_Peak[4] = {0, 0, 0, 0};
/* 本轮是否有通道饱和/顶到上限: bit0..3 对应 4 个通道 */
volatile uint8_t ADC_SatFlag = 0;
/* 是否曾经饱和过(粘滞标志, 给标定页面提示用) */
volatile uint8_t ADC_SatSeen = 0;

void ADC_MyInit(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 采样时间 55.5 周期: 电感调理板输出阻抗低, 239.5 周期纯属浪费
     * (239.5->55.5 可把 5ms 中断里的 ADC 占用从 ~1.7ms 降到 ~0.4ms) */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 4, ADC_SampleTime_55Cycles5);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_NbrOfChannel = 4;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_Init(ADC1, &ADC_InitStructure);

    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_BufferSize = 4;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_TempDat;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1) == SET);
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) == SET);
}

void ADC_GetValue(void)
{
    static uint8_t k = 0;

    DMA_Cmd(DMA1_Channel1, DISABLE);
    DMA_SetCurrDataCounter(DMA1_Channel1, 4);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    while (DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);

    ADC_Dat[k][0] = ADC_TempDat[0];
    ADC_Dat[k][1] = ADC_TempDat[1];
    ADC_Dat[k][2] = ADC_TempDat[2];
    ADC_Dat[k][3] = ADC_TempDat[3];

    if (++k > 19) { k = 0; ADC_Filter(); }

    DMA_ClearFlag(DMA1_FLAG_TC1);
}

/* 去极值平均: 一次遍历求 sum / min / max, 减去一个最大和一个最小后取 18 个平均。
 * 结果与"20 个样本插入排序后取中间 18 个"完全等价, 但不需要排序、
 * 不搬移数据、不破坏采样窗口, 且是无 FPU 的纯整数运算。
 * (参考开源电磁车的滑动平均/去极值抗噪做法, 省 Flash/RAM/CPU。) */
void ADC_Filter(void)
{
    int16_t i, n;
    int32_t sum[4];
    uint16_t vmin[4], vmax[4];

    for (n = 0; n < 4; n++)
    {
        uint16_t v0 = ADC_Dat[0][n];
        sum[n]  = v0;
        vmin[n] = v0;
        vmax[n] = v0;
    }

    for (i = 1; i < 20; i++)
    {
        for (n = 0; n < 4; n++)
        {
            uint16_t v = ADC_Dat[i][n];

            sum[n] += v;
            if (v < vmin[n]) vmin[n] = v;
            if (v > vmax[n]) vmax[n] = v;
        }
    }

    for (n = 0; n < 4; n++)
    {
        ADC_Value[n] = (uint16_t)((sum[n] - vmin[n] - vmax[n]) / 18);
    }
}

/* 单路归一化(浮点): (当前值 - 最小) / (最大 - 最小) * 100
 * 注意: 分母是该通道【自己的】峰值, 横置和竖置各标各的, 互不影响 */
static float ADC_NormF(int16_t index)
{
    float den, val;

    den = (float)(ADC_MAX[index] - ADC_MIN[index]);
    if (den <= 0.0f)
    {
        den = 1.0f;
    }
    val = ((float)ADC_Value[index] - (float)ADC_MIN[index]) / den * 100.0f;
    if (val > 100.0f) val = 100.0f;
    if (val < 0.0f) val = 0.0f;
    return val;
}

void Get_Error(void)
{
    float lr, rr, ml, mr, hsum_r, vsum_r;
    float eh, ev, num, den;

    /* 引脚映射(依据原理图):
     * PA7=ADC1=左横置(L)    -> ADC_Value[3]
     * PA6=ADC2=中左竖置(ML) -> ADC_Value[2]
     * PA5=ADC3=中右竖置(MR) -> ADC_Value[1]   (硬件增益偏大, 统一 ×0.75)
     * PA4=ADC4=右横置(R)    -> ADC_Value[0]
     */
    /* 先减各通道最小值再算差比和: 这样电感离导线变远(整体电平等比变小)时,
     * 误差百分比不变; 直接用原始值会被"非零最小值"拉偏(近处 57%、远处 42%)。 */
    lr = (float)ADC_Value[3] - (float)ADC_MIN[3];
    ml = (float)ADC_Value[2] - (float)ADC_MIN[2];
    mr = ((float)ADC_Value[1] - (float)ADC_MIN[1]) * MR_GAIN_SCALE;
    rr = (float)ADC_Value[0] - (float)ADC_MIN[0];
    if (lr < 0.0f) lr = 0.0f;
    if (ml < 0.0f) ml = 0.0f;
    if (mr < 0.0f) mr = 0.0f;
    if (rr < 0.0f) rr = 0.0f;
    ADC_Raw_MR = mr;

    /* 归一化值只用于显示和场强判断 */
    ADC_Lf  = ADC_NormF(3);
    ADC_MLf = ADC_NormF(2);
    ADC_MRf = ADC_NormF(1) * MR_GAIN_SCALE;
    if (ADC_MRf > 100.0f) ADC_MRf = 100.0f;
    ADC_Rf  = ADC_NormF(0);

    ADC_L  = (int16_t)ADC_Lf;
    ADC_ML = (int16_t)ADC_MLf;
    ADC_MR = (int16_t)ADC_MRf;
    ADC_R  = (int16_t)ADC_Rf;

    ADC_H_StrengthF = (ADC_Lf + ADC_Rf) * 0.5f;
    ADC_V_StrengthF = (ADC_MLf + ADC_MRf) * 0.5f;
    ADC_StrengthF = (W_OUT * ADC_H_StrengthF + W_IN * ADC_V_StrengthF) / (W_OUT + W_IN);
    ADC_Strength = (uint8_t)(ADC_StrengthF + 0.5f);
    ADC_Online = (ADC_Strength >= TRACK_ONLINE_TH) ? 1 : 0;

    Error_Last = Error;

    /* ------------------------------------------------------------
     *  误差改用【原始值差比和】, 不再用归一化值
     *
     *  为什么: 归一化是 (V-MIN)/(MAX-MIN), MAX 一旦标在直道读数上,
     *  直道就顶到 100; 左右两路都 100 -> 误差恒 0。现场表现就是
     *  "离得很近也显示 0、直道一直往一边偏、直角弯看不见"。
     *  差比和与绝对增益无关: 2800/2800 -> 0, 3000/2500 -> 9, 永远有分辨率。
     * ------------------------------------------------------------ */
    hsum_r = lr + rr;
    vsum_r = ml + mr;

    if (hsum_r > 50.0f) eh = (lr - rr) / hsum_r * 100.0f;
    else               eh = 0.0f;

    /* 直道零点: 只在中间两路 <500(直道)且横置有信号时学习。
     * 【只学发车后前 3 秒】: 长时间边跑边学, 车体轻微摆动会被慢慢学成零点,
     * 跑久了反而出现"直道越跑越摆"; 3 秒足够收敛, 之后锁定不再变。 */
    if (!s_zero_lock)
    {
        if ((vsum_r < V_RAW_IGNORE) && (hsum_r > 800.0f) && (fabsf(eh) < ZERO_GATE))
        {
            ADC_Zero_H += (eh - ADC_Zero_H) * ZERO_LEARN;
        }
        if (s_zero_ms < 60000) s_zero_ms += 5;
        if (s_zero_ms >= 3000) s_zero_lock = 1;
    }
    if (ADC_Zero_H >  ZERO_LIMIT) ADC_Zero_H =  ZERO_LIMIT;
    if (ADC_Zero_H < -ZERO_LIMIT) ADC_Zero_H = -ZERO_LIMIT;
    eh -= ADC_Zero_H;

    /* 中间两个竖置原始值 <500 时不参与(直道上本来就没信号, 不拿噪声打方向) */
    if (vsum_r >= V_RAW_IGNORE) ev = (ml - mr) / vsum_r * 100.0f;
    else                        ev = 0.0f;

    if (eh >  100.0f) eh =  100.0f;
    if (eh < -100.0f) eh = -100.0f;
    if (ev >  100.0f) ev =  100.0f;
    if (ev < -100.0f) ev = -100.0f;

    num = A * W_OUT * eh + B * W_IN * ev;
    den = A * W_OUT + B * W_IN;
    if (den < 0.001f) den = 1.0f;

    Error_Raw = num / den;
    Error = Error + (Error_Raw - Error) * ERROR_LPF;

    /* 两路独立误差, 已限幅在 ±100 */
    Error_H = Error_H + (eh - Error_H) * ERROR_LPF;
    Error_V = Error_V + (ev - Error_V) * ERROR_LPF;
}

/* 重新开始学习直道零点(发车/进入推车监视时调用) */
void ADC_Zero_Reset(void)
{
    ADC_Zero_H  = 0.0f;
    s_zero_lock = 0;      /* 发车重新开始 3 秒学习窗口 */
    s_zero_ms   = 0;
}

/* ============================================================
 *  电感标定
 *  1) 手动: Calib_State=1 时持续更新 MAX/MIN
 *  2) 自动: ADC_AutoCalib=1 时, MAX 立刻跟上新峰值但缓慢回落,
 *     MIN 缓慢回升 -> 自动跟随实际量程(电池电压/车高/赛道变化)
 *
 *  标定要点(复合布局):
 *    横置 MAX 取"正常巡航姿态下直道居中"的读数(调好后 1500~2500);
 *    竖置 MAX 取"十字/环岛交叉点"的峰值(3200~3550)。
 *    两者绝对值不要求一致 —— 各自 100 代表各自的峰值。
 * ============================================================ */
void ADC_Calibrate(void)
{
    uint8_t i;
    uint8_t sat = 0;

    for (i = 0; i < 4; i++)
    {
        /* 峰值保持: 调电位器时读"十字/环岛处的峰值" */
        if (ADC_Value[i] > ADC_Peak[i])
        {
            ADC_Peak[i] = ADC_Value[i];
        }

        /* 饱和检测: 3200~3550 最佳, 超过 ADC_SAT_TH_VALUE 必须降增益 */
        if (ADC_Value[i] >= ADC_SAT_TH_VALUE)
        {
            sat |= (uint8_t)(1u << i);
        }

        if (Calib_State == 1)
        {
            if (ADC_Value[i] > ADC_MAX[i]) ADC_MAX[i] = ADC_Value[i];
            /* 注意: 不能再用 "|| ADC_MIN[i]==0" 把第一拍读数当最小值。
             * 若第一拍车正好在导线附近(2800), MIN 就永远卡在 2800,
             * 之后直道仍是 2800 -> 归一化恒 0, 屏幕"离得近也显示 0"、
             * 直道往一边偏、直角弯看不见, 全是这一行造成的。 */
            if (ADC_Value[i] < ADC_MIN[i]) ADC_MIN[i] = ADC_Value[i];
            /* 防呆: 量程至少 200。否则分母趋 0, 归一化会被拉到满量程 */
            if (ADC_MAX[i] < ADC_MIN[i] + 200) ADC_MAX[i] = ADC_MIN[i] + 200;
        }
        else if (ADC_AutoCalib)
        {
            if (ADC_Value[i] > ADC_MAX[i])
            {
                ADC_MAX[i] = ADC_Value[i];          /* 新峰值立即采用 */
            }
            else
            {
                ADC_MAX[i] = ADC_MAX[i] - (uint16_t)((ADC_MAX[i] - ADC_Value[i]) / 64);
            }
            if (ADC_Value[i] < ADC_MIN[i])
            {
                ADC_MIN[i] = ADC_Value[i];          /* 新低立即采用 */
            }
            else
            {
                /* 回升放慢到 /512: 原来 /64 会把直道读数慢慢抬成最小值,
                 * 归一化随之掉到 0("飘得很大"的另一半原因)。 */
                ADC_MIN[i] = ADC_MIN[i] + (uint16_t)((ADC_Value[i] - ADC_MIN[i]) / 512);
            }
            if (ADC_MAX[i] <= ADC_MIN[i] + 10)
            {
                ADC_MAX[i] = ADC_MIN[i] + 10;       /* 防呆: 保证分母为正 */
            }
        }
    }

    /* 手动标定: 采集够 ADC_CALIB_MS 就自动锁定为"完成", 防止一直停在标定中 */
    if (Calib_State == 1)
    {
        if (s_calib_ms < 60000) s_calib_ms += 5;
        if (s_calib_ms >= ADC_CALIB_MS) Calib_State = 2;
    }

    ADC_SatFlag = sat;
    if (sat != 0)
    {
        ADC_SatSeen = 1;
    }
}

/* 清空标定值并开始重新标定(KEY3 调用)
 * 注意: 必须把 Calib_State 置 1, 否则采集分支永远不进,
 * MAX/MIN 一直为 0 -> 归一化分母为 1 -> 四路恒 100、误差恒 0(车"看不见线")。 */
void ADC_Calib_Reset(void)
{
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        ADC_MAX[i] = 0;
        ADC_MIN[i] = 4095;      /* 必须是"最大值", 不能是 0: 从 0 开始会卡在第一个读数 */
    }
    ADC_Zero_H = 0.0f;
    Calib_State = 1;        /* 开始采集, ADC_CALIB_MS 后自动锁定 */
    s_calib_ms  = 0;
    ADC_SatSeen = 0;
}

/* 清空峰值保持(调完电位器后清零, 再跑一遍看峰值) */
void ADC_Peak_Reset(void)
{
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        ADC_Peak[i] = 0;
    }
    ADC_SatSeen = 0;
}
