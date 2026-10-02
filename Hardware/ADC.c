#include "Common_headfile.h"

volatile uint16_t ADC_TempDat[4];
volatile uint16_t ADC_Dat[20][4];
volatile uint16_t ADC_Value[4];
volatile int16_t ADC_L = 0, ADC_ML = 0, ADC_MR = 0, ADC_R = 0;
volatile float Error = 0.0f, Error_Last = 0.0f;

uint16_t ADC_MAX[4] = {0, 0, 0, 0};
uint16_t ADC_MIN[4] = {0, 0, 0, 0};

float A = 1.0f;
float B = 1.0f;
float C = 1.0f;

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

    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 4, ADC_SampleTime_28Cycles5);

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

void ADC_Filter(void)
{
    int16_t i, j, n, current;
    int32_t ADC_Val_Sum[4] = {0, 0, 0, 0};

    for (n = 0; n < 4; n++)
    {
        for (i = 1; i < 20; i++)
        {
            current = ADC_Dat[i][n];
            j = i - 1;
            while (j >= 0 && ADC_Dat[j][n] > current)
            {
                ADC_Dat[j + 1][n] = ADC_Dat[j][n];
                j = j - 1;
            }
            ADC_Dat[j + 1][n] = current;
        }
    }

    for (i = 1; i < 19; i++)
    {
        ADC_Val_Sum[0] += ADC_Dat[i][0];
        ADC_Val_Sum[1] += ADC_Dat[i][1];
        ADC_Val_Sum[2] += ADC_Dat[i][2];
        ADC_Val_Sum[3] += ADC_Dat[i][3];
    }

    ADC_Value[0] = ADC_Val_Sum[0] / 18;
    ADC_Value[1] = ADC_Val_Sum[1] / 18;
    ADC_Value[2] = ADC_Val_Sum[2] / 18;
    ADC_Value[3] = ADC_Val_Sum[3] / 18;
}

void Get_Error(void)
{
    float den, num, den2;

    den = (float)(ADC_MAX[0] - ADC_MIN[0]);
    if (den <= 0.0f) den = 1.0f;
    ADC_L = (int16_t)(((float)ADC_Value[0] - (float)ADC_MIN[0]) / den * 100.0f);

    den = (float)(ADC_MAX[1] - ADC_MIN[1]);
    if (den <= 0.0f) den = 1.0f;
    ADC_ML = (int16_t)(((float)ADC_Value[1] - (float)ADC_MIN[1]) / den * 100.0f);

    den = (float)(ADC_MAX[2] - ADC_MIN[2]);
    if (den <= 0.0f) den = 1.0f;
    ADC_MR = (int16_t)(((float)ADC_Value[2] - (float)ADC_MIN[2]) / den * 100.0f);

    den = (float)(ADC_MAX[3] - ADC_MIN[3]);
    if (den <= 0.0f) den = 1.0f;
    ADC_R = (int16_t)(((float)ADC_Value[3] - (float)ADC_MIN[3]) / den * 100.0f);

    ADC_L = ADC_L > 100 ? 100 : ADC_L;
    ADC_ML = ADC_ML > 100 ? 100 : ADC_ML;
    ADC_MR = ADC_MR > 100 ? 100 : ADC_MR;
    ADC_R = ADC_R > 100 ? 100 : ADC_R;

    ADC_L = ADC_L < 0 ? 0 : ADC_L;
    ADC_ML = ADC_ML < 0 ? 0 : ADC_ML;
    ADC_MR = ADC_MR < 0 ? 0 : ADC_MR;
    ADC_R = ADC_R < 0 ? 0 : ADC_R;

    Error_Last = Error;

    /* 差比和误差: 适用于直线+小弯道+十字路口
     * 分子: 外侧差(A*(L-R)) + 内侧差(B*(ML-MR))
     * 分母: 外侧和 + 内侧差绝对值(防止除零并增强弯道灵敏度)
     */
    num = A * (ADC_L - ADC_R) + B * (ADC_ML - ADC_MR);
    den2 = A * (ADC_L + ADC_R) + C * abs(ADC_ML - ADC_MR);
    if (den2 < 1.0f) den2 = 1.0f;
    Error = num / den2 * 100.0f;

    if (Error > 100.0f) Error = 100.0f;
    else if (Error < -100.0f) Error = -100.0f;
}

/* ============================================================
 *  电感标定: 在静止状态下手动转动小车扫描赛道
 *  调用时持续更新每路电感的最大最小值
 * ============================================================ */
uint8_t Calib_State = 0;   /* 0=未标定 1=标定中 2=标定完成 */

void ADC_Calibrate(void)
{
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        if (Calib_State == 1)
        {
            if (ADC_Value[i] > ADC_MAX[i]) ADC_MAX[i] = ADC_Value[i];
            if (ADC_Value[i] < ADC_MIN[i] || ADC_MIN[i] == 0) ADC_MIN[i] = ADC_Value[i];
        }
    }
}
