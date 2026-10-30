/*
 * 主机端仿真用的 STM32 存根头文件。
 * 只为让 ADC.c / PID.c / Ring_Judge.c 能在 PC 上编译运行(逻辑验证),
 * 不涉及任何真实外设行为。
 */
#ifndef __STM32F10X_H_STUB
#define __STM32F10X_H_STUB

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define ENABLE  1
#define DISABLE 0
#define SET     1
#define RESET   0
#define Bit_SET   1
#define Bit_RESET 0

/* GPIO */
#define GPIO_Mode_AIN      0x00
#define GPIO_Mode_IPU      0x08
#define GPIO_Mode_Out_PP   0x10
#define GPIO_Speed_50MHz   0x03
#define GPIO_Pin_0         0x0001
#define GPIO_Pin_1         0x0002
#define GPIO_Pin_4         0x0010
#define GPIO_Pin_5         0x0020
#define GPIO_Pin_6         0x0040
#define GPIO_Pin_7         0x0080
#define GPIO_Pin_8         0x0100
#define GPIO_Pin_9         0x0200
#define GPIO_Pin_10        0x0400
#define GPIO_Pin_11        0x0800
#define GPIO_Pin_12        0x1000
#define GPIO_Pin_13        0x2000
#define GPIO_Pin_14        0x4000
#define GPIO_Pin_15        0x8000

typedef struct { uint32_t CRL; uint32_t CRH; uint32_t IDR; uint32_t ODR; } GPIO_TypeDef;
extern GPIO_TypeDef *GPIOA, *GPIOB, *GPIOC;
typedef struct { uint32_t GPIO_Mode; uint32_t GPIO_Pin; uint32_t GPIO_Speed; } GPIO_InitTypeDef;

/* RCC */
#define RCC_APB2Periph_ADC1   0x01
#define RCC_APB2Periph_GPIOA  0x04
#define RCC_APB2Periph_GPIOB  0x08
#define RCC_APB2Periph_TIM1   0x10
#define RCC_APB1Periph_TIM2   0x20
#define RCC_APB1Periph_TIM3   0x40
#define RCC_APB1Periph_TIM4   0x80
#define RCC_AHBPeriph_DMA1    0x100
#define RCC_PCLK2_Div6        0x200

/* ADC */
#define ADC_Channel_4   4
#define ADC_Channel_5   5
#define ADC_Channel_6   6
#define ADC_Channel_7   7
#define ADC_SampleTime_55Cycles5   2
#define ADC_SampleTime_239Cycles5  7
#define ADC_DataAlign_Right        1
#define ADC_ExternalTrigConv_None  0
#define ADC_Mode_Independent       0
typedef struct { uint32_t CR1, CR2, SMPR2, JOFR, JOFR1, HTR, LTR, SQR1, SQR2, SQR3, CR, DR; } ADC_TypeDef;
extern ADC_TypeDef *ADC1;
typedef struct {
    uint32_t ADC_ContinuousConvMode, ADC_DataAlign, ADC_ExternalTrigConv, ADC_Mode,
             ADC_NbrOfChannel, ADC_ScanConvMode;
} ADC_InitTypeDef;

/* DMA */
#define DMA_DIR_PeripheralSRC      1
#define DMA_M2M_Disable            0
#define DMA_MemoryDataSize_HalfWord 1
#define DMA_PeripheralDataSize_HalfWord 1
#define DMA_MemoryInc_Enable       1
#define DMA_Mode_Normal            0
#define DMA_PeripheralInc_Disable  0
#define DMA_Priority_High          2
#define DMA1_FLAG_TC1              1
typedef struct { uint32_t CCR, CNDTR, CPAR, CMAR, RESERVED; } DMA_Channel_TypeDef;
extern DMA_Channel_TypeDef *DMA1_Channel1;
typedef struct {
    uint32_t DMA_BufferSize, DMA_DIR, DMA_M2M, DMA_MemoryBaseAddr, DMA_MemoryDataSize,
             DMA_MemoryInc, DMA_Mode, DMA_PeripheralBaseAddr, DMA_PeripheralDataSize,
             DMA_PeripheralInc, DMA_Priority;
} DMA_InitTypeDef;

/* 函数原型(仿真里全部为空实现) */
void RCC_APB2PeriphClockCmd(uint32_t a, uint32_t b);
void RCC_AHBPeriphClockCmd(uint32_t a, uint32_t b);
void RCC_ADCCLKConfig(uint32_t a);
void GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *s);
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *p, uint16_t pin);
void GPIO_WriteBit(GPIO_TypeDef *p, uint16_t pin, uint8_t v);
void GPIO_ResetBits(GPIO_TypeDef *p, uint16_t pin);
void GPIO_SetBits(GPIO_TypeDef *p, uint16_t pin);
void ADC_RegularChannelConfig(ADC_TypeDef *p, uint8_t ch, uint8_t rank, uint8_t st);
void ADC_Init(ADC_TypeDef *p, ADC_InitTypeDef *s);
void ADC_DMACmd(ADC_TypeDef *p, uint32_t c);
void ADC_Cmd(ADC_TypeDef *p, uint32_t c);
void ADC_ResetCalibration(ADC_TypeDef *p);
uint8_t ADC_GetResetCalibrationStatus(ADC_TypeDef *p);
void ADC_StartCalibration(ADC_TypeDef *p);
uint8_t ADC_GetCalibrationStatus(ADC_TypeDef *p);
void ADC_SoftwareStartConvCmd(ADC_TypeDef *p, uint32_t c);
void DMA_Init(DMA_Channel_TypeDef *p, DMA_InitTypeDef *s);
void DMA_Cmd(DMA_Channel_TypeDef *p, uint32_t c);
void DMA_SetCurrDataCounter(DMA_Channel_TypeDef *p, uint16_t n);
uint8_t DMA_GetFlagStatus(uint32_t f);
void DMA_ClearFlag(uint32_t f);

#endif
