#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

/* 计数/里程 */
extern volatile int16_t Encoder_L_Count;   /* 本次 5ms 的计数(带方向) */
extern volatile int16_t Encoder_R_Count;
extern volatile int32_t Encoder_L_Total;   /* 累计(菜单 KEY3 可清零) */
extern volatile int32_t Encoder_R_Total;
extern volatile int32_t Enc_Pulse_L;       /* 累计(上电起, 诊断用, 不清零) */
extern volatile int32_t Enc_Pulse_R;
extern volatile int8_t  Enc_Dir_L;         /* 方向 +1 / -1 */
extern volatile int8_t  Enc_Dir_R;
extern volatile int32_t Encoder_CPR;       /* 每转计数(220 / 440 / 880) */
extern volatile int32_t Encoder_Edge;      /* 1=只数上升沿 2=上升+下降 */
extern volatile int32_t Encoder_Method;    /* 1=外部时钟计数 0=正交解码 */

/* 速度 */
extern volatile int32_t Speed_L_RPM;
extern volatile int32_t Speed_R_RPM;
extern volatile int32_t Speed_L_mms;       /* 左轮线速度 mm/s */
extern volatile int32_t Speed_R_mms;
extern volatile int32_t Speed_Avg_mms;

void    Encoder_Init(void);
void    Encoder_Update(float dt);
uint8_t Encoder_ReadLevel(void);           /* bit0=PA0 bit1=PA1 bit2=PB6 bit3=PB7 */

#endif
