#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

/* 速度单位统一为 mm/s */
extern volatile int16_t Speed_Basic;      /* 目标基准速度 mm/s */
extern volatile int16_t Speed_Target_L;   /* 左轮目标 mm/s */
extern volatile int16_t Speed_Target_R;   /* 右轮目标 mm/s */
extern volatile int16_t Duty_L;           /* 左轮实际占空比 */
extern volatile int16_t Duty_R;           /* 右轮实际占空比 */

/* 速度环参数(菜单可调) */
extern volatile float   Speed_KpS;
extern volatile float   Speed_KiS;
extern volatile uint8_t Speed_Loop_En;  /* 0=开环(仅前馈) 1=闭环 */
extern volatile uint8_t Speed_Fault;    /* 0=正常 1=测速失效已降级 2=编码器反接已纠正 */

void Motor_Init(void);
void Motor_Out(int16_t L_Speed, int16_t R_Speed);
void CarMotor_Control(void);
void Speed_Loop_Reset(void);
void Speed_Fault_Reset(void);

#endif
