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

/* 转向极性(整车只改这一处):
 *   代码内部约定: 差速 Δ > 0 = 左轮快 = 车应向右转
 *   实测如果"该右转却左转 / 该左转却右转", 把这里改 -1
 *   (左右电机接线互换、车头方向装反都会导致这种情况)
 * 只翻转"转向差速", 不影响前进方向和直道等速补偿。 */
#define STEER_SIGN   (-1.0f)

void Motor_Init(void);
void Motor_Out(int16_t L_Speed, int16_t R_Speed);
void CarMotor_Control(void);
void Speed_Loop_Reset(void);
void Speed_Fault_Reset(void);

#endif
