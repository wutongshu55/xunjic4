#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"

extern volatile float kp, kd;
/* 前瞻延迟比例(菜单第 7 页): 0=立即转弯, 0.30=默认(原 0.60 的一半) */
extern volatile float Lag_Ratio;
extern volatile uint8_t Lag_Cycles;   /* 当前延迟周期数(5ms/周期) */
extern volatile int16_t Speed_Delta;
extern volatile float D_Filt;

/* 弯道分级/转弯过程实时状态(菜单"弯道判定页"显示, 拍视频看数据用) */
extern volatile uint8_t Curve_Grade_Now;   /* 0直道 1=45° 2=90° 3=110° */
extern volatile uint8_t Curve_Turn_Now;    /* 1=转弯进行中 */
extern volatile uint8_t Curve_Conf_Now;    /* 同方向确认计数 */
void PID_Curve_Reset(void);

/* 丢线保护状态 */
extern uint8_t  Lost_State;
extern uint16_t Lost_Timer;
extern int8_t   Last_Dir;

/* 急弯(锯齿线/直角弯)保持状态 */
extern uint8_t  Sharp_State;
extern uint16_t Sharp_Timer;
extern uint16_t Sharp_ExitCnt;
extern int8_t   Sharp_Dir;

/* 无陀螺仪航向估计(编码器差速 -> 角速度 -> 航向角, 单位: 度)
 * 正 = 车头向右(顺时针)偏转。只在单次转弯内可信, 直道会自动回零。 */
extern volatile float   Yaw_Deg;
extern volatile float   Yaw_Rate_Dps;
extern volatile uint8_t Yaw_Valid;

void PID_Calculate(void);
void PID_LostLine_Check(void);
void PID_Sharp_Check(void);
void PID_Yaw_Update(void);
void PID_D_Reset(void);
void PID_Apply_Reset(void);
int16_t PID_Apply(int16_t delta);
int16_t PID_Speed_Schedule(int16_t basic);

#endif
