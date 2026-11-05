#ifndef __MENU_H
#define __MENU_H

#include "stm32f10x.h"

/* 发车页在菜单中的页号(改页数时这里要同步)。
 * 车速选择页已删除, 菜单 6 页: 0=发车 1=标定 2=距离/编码器 3=Kp 4=Kd 5=模式 */
#define MENU_PAGE_START 0

/* ============ 车速已固定, 不再可选 ============
 * 基准占空比固定 20%(PWM_BASE_DUTY), 全程恒速:
 * 弯道 / 丢线 / 出弯都不再变速(PID_Speed_Schedule 用开关 SPEED_SCHEDULE_EN 控制)。
 * 名义速度 SPEED_NOMINAL_MMS 只用于差速换算与"等效速度"显示。 */
#define PWM_BASE_DUTY     2000   /* 固定基准占空比: 2000 = 20% */
#define SPEED_NOMINAL_MMS  200   /* 名义速度(mm/s): 与 20% PWM 对应的等效值 */

/* 圈数功能已【删除】: 不再计圈, 也不存在"跑满圈数自动停车"。
 * 唯一能让车停下来的方式 = 按 KEY4(或运行页按 KEY4 回发车页)。 */

extern uint8_t Menu_HexDiaEn;      /* 六角菱形环开关 */
extern uint8_t Menu_Running;
extern uint8_t Menu_CrossCount;
extern uint8_t Menu_Page;
extern uint8_t Menu_Inited;
extern uint8_t Menu_RunInited;

/* 电感标定模式 0=固定标定值 1=运行中自动标定 */
extern uint8_t Menu_CalibMode;
/* 急弯(锯齿线/直角弯)保持模式 0=关 1=开 */
extern uint8_t Menu_ZigEn;
/* 距离测试页显示模式: 0=归一化 1=原始ADC 2=峰值保持 3=编码器自检 4=编码器引脚 */
extern uint8_t Menu_DistRaw;

/* 速度控制方式: 【固定为 PWM 开环 20%】, 菜单里不再可调。
 * 变量保留是为了兼容 Motor.c 的执行路径与主机端仿真。 */
#define SPD_MODE_PWM  2
extern uint8_t Menu_SpdMode;

void Menu_KeyHandler(uint8_t key);
void Menu_Display(void);
void Menu_RunDisplay(void);

#endif
