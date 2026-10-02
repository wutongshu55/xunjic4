# 智能车电磁循迹校赛软件（2026）

> 团队：天津科技大学疾风车队  
> 车型：电磁循迹智能车（四路电感）  
> 主控：STM32F103C8T6（标准外设库 V3.5.0）  
> 开发环境：Keil MDK V5（ARMCC V5.06）  
> 仓库：`git@gitee.com:wutongshu6/track-calibration-software.git`

---

## 一、项目简介

本项目是一套基于 STM32F103 标准库的**电磁循迹智能车固件**，面向校赛场景。车辆通过 4 路电磁电感采集地面 100Hz 交流导线产生的电磁场，经 ADC+DMA 高速采集、排序去极值滤波、归一化差比和求误差，由 PD 控制器输出差速驱动双电机转向。

除基础循迹外，本版本还包含：

- **赛道元素识别**：十字路口直行计数、右侧菱形环、右侧圆环（均为状态机 + 开环定时通过）
- **编码器测速**：TIM2/TIM4 正交解码，实时显示左右轮转速与线速度
- **OLED 菜单系统**：7 页参数配置（路线、菱形环、圆环、车速档位、圈数、发车、复位）
- **蜂鸣器事件提示**：过路口、过环、过圈、发车均有声音反馈

---

## 二、硬件方案

### 2.1 主控

| 项目 | 方案 |
|------|------|
| 芯片 | STM32F103C8T6（Cortex-M3，72MHz，64KB Flash，20KB SRAM） |
| 时钟 | 外部 8MHz 晶振 → PLL 倍频 72MHz |
| 调试 | SWD（ST-Link V2） |
| 供电 | 3.3V LDO + 电机独立供电 |

### 2.2 引脚分配（依据嘉立创原理图）

| 外设 | 引脚 | 复用功能 | 说明 |
|------|------|---------|------|
| 电感 L（左） | PA4 | ADC1_CH4 | 模拟输入 |
| 电感 ML（中左） | PA5 | ADC1_CH5 | 模拟输入 |
| 电感 MR（中右） | PA6 | ADC1_CH6 | 模拟输入 |
| 电感 R（右） | PA7 | ADC1_CH7 | 模拟输入 |
| 左编码器 A 相 | PA0 | TIM2_CH1 | 正交解码 |
| 左编码器 B 相 | PA1 | TIM2_CH2 | 正交解码 |
| 右编码器 A 相 | PB6 | TIM4_CH1 | 正交解码 |
| 右编码器 B 相 | PB7 | TIM4_CH2 | 正交解码 |
| 蜂鸣器 | PA8 | GPIO_OUT | 推挽输出 |
| KEY1 | PA9 | GPIO_IPU | 上拉输入，按下低电平 |
| KEY2 | PA10 | GPIO_IPU | 上拉输入 |
| KEY3 | PA11 | GPIO_IPU | 上拉输入 |
| KEY4 | PA12 | GPIO_IPU | 上拉输入 |
| 串口 TX | PA2 | USART2_TX | 预留 |
| 串口 RX | PA3 | USART2_RX | 预留 |
| PWM A（左电机） | PB0 | TIM3_CH3 | 17kHz |
| PWM B（右电机） | PB1 | TIM3_CH4 | 17kHz |
| 左电机方向 AIN1 | PB12 | GPIO_OUT | 推挽输出 |
| 左电机方向 AIN2 | PB13 | GPIO_OUT | 推挽输出 |
| 右电机方向 BIN1 | PB14 | GPIO_OUT | 推挽输出 |
| 右电机方向 BIN2 | PB15 | GPIO_OUT | 推挽输出 |
| OLED SCL | PB8 | GPIO_OD | 开漏，软件 I2C |
| OLED SDA | PB9 | GPIO_OD | 开漏，软件 I2C |
| 控制定时器 | TIM1 | — | 5ms 更新中断（200Hz） |

> **注意**：OLED 使用 PB8/PB9。早期版本曾占用 PB6/PB7，与右编码器冲突，已按原理图修正。

### 2.3 电机与机械参数

| 参数 | 值 | 说明 |
|------|----|------|
| 减速比 | 20 | 电机:车轮 |
| 轮子外径 | 48 mm | 周长 ≈ 150.80 mm |
| 编码器线数 | 11 PPR | 单相每转脉冲数，需按实际修改 |
| 每转计数 | 880 | 11 × 4 倍频 × 20 减速比 |
| PWM 频率 | 17 kHz | 高于听觉上限，减小啸叫 |
| PWM 分辨率 | 10000 | `PWM_DUTY_MAX`，即占空比 0~100% |

---

## 三、软件架构

### 3.1 系统分层

```
┌──────────────────────────────────────────────────────┐
│  应用层   main.c（主循环）                            │
│           ├─ 按键扫描 Key_GetNum()                    │
│           ├─ 菜单交互 Menu_KeyHandler()               │
│           └─ OLED 刷新 Menu_Display()/RunDisplay()    │
├──────────────────────────────────────────────────────┤
│  算法层   ADC.c（滤波+归一化+误差）                    │
│           PID.c（PD 差速）                            │
│           Ring_Judge.c（赛道元素状态机）               │
│           Encoder.c（测速）                           │
├──────────────────────────────────────────────────────┤
│  驱动层   PWM.c / Motor.c / OLED.c / Key.c / Beep.c   │
├──────────────────────────────────────────────────────┤
│  中断层   TIM1_UP_IRQHandler（5ms 控制周期）           │
└──────────────────────────────────────────────────────┘
```

### 3.2 控制链路

```
地面导线（100Hz 交变电磁场）
    ↓
4 路电感（PA4~PA7）
    ↓
ADC1 规则组扫描（4 通道）+ DMA1_CH1 自动搬运
    ↓
ADC_TempDat[4] → ADC_Dat[20][4] 环形缓冲
    ↓
每 20 次采样 → ADC_Filter()（插入排序 + 去极值均值）→ ADC_Value[4]
    ↓
Get_Error()：归一化 → 差比和差 → Error（-100~+100）
    ↓
PID_Calculate()：PD → Speed_Delta
    ↓
Track_Judge()：赛道元素状态机（可覆盖 Speed_Delta）
    ├─ Cross_Judge()   十字路口
    ├─ Diamond_Judge() 菱形环
    ├─ Ring_Judge()    圆环
    └─ Lap_Judge()     圈数
    ↓
CarMotor_Control()：L = Basic + Δ, R = Basic - Δ
    ↓
Motor_Out()：GPIO 方向 + PWM 占空比 → 电机
```

### 3.3 5ms 中断流程（TIM1_UP_IRQHandler）

```c
/* 1. ADC 采集：5ms 内采 20 次，触发一次滤波 */
for (i = 0; i < 20; i++) ADC_GetValue();
ADC_Calibrate();          // 标定模式时更新 MAX/MIN
Get_Error();              // 归一化 + 差比和误差

/* 2. 编码器测速（采样周期 0.005s） */
Encoder_Update(0.005f);

/* 3. 蜂鸣器计时递减 */
Beep_Tick();

/* 4. 运行控制 */
if (Menu_Running == 1) {
    PID_Calculate();
    Track_Judge();
    Speed_Basic = Speed_Level_Table[Menu_SpeedLevel];  // 三档车速
    CarMotor_Control();
} else if (Menu_Running == 2) {
    Motor_Out(0, 0);      // 完成停车
} else {
    Motor_Out(0, 0);      // 待发车
}
```

主循环 `while(1)` 仅负责按键扫描与 OLED 刷新，**不参与控制运算**，保证控制周期稳定。

### 3.4 时序

| 环节 | 周期 | 说明 |
|------|------|------|
| 控制周期 | 5 ms | TIM1 更新中断，200Hz |
| ADC 采样 | 5ms 内 20 次 | 每次 4 通道，DMA 传输约 4us |
| 滤波更新 | 5 ms 一次 | 20 样本排序去极值均值 |
| 编码器测速 | 5 ms | 正交计数差分 |
| 按键扫描 | 10 ms | 主循环 Delay_ms(10) |
| OLED 刷新 | 200 ms | 运行页；菜单页按需刷新 |

---

## 四、模块说明

| 文件 | 功能 |
|------|------|
| `Hardware/Common_headfile.h` | 公共头文件，统一包含所有模块 |
| `Hardware/Timer.c/h` | TIM1 5ms 定时中断初始化（200Hz） |
| `Hardware/PWM.c/h` | TIM3 CH3/CH4 PWM 初始化与占空比设置 |
| `Hardware/Motor.c/h` | 电机方向 GPIO + PWM 输出 + 差速控制 |
| `Hardware/ADC.c/h` | ADC+DMA 采集、排序去极值滤波、归一化误差计算、标定 |
| `Hardware/PID.c/h` | PD 控制器 |
| `Hardware/Ring_Judge.c/h` | 十字路口 / 菱形环 / 圆环 / 圈数状态机 |
| `Hardware/Encoder.c/h` | TIM2/TIM4 正交解码测速 |
| `Hardware/Key.c/h` | 四按键扫描消抖 |
| `Hardware/Beep.c/h` | 蜂鸣器定时控制 |
| `Hardware/Menu.c/h` | 菜单参数定义、按键处理、显示逻辑 |
| `Hardware/All_Init.c/h` | 总初始化 |
| `Hardware/OLED.c/h` | OLED 软件 I2C 驱动（SSD1306，PB8/PB9） |
| `Hardware/OLED_Font.h` | OLED 8x16 字库 |
| `Hardware/Delay.c/h` | 延时 |
| `User/main.c` | 主程序 + TIM1 中断服务函数 |

---

## 五、核心算法与数学原理

### 5.1 ADC + DMA 采集

- ADC1 规则组扫描 4 通道（CH4~CH7），扫描模式 + 软件触发
- DMA1_Channel1 外设到内存，半字宽度，传输 4 个样本到 `ADC_TempDat[4]`
- 每次采集前重配 DMA 计数器（`DMA_SetCurrDataCounter`），防止上次转换残留导致数据错位
- `while (DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET)` 等待传输完成

### 5.2 排序去极值均值滤波

对每个电感的 20 个样本做**插入排序**，去掉最小值与最大值，剩余 18 个求算术平均：

$$
\text{ADC\_Value}[n] = \frac{1}{18}\sum_{i=1}^{18} \text{ADC\_Dat}[i][n]
$$

其中 `ADC_Dat[0]` 为最小值、`ADC_Dat[19]` 为最大值，均被剔除。

**为什么用排序去极值而不是简单平均？**

电磁感应在导线接头、金属异物附近会产生**脉冲尖峰**，简单平均会被单个尖峰拉偏。排序去极值对脉冲干扰的抑制效果接近中值滤波，同时保留了均值的平滑性，且 20 个样本的插入排序在 M3 上耗时极短（约几十微秒），满足 5ms 实时性要求。

### 5.3 归一化

将原始 ADC 值（0~4095）线性映射到 0~100：

$$
\text{ADC\_L} = \frac{\text{ADC\_Value}[0] - \text{ADC\_MIN}[0]}{\text{ADC\_MAX}[0] - \text{ADC\_MIN}[0]} \times 100
$$

ML/MR/R 同理。带除零保护（分母 ≤ 0 时取 1）与限幅（0~100）。

**作用**：消除四路电感因线径、匝数、安装高度、放大器增益不同造成的个体差异，使误差计算与具体传感器解耦。

### 5.4 差比和误差（核心）

$$
\text{Error} = \frac{A(\text{L} - \text{R}) + B(\text{ML} - \text{MR})}{A(\text{L} + \text{R}) + C|\text{ML} - \text{MR}|} \times 100
$$

- **分子**：外侧电感差 `A(L-R)` 反映车辆相对导线的左右偏移；内侧电感差 `B(ML-MR)` 提供更精细的近端修正
- **分母**：外侧电感之和 `A(L+R)` 反映信号总强度；引入 `C|ML-MR|` 使分母在中间电感失衡时增大，**抑制小弯道下的误差过冲**
- **除零保护**：分母 < 1 时取 1
- **限幅**：Error ∈ [-100, +100]，负值代表左偏，正值代表右偏

**为什么用"差比和"而不是简单差？**

若直接用 `L - R`，当导线距离很远（信号都很弱）时，微小噪声会被放大成巨大误差；而除以总和后，误差是**相对量**，与信号强度无关，车辆在赛道不同位置（直线、坡道、远离导线处）都能获得一致的响应特性。

### 5.5 PD 控制

$$
\text{Speed\_Delta} = k_p \cdot \text{Error} + k_d \cdot (\text{Error} - \text{Error\_Last})
$$

- **比例项** $k_p \cdot \text{Error}$：产生与偏差成正比的纠偏力
- **微分项** $k_d \cdot \Delta\text{Error}$：反映偏差变化趋势，起阻尼作用，抑制过冲与振荡

差速输出：

$$
L_{\text{speed}} = \text{Speed\_Basic} + \text{Speed\_Delta}, \quad
R_{\text{speed}} = \text{Speed\_Basic} - \text{Speed\_Delta}
$$

Error > 0（右偏）时左轮加速、右轮减速，车辆向右修正。

### 5.6 编码器测速

TIM2/TIM4 配置为 **TI12 正交解码模式**（4 倍频），每个控制周期读取并清零计数器：

$$
N_{\text{rev}} = \frac{\Delta \text{Count}}{\text{COUNTS\_PER\_REV}}, \quad
\text{COUNTS\_PER\_REV} = \text{PPR} \times 4 \times \text{GEAR\_RATIO} = 11 \times 4 \times 20 = 880
$$

转速（RPM）：

$$
\text{RPM} = \frac{\Delta \text{Count}}{880} \times \frac{60}{\Delta t}
$$

线速度（mm/s）：

$$
v = \frac{\Delta \text{Count}}{880} \times \pi D \times \frac{1}{\Delta t}
= \frac{\Delta \text{Count}}{880} \times 150.80 \times \frac{1}{0.005}
$$

其中 $D = 48\text{mm}$ 为轮子外径，$\Delta t = 5\text{ms}$ 为采样周期。

### 5.7 赛道元素状态机

#### 十字路口

特征：四路电感同时 ≥ `TH_ALL_HIGH`(70)。动作：**直行通过不转弯**，计数 +1，蜂鸣 30ms。用 `Cross_Timer` 做 80 周期（400ms）闭锁，避免重复计数。

#### 菱形环（赛道右侧，固定右转）

| 阶段 | 动作 | 转移条件 |
|------|------|---------|
| 0 | 检测入口：`ADC_R ≥ 60 && ADC_MR ≥ 60` | 满足 → 1 |
| 1 | 强制右转 `Speed_Delta = -1200` | 200 周期 → 2 |
| 2 | 恢复正常 PD 循迹 | 150 周期 → 3 |
| 3 | 强制右转出环 `Speed_Delta = -1000` | 120 周期 → 0 |

#### 圆环（赛道右侧，固定右转）

| 阶段 | 动作 | 转移条件 |
|------|------|---------|
| 0 | 检测入口：`ADC_R ≥ 60 && ADC_MR < 15` | 满足 → 1 |
| 1 | 强制右转 `Speed_Delta = -1050` | 200 周期 → 2 |
| 2 | 恢复正常 PD 循迹 | 150 周期 → 3 |
| 3 | 检测出环点：`ADC_R ≥ 60 && ADC_MR ≥ 60` | 满足 → 4 |
| 4 | 强制右转出环 `Speed_Delta = -1000` | 120 周期 → 0 |

> 环内采用"开环定时 + 强制差速"策略，不依赖环内电感信号（环内磁场畸变严重，闭环不可靠）。各阶段延时需按实际车速与环尺寸现场标定。

#### 圈数判断

起点特征为四路电感同时 ≥ 70。用 `lap_lock` 做边沿检测，每进入起点一次 `Menu_DoneLaps++`，达到 `Menu_TargetLaps` 后 `Menu_Running = 2` 停车。离开起点（L、R 均 < 15）后解锁，避免同一起点重复计数。

---

## 六、OLED 菜单系统

详细说明见 [`菜单功能说明.md`](菜单功能说明.md)。概要：

### 6.1 按键

| 按键 | 引脚 | 菜单模式 | 运行模式 |
|:----:|:----:|:--------|:--------|
| KEY1 | PA9 | 上一页 | 任意键暂停回菜单 |
| KEY2 | PA10 | 下一页 | 同上 |
| KEY3 | PA11 | 数值减 | 同上 |
| KEY4 | PA12 | 数值加 / 确认 | 同上 |

### 6.2 菜单页（7 页）

| 页 | 标题 | 参数 | 范围 |
|:--:|------|------|------|
| 1 | Route | 路线模式 | 0=主路线 1=菱形环 2=圆环 |
| 2 | Diamnd | 菱形环开关 | 0=跳过 1=进入 |
| 3 | Ring | 圆环开关 | 0=跳过 1=进入 |
| 4 | Speed | 车速档位 | 0=慢 1=中 2=快 |
| 5 | Laps | 目标圈数 | 1~9 |
| 6 | Start | 发车 | IDLE / RUN / DONE |
| 7 | Reset | 复位计数 | K4 清零 |

### 6.3 三档车速

```c
const int16_t Speed_Level_Table[3] = {3000, 5000, 7500};  // PWM 占空比
```

| 档位 | PWM | 适用场景 |
|:----:|:---:|:--------|
| 慢 | 3000 | 调试、过密弯、过环岛 |
| 中 | 5000 | 默认，兼顾稳定与速度 |
| 快 | 7500 | 直线冲刺 |

### 6.4 运行显示页

```
+---------------------+
|L100 ML100 MR100 50  |  行1：四路电感归一化值（0~100）
|R100 E+0050          |  行2：右电感 + 循迹误差
|V1234 mm/s    L0/    |  行3：平均速度 + 已完成圈数
|2 Crs03 R0 Spd5000   |  行4：目标圈/十字计数/环状态/基础PWM
+---------------------+
```

---

## 七、参数标定指南

### 7.1 ADC_MAX / ADC_MIN（必标）

在 `All_Init.c` 中默认 `4000 / 200`（占位值）。标定步骤：

1. 车放赛道无电磁场处，记录 4 路 `ADC_Value` 作为 `ADC_MIN[i]`
2. 车放导线正上方，记录 4 路 `ADC_Value` 作为 `ADC_MAX[i]`
3. 写入 `All_Init()` 初始值

未标定时归一化退化，车可跑但循迹精度差。

### 7.2 PD 参数 kp / kd

`PID.c` 中默认 `kp = 15.0, kd = 5.0`。

| 现象 | 调整 |
|------|------|
| 弯道振荡、直道蛇形 | 减小 kp 或增大 kd |
| 弯道冲出 | 增大 kp |
| 高频抖动 | 减小 kd |
| 响应慢、超调大 | 增大 kd |

### 7.3 误差权重 A / B / C

`ADC.c` 中默认全为 1.0。

| 权重 | 作用 |
|------|------|
| A | 外侧电感（L/R）权重 |
| B | 内侧电感（ML/MR）权重 |
| C | 分母中 \|ML-MR\| 修正强度，增大可抑制小弯过冲 |

### 7.4 编码器线数 ENCODER_PPR

`Encoder.c` 中默认 11。若实测转速显示值与实际不符，修改此值：

$$
\text{显示值} \times \frac{\text{实际PPR}}{11} = \text{真实值}
$$

### 7.5 环岛阈值与延时

`Ring_Judge.c` 中：

| 参数 | 默认 | 说明 |
|------|------|------|
| `TH_HIGH` | 60 | 电感"在线"阈值 |
| `TH_LOW` | 15 | 电感"离线"阈值 |
| `TH_ALL_HIGH` | 70 | 四路全高（路口/起点）阈值 |
| `RING_TURN` | 200 | 进环转弯延时（×5ms = 1s） |
| `RING_DELAY` | 150 | 环内循迹延时 |
| `RING_EXIT` | 120 | 出环延时 |
| `CROSS_DELAY` | 80 | 十字路口直行闭锁 |

---

## 八、编译与烧录

### 8.1 编译

1. Keil MDK V5 打开 `Project.uvprojx`
2. 工程已配置：芯片 STM32F103C8、包含路径、启动文件 `startup_stm32f10x_md.s`
3. Build（F7）或命令行：
   ```
   UV4.exe -j0 -b Project.uvprojx -o build.log
   ```
4. 输出：`Objects/gujianku.axf` + `Objects/gujianku.hex`

### 8.2 烧录

1. ST-Link V2 通过 SWD 连接（SWDIO=PA13, SWCLK=PA14）
2. Keil 烧录配置：UL2CM3 + STM32F10x_128 Flash 算法
3. Download（F8）

---

## 九、工程目录结构

```
智能车软件/
├── Project.uvprojx          Keil 工程文件
├── README.md                本文档
├── 菜单功能说明.md           菜单系统详细说明
├── CMSIS/                   内核与系统（core_cm3, system_stm32f10x）
├── Startup/                 启动文件（startup_stm32f10x_md.s）
├── Lib/                     STM32 标准外设库（inc/ + src/）
├── Hardware/                硬件驱动模块
│   ├── Common_headfile.h    公共头文件
│   ├── Timer.c/h            定时器（5ms 中断）
│   ├── PWM.c/h              PWM 输出
│   ├── Motor.c/h            电机驱动 + 差速控制
│   ├── ADC.c/h              ADC 采集 + 滤波 + 归一化 + 误差
│   ├── PID.c/h              PD 控制
│   ├── Ring_Judge.c/h       赛道元素状态机
│   ├── Encoder.c/h          编码器正交解码测速
│   ├── Key.c/h              四按键扫描
│   ├── Beep.c/h             蜂鸣器
│   ├── Menu.c/h             菜单系统
│   ├── All_Init.c/h         总初始化
│   ├── OLED.c/h             OLED 显示（软件 I2C，PB8/PB9）
│   ├── OLED_Font.h          OLED 字库
│   └── Delay.c/h            延时
├── User/                    用户代码
│   ├── main.c               主程序 + TIM1 中断
│   ├── stm32f10x_conf.h     标准库配置
│   └── stm32f10x_it.c/h     中断模板
├── Objects/                 编译输出
└── Listings/                列表文件
```

---

## 十、已知限制与改进方向

| 项 | 现状 | 改进建议 |
|----|------|---------|
| 速度闭环 | 编码器仅显示，未参与控制 | 用 `Speed_L_mms`/`Speed_R_mms` 做速度闭环，替代开环 PWM |
| 环岛通过 | 开环定时，依赖现场标定 | 改为电感特征识别 + 闭环，或引入陀螺仪 |
| 丢线保护 | 无 | 四路全低时按最后 Error 方向惯性搜索 |
| 参数调试 | 需重新烧录 | 串口（PA2/PA3 已预留）或蓝牙在线调参 |
| ADC 标定 | 手动量取写入代码 | 菜单增加标定页，`ADC_Calibrate()` 已预留接口 |
| 制动 | 无主动刹车 | 出环/停车阶段加能耗制动 |
| OLED 刷新 | 软件 I2C 约 50ms/帧 | 改硬件 I2C 或 SPI OLED 提速 |

---

## 十一、版本记录

| 版本 | 日期 | 内容 |
|------|------|------|
| v1.0 | 2026-10 | 按嘉立创原理图重构：修正 OLED 引脚至 PB8/PB9；新增编码器测速、四按键菜单、蜂鸣器；重写赛道元素状态机（十字/菱形环/圆环/圈数）；三档车速 3000/5000/7500 |
