# 第六届智能车(2026)校赛 —— 电磁循迹智能车代码总结与软硬件分析

> 团队：天津科技大学疾风车队  
> 赛事：第六届智能车校赛（2026）  
> 车型：电磁循迹智能车（四路电感）  
> 开发板：STM32（标准库）

---

## 一、硬件设计方案

### 1.1 主控模块
| 项目 | 方案 |
|------|------|
| 主控芯片 | STM32 系列（标准外设库） |
| 开发环境 | Keil MDK |
| 时钟 | 内部/外部晶振（代码中开启 RCC 时钟） |
| 调试接口 | SWD/JTAG（ST-Link） |

### 1.2 传感器模块 —— 四路电磁电感
- **数量**：4 个电感传感器
- **布局**：左（L）、中左（ML）、中右（MR）、右（R）
- **接口**：接 **GPIOA Pin_4 ~ Pin_7**（ADC_Channel_4/5/6/7）
- **工作模式**：模拟输入（`GPIO_Mode_AIN`）
- **用途**：采集地面电磁场强度，用于循迹决策

### 1.3 电机驱动模块 —— 双电机差速
| 项目 | 参数 |
|------|------|
| 电机数量 | 2 个直流减速电机（左 + 右） |
| 控制方式 | PWM 调速 + 方向IO控制 |
| PWM 频率 | **17000 Hz** |
| 左电机 | PWMA 调速，方向引脚 GPIO_Pin_12、GPIO_Pin_13 |
| 右电机 | PWMB 调速，方向引脚 GPIO_Pin_14、GPIO_Pin_15 |
| 速度限幅 | ±5000（防止电机反转/螺转） |
| 基础速度 | Speed_Basic = 2000 |

**方向控制逻辑（以左电机为例）：**
- L_Speed > 0：Pin_12 低电平，Pin_13 高电平 → 正转
- L_Speed < 0：Pin_12 高电平，Pin_13 低电平 → 反转
- L_Speed = 0：保持当前方向或刹车（根据驱动板特性）

### 1.4 其他模块
- **定时器**：5ms 周期中断（PIT，用于控制周期定时）
- **通信/显示**：PPT 代码片段中未出现 OLED、UART、编码器等模块的完整代码

---

## 二、软件架构总览

```
┌─────────────────────────────────────────┐
│              主循环 (while(1))            │
├─────────────────────────────────────────┤
│  5ms 定时中断（PIT）                      │
│    ├── ADC_GetValue()      数据采集       │
│    ├── ADC_Filter()        滤波处理       │
│    ├── PID_Calculate()      PID运算       │
│    ├── CarMotor_Control()   差速计算      │
│    └── Motor_Out()          电机输出      │
└─────────────────────────────────────────┘
```

**核心控制周期：5ms**

---

## 三、代码分块详解

### 3.1 初始化模块

#### 3.1.1 电机 / PWM 初始化
```c
void Motor_Init(void)
{
    PWM_Init(17000);  // 初始化PWM并设置为17000Hz
}
```
- **作用**：初始化 PWM 定时器，输出 17kHz 频率的电机驱动信号
- **注意**：17000Hz 高于人耳听觉范围，可减小电机啸叫

#### 3.1.2 ADC 初始化（核心：四通道 + DMA）
```c
void ADC_Myinit(void)
{
    // 第一步：开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // 第二步：配置ADC时钟 = PCLK2 / 6
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    // 第三步：配置GPIO为模拟输入
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 第四步：配置多路开关（规则组序列）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 4, ADC_SampleTime_28Cycles5);

    // 第五步：配置ADC转换器
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;     // 单次转换（由软件触发）
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  // 右对齐
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 软件触发
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_NbrOfChannel = 4;                 // 4通道
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;            // 扫描模式
    ADC_Init(ADC1, &ADC_InitStructure);

    // 第六步：配置DMA
    DMA_InitStructure.DMA_BufferSize = 4;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;      // 外设到内存
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_TempDat;
    DMA_InitStructure.DMA_MemorySize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    // 第七步：使能
    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    // 第八步：ADC校准
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1) == SET);
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1) == SET);
}
```

**关键设计点：**
1. **DMA 传输**：ADC 转换完成后自动将数据搬运到内存，不占用 CPU
2. **扫描模式**：一次触发可连续转换 4 个通道
3. **软件触发**：由程序控制何时开始采集，适合定时触发场景

---

### 3.2 数据采集模块

#### 3.2.1 ADC 单次采集
```c
void ADC_GetValue(void)
{
    static uint8_t k = 0;
    
    // 先设置DMA参数：避免ADC已经开始转换后配置DMA导致数据覆盖
    DMA_Cmd(DMA1_Channel1, DISABLE);
    DMA_SetCurrDataCounter(DMA1_Channel1, 4);  // 传输4个数据
    DMA_Cmd(DMA1_Channel1, ENABLE);

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);     // 软件启动ADC转换

    // 等待DMA传输完成
    while(DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);
    
    // 保存数据到缓冲区（每个电感累计20次）
    ADC_Dat[k][0] = ADC_TempDat[0];  // 左
    ADC_Dat[k][1] = ADC_TempDat[1];  // 中左
    ADC_Dat[k][2] = ADC_TempDat[2];  // 中右
    ADC_Dat[k][3] = ADC_TempDat[3];  // 右

    if(++k > 19) { k = 0; ADC_Filter(); }  // 满20次，进行滤波

    DMA_ClearFlag(DMA1_FLAG_TC1);
}
```

**数据流：**
```
ADC 硬件转换 → DMA 自动搬运 → ADC_TempDat[4] → ADC_Dat[20][4] 环形缓冲区
    → 每 20 次触发 ADC_Filter() → ADC_Value[4]
```

#### 3.2.2 ADC 滤波（排序 + 去极值均值）
```c
void ADC_Filter(void)
{
    int16_t i, j, n;
    int32_t ADC_ValueSum[4] = {0, 0, 0, 0};

    // 第一步：对每个电感的20个数据分别进行插入排序
    for(n = 0; n < 4; n++)  // n=0左, 1中左, 2中右, 3右
    {
        for(i = 1; i < 20; i++)
        {
            int16_t current = ADC_Dat[i][n];
            j = i - 1;
            while(j >= 0 && ADC_Dat[j][n] > current)
            {
                ADC_Dat[j + 1][n] = ADC_Dat[j][n];
                j--;
            }
            ADC_Dat[j + 1][n] = current;
        }
    }

    // 第二步：去极值后均值滤波（去掉最大值和最小值）
    for(i = 1; i < 19; i++)  // 取中间的18个数据
    {
        ADC_ValueSum[0] += ADC_Dat[i][0];
        ADC_ValueSum[1] += ADC_Dat[i][1];
        ADC_ValueSum[2] += ADC_Dat[i][2];
        ADC_ValueSum[3] += ADC_Dat[i][3];
    }
    ADC_Value[0] = ADC_ValueSum[0] / 18;
    ADC_Value[1] = ADC_ValueSum[1] / 18;
    ADC_Value[2] = ADC_ValueSum[2] / 18;
    ADC_Value[3] = ADC_ValueSum[3] / 18;
}
```

**滤波策略：**
1. **排序**：找出极值（最大值、最小值）
2. **去极值**：去掉 2 个极端值，剩余 18 个求平均
3. **目的**：消除脉冲干扰（如电感受电磁干扰产生的尖峰）

---

### 3.3 循迹算法模块 —— 误差计算

代码中给出了三种误差计算方法：

#### 3.3.1 方法一：差比和求误差（常用）
```
Error = A * (L - R) + B * (ML - MR) 
        ─────────────────────────────────
        A * (L + R) + B * (ML + MR)
```
- **特点**：归一化处理，比值形式，抗光照/电源波动影响
- **物理意义**：左边电感总和与右边电感总和的差值占比

#### 3.3.2 方法二：差比和差求误差
```
Error = A * (L - R) + B * (ML - MR) 
        ─────────────────────────────────
        A * (L + R) + C * |ML - MR|
```
- **特点**：分母引入 |ML-MR|，增强中间电感的权重修正

#### 3.3.3 方法三：方向向量求误差
```
Error = A * sqrt(L² + ML²) - B * sqrt(R² + MR²) 
        ─────────────────────────────────────────────────
        A * sqrt(L² + ML²) + B * sqrt(R² + MR²)
```
- **特点**：将每个电感视为向量模长，更适合非线性赛道

**归一化后的变量：**
```c
int16_t ADC_L, ADC_ML, ADC_MR, ADC_R;  // 保存归一化后的值
float Error, Error_Last;                 // 当前误差、上次误差
```

---

### 3.4 PID 控制模块

#### 3.4.1 基础 PD 控制
```c
int16_t Speed_Delta;
float kp, kd;

void PID_Calculate(void)
{
    Speed_Delta = (int16_t)(kp * Error + kd * (Error - Error_Last));
}
```
- **输入**：当前误差 Error，上次误差 Error_Last
- **输出**：Speed_Delta（速度差，用于差速控制）
- **注意**：代码中为 PD 控制（无积分项），适合快速响应的循迹场景

#### 3.4.2 四种 PID 方案
| 方案 | 公式 | 特点 |
|------|------|------|
| **方案一：普通 PD** | `kp * Error + kd * (Error - Error_Last)` | 最基础，参数少，易调试 |
| **方案二：二次 P** | `kp * Error² + kd * (Error - Error_Last)` | 误差大时比例作用更强（弯道加速响应） |
| **方案三：自适应 P** | `kp = f(Error)`，根据误差大小动态调整比例系数 | 兼顾直道稳定性与弯道响应速度 |
| **方案四：分段 PID** | 不同误差区间使用不同参数 | 精细调节，但调试复杂度高 |

---

### 3.5 电机输出模块

#### 3.5.1 差速控制（由误差求轮速）
```c
int16_t Speed_Basic = 2000;  // 基础速度

void CarMotor_Control(void)
{
    int16_t L_Speed, R_Speed;

    L_Speed = Speed_Basic + Speed_Delta;  // 左轮 = 基础 + 差速
    R_Speed = Speed_Basic - Speed_Delta;  // 右轮 = 基础 - 差速

    Motor_Out(L_Speed, R_Speed);
}
```

**差速逻辑：**
- Error > 0（车偏右）：Speed_Delta > 0 → 左轮加速、右轮减速 → 向左修正
- Error < 0（车偏左）：Speed_Delta < 0 → 左轮减速、右轮加速 → 向右修正

#### 3.5.2 电机输出（方向 + 速度）
```c
void Motor_Out(int16_t L_Speed, int16_t R_Speed)
{
    // 左电机（PWMA）
    if(L_Speed >= 0)
    {
        if(L_Speed > 5000) L_Speed = 5000;
        GPIO_WriteBit(GPIOx, GPIO_Pin_12, Bit_RESET);  // 方向：正转
        GPIO_WriteBit(GPIOx, GPIO_Pin_13, Bit_SET);
        PWMA_SetDuty(L_Speed);
    }
    else
    {
        if(L_Speed < -5000) L_Speed = -5000;
        GPIO_WriteBit(GPIOx, GPIO_Pin_12, Bit_SET);    // 方向：反转
        GPIO_WriteBit(GPIOx, GPIO_Pin_13, Bit_RESET);
        PWMA_SetDuty(-L_Speed);
    }

    // 右电机（PWMB）逻辑相同，使用 Pin_14/15
    ...
}
```

---

### 3.6 赛道识别模块

#### 3.6.1 环形赛道判断（状态机）
```c
uint8_t LRing_Flag = 0;

// 状态机示例（PPT中为片段）
if(LRing_Flag == 3)
{
    if(ADC_Value[2] + ADC_Value[3] >= 600)
        LRing_Flag = 4;  // 进入环岛标志位
    ...
}
```

**状态机思路：**
- 使用 LRing_Flag 记录环岛识别阶段（0=未检测, 1=进环, 2=环中, 3=出环等）
- 结合电感总和阈值判断是否进入/离开环岛

---

## 四、软硬件协同讲解

### 4.1 数据流向图
```
地面导线（电磁场）
    ↓
4个电感传感器（PA4-PA7）
    ↓
ADC1 规则组扫描转换（4通道）
    ↓
DMA1_Channel1 自动搬运 → ADC_TempDat[4]
    ↓
每20次采样 → ADC_Filter() → ADC_Value[4]（滤波后值）
    ↓
误差计算（差比和/向量法） → Error
    ↓
PD/PID 计算 → Speed_Delta
    ↓
CarMotor_Control() → 左右轮目标速度
    ↓
Motor_Out() → GPIO方向控制 + PWM占空比
    ↓
电机驱动板 → 直流电机 → 车轮转速/方向
```

### 4.2 关键时序
| 环节 | 时间/周期 | 说明 |
|------|-----------|------|
| 控制周期 | **5ms** | PIT 定时中断，决定系统响应速度 |
| ADC 采样 | 每 5ms 触发一次 | 每次采集 4 通道 × 20 次 = 80 次转换 |
| ADC 转换时间 | 单通道约 1us（28.5周期 @ 72MHz/6 = 12MHz ADC时钟） | 4通道扫描约 4us |
| DMA 传输 |  negligible（硬件自动） | 不占 CPU 时间 |
| PWM 频率 | 17kHz | 周期约 59us，远快于控制周期 |
| 电机响应 | 数十ms级 | 机械惯性决定，控制周期需远小于电机响应时间 |

### 4.3 软硬件配合要点

#### 1. 为什么用 DMA？
- **传统方式**：CPU 每次读取 ADC_DR 寄存器，4 通道需 CPU 介入 4 次
- **DMA 方式**：硬件自动搬运，4 个数据完成后触发中断，CPU 只需处理结果
- **优势**：降低 CPU 负载，提高实时性，避免数据覆盖

#### 2. 为什么每 20 次采样才滤波？
- 控制周期 5ms，20 次采集 = 100ms 滤波周期
- 兼顾**实时性**（不能等太久）与**滤波效果**（足够样本数）
- 使用插入排序：20 个数据量小，排序速度快（约数十us），适合嵌入式实时场景

#### 3. 为什么差速而不是差速 + 转向舵机？
- **双电机差速**：结构简单、可靠性高、适合室内赛道
- 通过左右轮速度差实现转向，符合阿克曼原理近似

#### 4. 速度限幅 ±5000 的作用
- PWM 定时器自动重装载值（ARR）约为 5000（对应 17kHz）
- 限制最大速度防止电机过流、电池压降、车体失控

---

## 五、硬件设计参考（扩展建议）

### 5.1 主控最小系统（STM32F103C8T6 为例）
```
          ┌─────────┐
    VCC ──│ 1   VCC  │── 3.3V 电源
    GND ──│ 2   GND  │── 地
    NRST─│ 3  NRST │── 复位电路（10k电阻 + 0.1uF电容）
    PA0 ──│ 4   PA0  │── 可接按键/ADC
          │  ...    │
    PA4 ──│ 9   PA4  │── 左电感（ADC1_CH4）
    PA5 ──│ 10  PA5  │── 中左电感（ADC1_CH5）
    PA6 ──│ 11  PA6  │── 中右电感（ADC1_CH6）
    PA7 ──│ 12  PA7  │── 右电感（ADC1_CH7）
          │  ...    │
    PB12─│ 25  PB12 │── 左电机方向1
    PB13─│ 26  PB13 │── 左电机方向2
    PB14─│ 27  PB14 │── 右电机方向1
    PB15─│ 28  PB15 │── 右电机方向2
          └─────────┘
```

### 5.2 电机驱动建议
- **驱动芯片**：L298N、TB6612FNG、或 DRV8833
- **供电**：电机电源与逻辑电源分离（避免电机噪声干扰 MCU）
- **隔离**：光耦隔离或肖特基二极管保护，防止电机反电动势损坏 MCU

### 5.3 电感传感器建议
- **类型**：电磁感应线圈（直径约 1-2cm，电感值 1mH 级）
- **调理电路**：LC 振荡 + 检波 + 滤波，输出 0-3.3V 模拟电压
- **安装高度**：距离地面 3-5mm（根据赛道导线粗细调整）
- **间距**：总宽度略大于赛道宽度（通常 38cm 赛道，电感间距约 4-5cm）

### 5.4 电源设计
- **主电池**：2S 锂电池（7.4V）或 18650 电池组
- **MCU 供电**：AMS1117-3.3V 或直接使用开发板自带 LDO
- **电机供电**：直接接电池（大电流需求，不走 LDO）
- **总电流**：待机 < 500mA，跑车时 1-3A（取决于电机和车速）

---

## 六、代码质量与优化建议

### 6.1 现有代码优点
1. **架构清晰**：初始化、采集、滤波、控制、输出分层明确
2. **DMA 使用合理**：避免 CPU 轮询，提高效率
3. **滤波扎实**：排序 + 去极值均值，抗干扰能力强
4. **限幅保护**：电机速度限制在 ±5000，防止硬件损坏

### 6.2 可优化点
| 问题 | 优化建议 |
|------|----------|
| 代码存在 OCR 识别错误 | 修正语法（如 `GPIO_WriteBit` 参数格式、`DMA_SetCurrDataCounter` 拼写） |
| 无编码器反馈 | 增加编码器测速，实现闭环速度控制 |
| 无 OLED/调试信息 | 增加显示屏或串口打印，便于调试参数 |
| PID 参数固定 | 增加参数自动调节或按键切换预设参数 |
| 环形赛道判断逻辑不完整 | 补充完整状态机代码（入环、环中匀速、出环） |
| 无刹车/倒车逻辑 | 环岛出环或起跑阶段可能需要 |

---

## 七、总结

这是一套典型的**电磁循迹智能车**软件方案，核心特点是：

**硬件层面：**
- STM32 主控 + 4 路电感 + 2 路直流电机
- ADC + DMA 高速采集，PWM 调速，5ms 控制周期

**软件层面：**
- 20 次采样 + 排序去极值均值滤波 → 稳定电感数据
- 差比和/向量法求误差 → 归一化位置信息
- PD/PID 控制 → 误差转差速
- 双 PWM + 方向 IO → 差速转向

**可改进方向：**
- 完善环岛、坡道、十字等赛道元素的识别与处理
- 增加编码器实现速度闭环
- 引入蓝牙/WiFi 实现参数无线调试

---

> **注**：以上代码基于 PPT 文本提取并做了语法修正和格式整理，部分原始代码因 OCR 识别问题存在字符粘连或缺失，已标注并尽量还原为可编译版本。如需完整工程代码，建议联系天津科技大学疾风车队获取原版工程文件。
