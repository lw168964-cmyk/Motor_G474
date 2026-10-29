# AS5048A PWM 输入捕获配置说明

本说明对应 `AS5048A_SPI.c` 和 `AS5048_SPI.h`。虽然文件名保留了 `SPI`，
当前驱动已经改为使用 AS5048A 的 PWM 输出，不需要 SPI 外设、SPI 时钟、
片选或寄存器读写。

## 1. AS5048A PWM 协议

根据 AS5048A/AS5048B Datasheet Rev. 1.3 的 PWM Interface：

| 区段 | 时钟数 | 正常状态 |
| --- | ---: | --- |
| Init | 12 | 高电平 |
| Error_n | 4 | 高电平 |
| Data | 4095 | 按角度改变高电平时间 |
| Exit | 8 | 低电平 |

完整 PWM 周期为 `4119` 个 PWM 时钟，PWM 数据分辨率为 `4096`。驱动同时
测量高电平时间和完整周期，使用下面的比例恢复数据值：

```text
data = high_time / period * 4119 - 16
```

然后把 `data`（0 到 4095）换算为等效 14 位角度值 `data * 4`。错误状态
下 Error_n 区段不输出高电平，驱动会将该帧标记为无效。

PWM 是芯片上电后的独立输出，不需要先接 SPI，也不需要写寄存器才能启用。

## 2. 当前硬件连接

当前工程和网表对应的连接为：

| AS5048A PWM 接口 | STM32G474RET6 工程 |
| --- | --- |
| PWM | `PC0 / TIM1_CH1` |
| 供电 | 网表 `CN3.2` 为 `+5V` |
| 地 | 网表 `CN3.3` 为 `GND` |

`PC0` 在 STM32G474RE 数据手册中属于 FT 输入类型，可用于 TIM1_CH1；仍需
确认外部 AS5048A 模块的 PWM 输出电平和实际供电方式，不能把未知电压信号
直接接入 MCU。

## 3. CubeMX 配置

以下项目由用户在 CubeMX 中配置，驱动不会修改 `.ioc` 或 CubeMX 生成文件。

### TIM1 基本参数

| 项目 | 建议值 |
| --- | --- |
| Channel 1 | `Input Capture direct mode` / `Input_Capture1_from_TI1` |
| Prescaler | `169` |
| Counter Mode | `Up` |
| Counter Period | `65535` |
| Clock Division | `DIV1` |
| Repetition Counter | `0` |
| Auto-reload preload | `Disable` |

当前系统时钟和 TIM1 输入时钟为 `170 MHz`，预分频 `169` 后计数频率为：

```text
170 MHz / (169 + 1) = 1 MHz
```

即 1 个计数约等于 1 us。AS5048A PWM 周期约 1 ms，使用 `65535` 的自动重装
值可以覆盖正常周期，并允许一次计数器回绕。

### TIM1 CH1 输入捕获参数

| 项目 | 建议值 |
| --- | --- |
| IC Polarity | `Rising` |
| IC Selection | `Direct TI` |
| IC Prescaler | `DIV1` |
| IC Filter | `0`；若示波器确认有毛刺，可改为小值 |
| GPIO | `PC0`, `AF2_TIM1`, `No Pull` |

第一次捕获必须从上升沿开始。驱动在回调中自动把 CH1 极性切换为：

```text
上升沿 -> 下降沿 -> 上升沿 -> 下降沿 -> ...
```

### NVIC 配置

打开：

```text
TIM1 Capture Compare interrupt
```

CubeMX 生成的中断函数必须最终调用：

```c
HAL_TIM_IRQHandler(&htim1);
```

不要配置 TIM1 Update 中断来代替 Capture Compare 中断。

## 4. 应用层调用

在 `MX_TIM1_Init()` 执行完成后初始化并启动驱动：

```c
#include "AS5048_SPI.h"
#include "tim.h"

static AS5048A_HandleTypeDef g_as5048a;

AS5048A_Init(&g_as5048a, &htim1);
if (AS5048A_Start(&g_as5048a) != HAL_OK) {
    Error_Handler();
}
```

在用户代码区添加 HAL 回调转发：

```c
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    AS5048A_CaptureCallback(&g_as5048a, htim);
}
```

FOC 任务或速度环中读取最近一次有效角度：

```c
if (AS5048A_IsValid(&g_as5048a) != 0U) {
    float mechanical_angle_rad = AS5048A_ReadAngleRad(&g_as5048a);
    uint16_t angle_raw = AS5048A_GetAngleRaw(&g_as5048a);
    (void)mechanical_angle_rad;
    (void)angle_raw;
}
```

不要在 `HAL_TIM_IC_CaptureCallback()` 中执行 OLED 刷屏、`printf` 或复杂的
FOC 运算；回调只负责保存捕获值和解码角度。

## 5. 当前工程还需要确认的问题

这些问题不在 AS5048A 驱动文件中，需用户在 CubeMX 或应用代码中处理：

1. 当前 `tim.c` 仍是 `Prescaler = 0`，请改为 `169`；当前 `main.c` 也没有
   调用 `HAL_TIM_IC_Start_IT()`，应通过 `AS5048A_Start()` 启动。
2. 当前中断文件没有 `TIM1_CC_IRQHandler()`，必须启用 CubeMX 的 TIM1
   Capture Compare 中断并重新生成代码。
3. ADC2 已配置 HRTIM 注入触发，但 `main.c` 当前启动的是 `hadc1` 的注入组；
   ADC1 没有注入组，应改为按实际用途启动 ADC1 规则组，并启动 ADC2 注入组。
4. ADC1 的 DMA 当前为 `DMA_NORMAL`，且应用代码未看到 `HAL_ADC_Start_DMA()`；
   若要连续采样母线电压，需要改为循环模式并启动 DMA。
5. 网表有 `PB0` 连接驱动器 `nFAULT`，当前 `.ioc` 未配置 PB0；如需故障保护，
   应将 PB0 配成输入或 EXTI，并按驱动器有效电平设置上拉/下拉。
6. FDCAN2 当前生成参数约为 `3.54 Mbit/s`，且标准过滤器数量为 `0`；如目标
   是常用的 500 kbit/s 或 1 Mbit/s，需要在 CubeMX 中重新设置位时序和过滤器。
7. `PC12` 是网表中的 `ENABLE`，当前复位初始化为低电平；请确认驱动器的
   使能有效电平和上电安全状态。

## 6. 驱动接口

| 函数 | 作用 |
| --- | --- |
| `AS5048A_Init()` | 保存 TIM 句柄并复位捕获状态 |
| `AS5048A_Start()` | 启动 TIM1_CH1 输入捕获中断 |
| `AS5048A_Stop()` | 停止 TIM1_CH1 输入捕获中断 |
| `AS5048A_CaptureCallback()` | 在 HAL 捕获回调中处理一个边沿 |
| `AS5048A_IsValid()` | 判断最近一次角度是否有效 |
| `AS5048A_GetAngleRaw()` | 获取等效 14 位原始角度 |
| `AS5048A_ReadAngleRad()` | 获取机械角度弧度值 |

零位偏移、旋转方向和电机极对数不属于 AS5048A PWM 解码层，应在 FOC 应用层
完成。
