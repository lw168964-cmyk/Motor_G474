#ifndef AS5048_SPI_H
#define AS5048_SPI_H

/*
 * AS5048A PWM 角度读取接口。
 *
 * 文件名保留为 AS5048_SPI.h，是为了兼容当前工程文件名；本版本不再
 * 使用 SPI，而是使用 AS5048A 的 PWM 输出连接到 TIM1_CH1。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* 单通道输入捕获的工作阶段。 */
typedef enum
{
    AS5048A_CAPTURE_WAIT_RISING = 0U, /* 等待一帧开始的上升沿。 */
    AS5048A_CAPTURE_WAIT_FALLING,     /* 已捕获上升沿，等待高电平结束。 */
    AS5048A_CAPTURE_WAIT_NEXT_RISING  /* 已捕获下降沿，等待下一帧开始。 */
} AS5048A_CaptureStateTypeDef;

typedef struct
{
    TIM_HandleTypeDef *htim; /* 连接 AS5048A PWM 的定时器句柄。 */

    /* 以下变量会在中断回调中更新，主循环读取时保持最近一次有效值。 */
    volatile uint32_t rising_capture; /* 当前 PWM 周期起始计数值。 */
    volatile uint32_t high_ticks;     /* PWM 高电平持续计数值。 */
    volatile uint32_t period_ticks;   /* PWM 完整周期计数值。 */
    volatile uint16_t angle_raw;      /* 按14位格式保存的等效角度值。 */
    volatile uint8_t valid;           /* 1: 最近一次角度有效，0: 尚未有效。 */
    volatile uint8_t error;           /* 1: 捕获到错误脉宽或计时异常。 */

    AS5048A_CaptureStateTypeDef capture_state; /* 当前边沿捕获状态。 */
} AS5048A_HandleTypeDef;

/* 初始化驱动句柄；调用前须完成 TIM1 和 GPIO 的 CubeMX 初始化。 */
void AS5048A_Init(AS5048A_HandleTypeDef *sensor,
                  TIM_HandleTypeDef *htim);

/* 启动 TIM 输入捕获中断；CubeMX 必须同时打开 TIM1 Capture Compare 中断。 */
HAL_StatusTypeDef AS5048A_Start(AS5048A_HandleTypeDef *sensor);

/* 停止 TIM 输入捕获中断。 */
HAL_StatusTypeDef AS5048A_Stop(AS5048A_HandleTypeDef *sensor);

/*
 * 在 HAL_TIM_IC_CaptureCallback() 中调用本函数。
 * 该函数使用 TIM1_CH1 一个通道交替捕获上升沿和下降沿，计算 PWM 周期
 * 以及高电平时间。
 */
void AS5048A_CaptureCallback(AS5048A_HandleTypeDef *sensor,
                             TIM_HandleTypeDef *htim);

/* 获取最近一次有效的等效14位角度值，范围为 0 到 16380。 */
uint16_t AS5048A_GetAngleRaw(const AS5048A_HandleTypeDef *sensor);

/* 获取最近一次有效的机械角度，范围为 0 到 2π 弧度。 */
float AS5048A_ReadAngleRad(const AS5048A_HandleTypeDef *sensor);

/* 判断是否已经捕获到有效 PWM 角度。 */
uint8_t AS5048A_IsValid(const AS5048A_HandleTypeDef *sensor);

/* 获取最近一次捕获的 PWM 高电平计数值和完整周期计数值，便于调试。 */
uint32_t AS5048A_GetHighTicks(const AS5048A_HandleTypeDef *sensor);
uint32_t AS5048A_GetPeriodTicks(const AS5048A_HandleTypeDef *sensor);

#ifdef __cplusplus
}
#endif

#endif
