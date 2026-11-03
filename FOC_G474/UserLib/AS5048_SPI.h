#ifndef AS5048_SPI_H
#define AS5048_SPI_H

/*
 * AS5048A PWM 角度读取接口。
 *
 * 文件名保留为 AS5048_SPI.h，是为了兼容当前工程文件名；本版本不使用 SPI，
 * 而是把 AS5048A 的 PWM 输出接到 TIM1_CH1（PC0），用 TIM 的 PWM 输入模式读取：
 *   - TIM1 从模式 Reset，触发源 TI1FP1：每个上升沿清零计数器；
 *   - CH1 直接输入，上升沿捕获 -> 完整周期；
 *   - CH2 间接输入，下降沿捕获 -> 高电平时间；
 *   - 预分频 3，计数频率 42.5MHz，约1ms的周期不会溢出。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

typedef struct
{
    TIM_HandleTypeDef *htim; /* 连接 AS5048A PWM 的定时器句柄。 */

    /* 以下变量在捕获中断中更新。 */
    volatile uint32_t high_ticks;   /* 最近一帧高电平计数值。 */
    volatile uint32_t period_ticks; /* 最近一帧完整周期计数值。 */
    volatile uint16_t angle_raw;    /* 最近一次有效角度，按14位格式（0..16380）。 */
    volatile uint8_t  valid;        /* 1: 至少解码成功过一帧。 */
    volatile uint8_t  error;        /* 1: 最近一帧无效（角度保持上一次有效值）。 */
    volatile uint32_t frame_count;  /* 有效帧计数，不增长说明信号丢失。 */
    volatile uint32_t error_count;  /* 无效帧计数，用于调试。 */
} AS5048A_HandleTypeDef;

/* 初始化驱动句柄；调用前须完成 MX_TIM1_Init()。 */
void AS5048A_Init(AS5048A_HandleTypeDef *sensor, TIM_HandleTypeDef *htim);

/* 启动捕获：CH1 开中断，CH2 只捕获。 */
HAL_StatusTypeDef AS5048A_Start(AS5048A_HandleTypeDef *sensor);

/* 停止捕获。 */
HAL_StatusTypeDef AS5048A_Stop(AS5048A_HandleTypeDef *sensor);

/* 在 HAL_TIM_IC_CaptureCallback() 中调用。 */
void AS5048A_CaptureCallback(AS5048A_HandleTypeDef *sensor, TIM_HandleTypeDef *htim);

/* 最近一次有效的等效14位角度值，范围 0..16380。 */
uint16_t AS5048A_GetAngleRaw(const AS5048A_HandleTypeDef *sensor);

/* 最近一次有效的机械角度，范围 [0, 2π)。无效帧时保持上一次的有效值。 */
float AS5048A_ReadAngleRad(const AS5048A_HandleTypeDef *sensor);

/* 是否已经得到过有效角度。 */
uint8_t AS5048A_IsValid(const AS5048A_HandleTypeDef *sensor);

/* 调试用：最近一帧的高电平和周期计数值。 */
uint32_t AS5048A_GetHighTicks(const AS5048A_HandleTypeDef *sensor);
uint32_t AS5048A_GetPeriodTicks(const AS5048A_HandleTypeDef *sensor);

#ifdef __cplusplus
}
#endif

#endif
