/**
 * @file AS5048A_SPI.c
 * @brief AS5048A PWM 角度读取驱动（TIM1 PWM 输入模式）。
 *
 * 文件名保留为 AS5048A_SPI.c，是为了兼容当前工程；本版本不通过 SPI
 * 访问寄存器，而是读取 AS5048A PWM 引脚输出的机械角度。
 *
 * AS5048A 数据手册中的 PWM 帧由以下部分组成：
 *   - Init：12 个 PWM 时钟，高电平；
 *   - Error_n：4 个 PWM 时钟，正常时为高电平；
 *   - Data：4095 个 PWM 时钟，编码角度；
 *   - Exit：8 个 PWM 时钟，低电平。
 *
 * 完整周期为 4119 个 PWM 时钟（约1ms），数据分辨率为 4096。驱动同时测量
 * 完整周期和高电平时间，用比例计算，消除芯片 PWM 时钟 ±10% 误差的影响。
 *
 * 硬件测量（见 tim.c）：TIM1 从模式 Reset，每个上升沿清零计数器；
 * CCR1 = 周期，CCR2 = 高电平时间，中断里不需要切换极性，也不用处理回绕。
 */

#include "AS5048_SPI.h"

/* AS5048A PWM 帧参数，来自数据手册 PWM Interface。 */
#define AS5048A_PWM_INIT_CLOCKS        12UL
#define AS5048A_PWM_ERROR_CLOCKS        4UL
#define AS5048A_PWM_DATA_CLOCKS      4095UL
#define AS5048A_PWM_EXIT_CLOCKS         8UL
#define AS5048A_PWM_PERIOD_CLOCKS    (AS5048A_PWM_INIT_CLOCKS + \
                                      AS5048A_PWM_ERROR_CLOCKS + \
                                      AS5048A_PWM_DATA_CLOCKS + \
                                      AS5048A_PWM_EXIT_CLOCKS)
#define AS5048A_PWM_DATA_OFFSET_CLOCKS (AS5048A_PWM_INIT_CLOCKS + \
                                        AS5048A_PWM_ERROR_CLOCKS)

/*
 * 周期合法范围（TIM1 计数频率 42.5MHz）：
 * 标称 1ms = 42500 计数，芯片时钟 ±10% -> 38250..46750，再留一些余量。
 * 超出范围说明丢边沿、毛刺或信号断开。
 */
#define AS5048A_PERIOD_MIN_TICKS     36000UL
#define AS5048A_PERIOD_MAX_TICKS     50000UL

#define AS5048A_ANGLE_RESOLUTION     16384.0f
#define AS5048A_TWO_PI               6.28318530717958647692f

/*
 * 根据完整周期和高电平时间恢复 PWM 数据计数：
 *   high_clocks = high_ticks / period_ticks * 4119（四舍五入）
 *   data_clocks = high_clocks - 16
 * 错误状态下 Error_n 为低电平，高电平只有约12个时钟，小于16，判为无效。
 */
static uint8_t AS5048A_DecodePwm(uint32_t high_ticks,
                                 uint32_t period_ticks,
                                 uint16_t *angle_raw)
{
    uint32_t high_clocks;
    uint32_t data_clocks;

    if ((period_ticks < AS5048A_PERIOD_MIN_TICKS) ||
        (period_ticks > AS5048A_PERIOD_MAX_TICKS) ||
        (high_ticks >= period_ticks)) {
        return 0U;
    }

    /* 50000 * 4119 ≈ 2.06e8，在 uint32_t 范围内。 */
    high_clocks = (high_ticks * AS5048A_PWM_PERIOD_CLOCKS + (period_ticks / 2U)) /
                  period_ticks;

    if ((high_clocks < AS5048A_PWM_DATA_OFFSET_CLOCKS) ||
        (high_clocks > (AS5048A_PWM_DATA_OFFSET_CLOCKS + AS5048A_PWM_DATA_CLOCKS))) {
        return 0U;
    }

    data_clocks = high_clocks - AS5048A_PWM_DATA_OFFSET_CLOCKS;
    if (data_clocks > 4095U) {
        data_clocks = 4095U;
    }

    /* 12位 PWM 数据按14位格式保存：每个 PWM 计数对应 4 个14位 LSB。 */
    *angle_raw = (uint16_t)(data_clocks << 2U);
    return 1U;
}

void AS5048A_Init(AS5048A_HandleTypeDef *sensor, TIM_HandleTypeDef *htim)
{
    if ((sensor == NULL) || (htim == NULL)) {
        return;
    }

    sensor->htim         = htim;
    sensor->high_ticks   = 0U;
    sensor->period_ticks = 0U;
    sensor->angle_raw    = 0U;
    sensor->valid        = 0U;
    sensor->error        = 0U;
    sensor->frame_count  = 0U;
    sensor->error_count  = 0U;
}

HAL_StatusTypeDef AS5048A_Start(AS5048A_HandleTypeDef *sensor)
{
    if ((sensor == NULL) || (sensor->htim == NULL)) {
        return HAL_ERROR;
    }

    /* CH2 只捕获不进中断；CH1 每个上升沿进一次中断，此时 CCR1/CCR2 都是上一帧的值。 */
    if (HAL_TIM_IC_Start(sensor->htim, TIM_CHANNEL_2) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_TIM_IC_Start_IT(sensor->htim, TIM_CHANNEL_1);
}

HAL_StatusTypeDef AS5048A_Stop(AS5048A_HandleTypeDef *sensor)
{
    if ((sensor == NULL) || (sensor->htim == NULL)) {
        return HAL_ERROR;
    }

    HAL_TIM_IC_Stop_IT(sensor->htim, TIM_CHANNEL_1);
    return HAL_TIM_IC_Stop(sensor->htim, TIM_CHANNEL_2);
}

void AS5048A_CaptureCallback(AS5048A_HandleTypeDef *sensor, TIM_HandleTypeDef *htim)
{
    uint32_t period;
    uint32_t high;
    uint16_t angle_raw;

    if ((sensor == NULL) || (htim != sensor->htim) ||
        (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1)) {
        return;
    }

    /* 计数器在上升沿被清零，捕获值 + 1 即为计数个数。 */
    period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) + 1U;
    high   = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) + 1U;

    sensor->period_ticks = period;
    sensor->high_ticks   = high;

    if (AS5048A_DecodePwm(high, period, &angle_raw) != 0U) {
        sensor->angle_raw = angle_raw;
        sensor->valid     = 1U;
        sensor->error     = 0U;
        sensor->frame_count++;
    } else {
        /* 无效帧：保持上一次有效角度，只记录错误。 */
        sensor->error = 1U;
        sensor->error_count++;
    }
}

uint16_t AS5048A_GetAngleRaw(const AS5048A_HandleTypeDef *sensor)
{
    return (sensor != NULL) ? sensor->angle_raw : 0U;
}

float AS5048A_ReadAngleRad(const AS5048A_HandleTypeDef *sensor)
{
    if (sensor == NULL) {
        return 0.0f;
    }

    return (float)sensor->angle_raw * (AS5048A_TWO_PI / AS5048A_ANGLE_RESOLUTION);
}

uint8_t AS5048A_IsValid(const AS5048A_HandleTypeDef *sensor)
{
    return ((sensor != NULL) && (sensor->valid != 0U)) ? 1U : 0U;
}

uint32_t AS5048A_GetHighTicks(const AS5048A_HandleTypeDef *sensor)
{
    return (sensor != NULL) ? sensor->high_ticks : 0U;
}

uint32_t AS5048A_GetPeriodTicks(const AS5048A_HandleTypeDef *sensor)
{
    return (sensor != NULL) ? sensor->period_ticks : 0U;
}
