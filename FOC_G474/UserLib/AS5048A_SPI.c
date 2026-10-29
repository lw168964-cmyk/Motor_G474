/**
 * @file AS5048A_SPI.c
 * @brief AS5048A PWM 角度读取驱动。
 *
 * 文件名保留为 AS5048A_SPI.c，是为了兼容当前工程；本版本不再通过 SPI
 * 访问寄存器，而是使用 AS5048A 的 PWM 引脚输出读取机械角度。
 *
 * AS5048A 数据手册中的 PWM 帧由以下部分组成：
 *   - Init：12 个 PWM 时钟，高电平；
 *   - Error_n：4 个 PWM 时钟，正常时为高电平；
 *   - Data：4095 个 PWM 时钟，编码角度；
 *   - Exit：8 个 PWM 时钟，低电平。
 *
 * 因此完整周期为 4119 个 PWM 时钟，PWM 数据分辨率为 4096。驱动通过
 * 同时测量完整周期和高电平时间消除芯片 PWM 时钟 ±10% 误差的影响。
 */

#include "AS5048_SPI.h"

/* AS5048A PWM 帧参数，来自数据手册 PWM Interface / Figure 14 / Table 17。 */
#define AS5048A_PWM_INIT_CLOCKS        12ULL
#define AS5048A_PWM_ERROR_CLOCKS        4ULL
#define AS5048A_PWM_DATA_CLOCKS      4095ULL
#define AS5048A_PWM_EXIT_CLOCKS         8ULL
#define AS5048A_PWM_PERIOD_CLOCKS    (AS5048A_PWM_INIT_CLOCKS + \
                                      AS5048A_PWM_ERROR_CLOCKS + \
                                      AS5048A_PWM_DATA_CLOCKS + \
                                      AS5048A_PWM_EXIT_CLOCKS)
#define AS5048A_PWM_DATA_OFFSET_CLOCKS \
                                     (AS5048A_PWM_INIT_CLOCKS + \
                                      AS5048A_PWM_ERROR_CLOCKS)
#define AS5048A_ANGLE_RESOLUTION    16384.0f
#define AS5048A_TWO_PI                  6.28318530717958647692f

/*
 * 计算两个捕获值之间的计数差。
 * TIM1 配置为向上计数，允许在自动重装值处回绕一次。
 */
static uint32_t AS5048A_CaptureDelta(const AS5048A_HandleTypeDef *sensor,
                                     uint32_t current,
                                     uint32_t previous)
{
    uint32_t counter_range;

    counter_range = __HAL_TIM_GET_AUTORELOAD(sensor->htim) + 1U;
    if (current >= previous) {
        return current - previous;
    }

    return counter_range - previous + current;
}

/*
 * 根据完整周期和高电平时间恢复 PWM 数据计数。
 *
 * data_clocks = high_time / period * 4119 - 16
 *
 * 用比例计算而不是直接使用定时器计数值，可以消除 AS5048A PWM 时钟
 * 频率误差。错误状态下 Error_n 不输出高电平，测得的高电平约为12个
 * PWM 时钟，小于正常帧的16个时钟，因此直接判为无效。
 */
static uint8_t AS5048A_DecodePwm(const AS5048A_HandleTypeDef *sensor,
                                 uint32_t high_ticks,
                                 uint32_t period_ticks,
                                 uint16_t *angle_raw)
{
    uint64_t high_clock_x_period;
    uint64_t high_clocks;
    uint64_t data_clocks;
    uint64_t raw_angle;

    if ((period_ticks == 0U) || (high_ticks >= period_ticks)) {
        return 0U;
    }

    /* 先把高电平时间换算为 PWM 时钟数，采用四舍五入。 */
    high_clock_x_period = (uint64_t)high_ticks * AS5048A_PWM_PERIOD_CLOCKS;
    high_clocks = (high_clock_x_period + (period_ticks / 2U)) /
                  period_ticks;

    /* 正常 PWM 在 Init + Error_n 阶段至少保持16个时钟为高电平。 */
    if (high_clocks < AS5048A_PWM_DATA_OFFSET_CLOCKS) {
        return 0U;
    }

    /* 超出最大正常高电平范围，说明捕获边沿或输入信号异常。 */
    if (high_clocks > (AS5048A_PWM_DATA_OFFSET_CLOCKS +
                       AS5048A_PWM_DATA_CLOCKS)) {
        return 0U;
    }

    data_clocks = high_clocks - AS5048A_PWM_DATA_OFFSET_CLOCKS;

    /*
     * PWM 输出分辨率是12位（0..4095），等效14位角度每个 PWM 计数对应
     * 4个14位 LSB。这样角度范围为0..16380，对应0..360度未闭区间。
     */
    raw_angle = data_clocks << 2U;
    if (raw_angle >= (uint64_t)AS5048A_ANGLE_RESOLUTION) {
        raw_angle = (uint64_t)AS5048A_ANGLE_RESOLUTION - 4ULL;
    }

    *angle_raw = (uint16_t)raw_angle;
    (void)sensor;
    return 1U;
}

/* 初始化驱动句柄，默认从上升沿开始捕获。 */
void AS5048A_Init(AS5048A_HandleTypeDef *sensor,
                  TIM_HandleTypeDef *htim)
{
    if ((sensor == NULL) || (htim == NULL)) {
        return;
    }

    sensor->htim = htim;
    sensor->rising_capture = 0U;
    sensor->high_ticks = 0U;
    sensor->period_ticks = 0U;
    sensor->angle_raw = 0U;
    sensor->valid = 0U;
    sensor->error = 0U;
    sensor->capture_state = AS5048A_CAPTURE_WAIT_RISING;

    /* 初始边沿必须和 CubeMX 中的输入捕获极性保持一致。 */
    __HAL_TIM_SET_CAPTUREPOLARITY(htim,
                                  TIM_CHANNEL_1,
                                  TIM_INPUTCHANNELPOLARITY_RISING);
}

/* 启动 TIM1_CH1 输入捕获中断。 */
HAL_StatusTypeDef AS5048A_Start(AS5048A_HandleTypeDef *sensor)
{
    if ((sensor == NULL) || (sensor->htim == NULL)) {
        return HAL_ERROR;
    }

    sensor->capture_state = AS5048A_CAPTURE_WAIT_RISING;
    sensor->valid = 0U;
    sensor->error = 0U;
    __HAL_TIM_SET_CAPTUREPOLARITY(sensor->htim,
                                  TIM_CHANNEL_1,
                                  TIM_INPUTCHANNELPOLARITY_RISING);

    return HAL_TIM_IC_Start_IT(sensor->htim, TIM_CHANNEL_1);
}

/* 停止 TIM1_CH1 输入捕获中断。 */
HAL_StatusTypeDef AS5048A_Stop(AS5048A_HandleTypeDef *sensor)
{
    if ((sensor == NULL) || (sensor->htim == NULL)) {
        return HAL_ERROR;
    }

    return HAL_TIM_IC_Stop_IT(sensor->htim, TIM_CHANNEL_1);
}

/*
 * 处理 TIM 输入捕获回调。
 *
 * 由于只使用 CH1，一个 PWM 周期需要按“上升 -> 下降 -> 上升”顺序
 * 交替修改捕获极性：
 *   - 第一个上升沿：记录周期起点；
 *   - 下降沿：得到高电平时间；
 *   - 下一个上升沿：得到完整周期并解码上一帧。
 */
void AS5048A_CaptureCallback(AS5048A_HandleTypeDef *sensor,
                             TIM_HandleTypeDef *htim)
{
    uint32_t capture;
    uint32_t period;
    uint16_t angle_raw;

    if ((sensor == NULL) || (sensor->htim == NULL) ||
        (htim != sensor->htim) ||
        (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1)) {
        return;
    }

    capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    if (sensor->capture_state == AS5048A_CAPTURE_WAIT_RISING) {
        sensor->rising_capture = capture;
        sensor->capture_state = AS5048A_CAPTURE_WAIT_FALLING;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim,
                                      TIM_CHANNEL_1,
                                      TIM_INPUTCHANNELPOLARITY_FALLING);
        return;
    }

    if (sensor->capture_state == AS5048A_CAPTURE_WAIT_FALLING) {
        sensor->high_ticks = AS5048A_CaptureDelta(sensor,
                                                  capture,
                                                  sensor->rising_capture);
        sensor->capture_state = AS5048A_CAPTURE_WAIT_NEXT_RISING;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim,
                                      TIM_CHANNEL_1,
                                      TIM_INPUTCHANNELPOLARITY_RISING);
        return;
    }

    period = AS5048A_CaptureDelta(sensor,
                                  capture,
                                  sensor->rising_capture);
    sensor->period_ticks = period;

    if (AS5048A_DecodePwm(sensor,
                          sensor->high_ticks,
                          period,
                          &angle_raw) != 0U) {
        sensor->angle_raw = angle_raw;
        sensor->valid = 1U;
        sensor->error = 0U;
    } else {
        sensor->valid = 0U;
        sensor->error = 1U;
    }

    /* 当前上升沿同时作为下一帧的周期起点。 */
    sensor->rising_capture = capture;
    sensor->capture_state = AS5048A_CAPTURE_WAIT_FALLING;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim,
                                  TIM_CHANNEL_1,
                                  TIM_INPUTCHANNELPOLARITY_FALLING);
}

/* 获取最近一次有效的等效14位角度值。 */
uint16_t AS5048A_GetAngleRaw(const AS5048A_HandleTypeDef *sensor)
{
    if ((sensor == NULL) || (sensor->valid == 0U)) {
        return 0U;
    }

    return sensor->angle_raw;
}

/* 获取最近一次有效的机械角度，返回范围为0到2π弧度。 */
float AS5048A_ReadAngleRad(const AS5048A_HandleTypeDef *sensor)
{
    if ((sensor == NULL) || (sensor->valid == 0U)) {
        return 0.0f;
    }

    return (float)sensor->angle_raw * AS5048A_TWO_PI /
           AS5048A_ANGLE_RESOLUTION;
}

/* 判断最近一次捕获是否有效。 */
uint8_t AS5048A_IsValid(const AS5048A_HandleTypeDef *sensor)
{
    return ((sensor != NULL) && (sensor->valid != 0U)) ? 1U : 0U;
}

/* 获取最近一次 PWM 高电平时间。 */
uint32_t AS5048A_GetHighTicks(const AS5048A_HandleTypeDef *sensor)
{
    return (sensor != NULL) ? sensor->high_ticks : 0U;
}

/* 获取最近一次 PWM 完整周期时间。 */
uint32_t AS5048A_GetPeriodTicks(const AS5048A_HandleTypeDef *sensor)
{
    return (sensor != NULL) ? sensor->period_ticks : 0U;
}
