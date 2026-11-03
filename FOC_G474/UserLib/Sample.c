#include "Sample.h"
#include "adc.h"

extern uint16_t dma_buffer[];

// DRV8323RHRGZR 内部集成三相下桥臂电流采样：采样电阻 10mΩ，增益 40V/V，直流偏置约 1.65V，ADC 参考 3.3V
#define CURRENT_SCALE   (3.3f / 4096.0f / 40.0f / 0.01f)   // A/LSB ≈ 2.01mA
#define OFFSET_SAMPLES  1024U                              // 20kHz 下约 51ms

// 零电流时的 ADC 读数，初值为理论 1.65V 对应的 2048，上电后用实测值替换
static float    offset[3] = { 2048.0f, 2048.0f, 2048.0f };
static uint32_t acc[3]    = { 0U, 0U, 0U };
static uint32_t acc_cnt   = 0U;
static volatile uint8_t offset_done = 0U;

/**
 * @brief 零电流状态下累加原始 ADC 值，满 OFFSET_SAMPLES 次后求平均作为每相零偏
 */
void CurrentOffset_Accumulate(void)
{
    if (offset_done) return;

    acc[0] += HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    acc[1] += HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
    acc[2] += HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);

    if (++acc_cnt >= OFFSET_SAMPLES) {
        for (int i = 0; i < 3; i++) {
            offset[i] = (float)acc[i] / (float)OFFSET_SAMPLES;
        }
        offset_done = 1U;
    }
}

uint8_t CurrentOffset_Done(void)
{
    return offset_done;
}

/**
 * @brief 获取三相电流采样值（已减去实测零偏），单位 [A]
 * @param sample 指向 CurrentSample_t 结构体的指针
 */
void CurrentSample(CurrentSample_t *sample)
{
    sample->Ia = ((float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1) - offset[0]) * CURRENT_SCALE;
    sample->Ib = ((float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2) - offset[1]) * CURRENT_SCALE;
    sample->Ic = ((float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3) - offset[2]) * CURRENT_SCALE;
}

float busVoltage(void)
{
    return (float)dma_buffer[0] * 3.3f / 4096.0f;
}
