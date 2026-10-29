#include "control.h"
#include "main.h"
#include "adc.h"
#include "hrtim.h"
#include "arm_math.h"
#include "algorithm.h"
#include "SVPWM.h"

//PWM参数：HRTIM 170MHz×4，增减计数周期17000，完整周期34000计数 = 20kHz
#define PWM_HALF_PERIOD   17000
#define PWM_FULL_PERIOD   34000.0f
#define PWM_TS            (1.0f / 20000.0f)   //控制周期[s]，ADC2由HRTIM Master周期触发
#define CMP_MARGIN        64                  //比较值离边界留余量，避开HRTIM最小比较值限制

#define U_DC              24.0f               //母线电压[V]

//开环参数，可在调试器中在线修改
//电机：Rs=10.32Ω，额定0.9A，14对极；Uq=2V时电流约0.19A，Uq不要超过9V
volatile float openloop_uq   = 2.0f;          //q轴电压[V]
volatile float openloop_freq = 2.0f;          //电频率[Hz]，2Hz对应机械约8.6rpm

static ElecAngle_t elec_angle = { .elec_angle = 0.0f, .elec_omega = 0.0f, .ts = PWM_TS };

static uint16_t Cmp_Limit(uint16_t cmp)
{
  if (cmp < CMP_MARGIN) return CMP_MARGIN;
  if (cmp > PWM_HALF_PERIOD - CMP_MARGIN) return PWM_HALF_PERIOD - CMP_MARGIN;
  return cmp;
}

void Driver_Init(void)
{
  //关闭DRV驱动芯片
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
  //初始化ADC2启动ADC触发中断
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_InjectedStart_IT(&hadc2);
}

void Driver_Enable(void)
{
  //使能DRV驱动芯片
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);

  //启动HRTIM计数器和输出
  HAL_HRTIM_WaveformCounterStart(
    &hhrtim1,
    HRTIM_TIMERID_MASTER |
    HRTIM_TIMERID_TIMER_A |
    HRTIM_TIMERID_TIMER_B |
    HRTIM_TIMERID_TIMER_C
    );

  HAL_HRTIM_WaveformOutputStart(
    &hhrtim1,
    HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
    HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
    HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2);
}

//开环：角度自增 -> 反Park -> SVPWM -> 写HRTIM比较寄存器，20kHz执行
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC2)
  {
    ParkTransform_t park = { .u_d = 0.0f, .u_q = openloop_uq };
    uint16_t cmp[3];

    elec_angle.elec_omega = 2.0f * PI * openloop_freq;
    ElecAngle_Gen(&elec_angle);

    InverseParkTransform(&park, elec_angle.elec_angle);
    svpwm(park.u_alpha, park.u_beta, U_DC, PWM_FULL_PERIOD, cmp);

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, Cmp_Limit(cmp[0]));
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, Cmp_Limit(cmp[1]));
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, Cmp_Limit(cmp[2]));
  }
}
