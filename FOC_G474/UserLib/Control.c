#include "control.h"
#include "main.h"
#include "adc.h"
#include "hrtim.h"
#include "arm_math.h"
#include "algorithm.h"
#include "SVPWM.h"
#include "Sample.h"
#include "AS5048_SPI.h"
#include <math.h>

//PWM参数：HRTIM 170MHz×4，增减计数周期17000，完整周期34000计数 = 20kHz
#define PWM_HALF_PERIOD   17000
#define PWM_FULL_PERIOD   34000.0f
#define PWM_TS            (1.0f / 20000.0f)   //控制周期[s]，ADC2由HRTIM Master周期触发
#define PWM_FREQ          20000U
#define CMP_MARGIN        64                  //比较值离边界留余量，避开HRTIM最小比较值限制
#define DMA_BUFFER_SIZE 1024

#define U_DC              24.0f               //母线电压[V]
#define POLE_PAIRS        14                  //电机极对数
#define TWO_PI_F          6.28318530718f

//对齐时序（单位：控制周期，20kHz）
#define ALIGN_PRE_TICKS   (PWM_FREQ / 2U)     //0.5s：电压从0斜坡上升，电角度π/3（预定位，避开180°死点）
#define ALIGN_HOLD_TICKS  (PWM_FREQ)          //1.0s：电角度0，等转子稳定
//校验：每段开环转动的电周期数，14个电周期 = 机械1圈
#define CHECK_CYCLES      POLE_PAIRS
//转速计算窗口：400个控制周期 = 20ms
#define SPEED_WINDOW      400U

//电机：Rs=10.32Ω，Ls=3.26mH，额定0.9A，14对极；2V对应约0.19A，电压不要超过9V
volatile float align_ud   = 2.0f;             //对齐/校验阶段d轴电压[V]
volatile float check_freq = 4.0f;             //校验阶段开环电频率[Hz]，4Hz对应机械约17rpm
volatile float foc_uq     = 1.0f;             //电压模式：RUN阶段q轴电压[V]，正转为正，反转为负
volatile uint8_t foc_restart = 0;             //调试器中置1：重新执行对齐和校验

//电流环参数（可在调试器中在线修改）
volatile uint8_t foc_mode = 1;                //控制模式：0=电压模式，1=电流模式
volatile float id_ref = 0.0f;                 //d轴目标电流[A]，id=0控制（最大转矩/电流比）
volatile float iq_ref = 0.1f;                 //q轴目标电流[A]，控制转矩（额定0.9A）
volatile float current_kp = 4.89f;            //电流环Kp = Ls × ωc = 0.00326 × 1500
volatile float current_ki = 12000.0f;         //电流环Ki = Rs × ωc = 10.32 × 1500

volatile FocDebug_t foc_dbg;

CurrentSample_t current_sample = { .Ia = 0.0f, .Ib = 0.0f, .Ic = 0.0f };  //三相电流采样值[A]
uint16_t dma_buffer[DMA_BUFFER_SIZE];

extern AS5048A_HandleTypeDef as5048a;          //main.c 中定义

//d轴和q轴电流PI控制器
static PI_Controller_t pi_id;
static PI_Controller_t pi_iq;

//状态机内部变量（只在ADC2注入回调中访问）
static FocState_t state = FOC_STATE_ALIGN;
static uint32_t   tick = 0;                    //当前阶段已运行的控制周期数
static float      ol_angle = 0.0f;             //开环电角度（未回卷，用于统计总增量）
static float      mech_prev = 0.0f;            //上一次机械角
static float      mech_accum = 0.0f;           //CHECK第一段机械角累计增量
static float      diff_sum = 0.0f, diff_min = 0.0f, diff_max = 0.0f;
static uint32_t   diff_n = 0;
static float      spd_accum = 0.0f;
static uint32_t   spd_n = 0;

static uint16_t Cmp_Limit(uint16_t cmp)
{
  if (cmp < CMP_MARGIN) return CMP_MARGIN;
  if (cmp > PWM_HALF_PERIOD - CMP_MARGIN) return PWM_HALF_PERIOD - CMP_MARGIN;
  return cmp;
}

//回卷到 [0, 2π)
static float Wrap2Pi(float x)
{
  return x - TWO_PI_F * floorf(x / TWO_PI_F);
}

//回卷到 [-π, π)
static float WrapPi(float x)
{
  return Wrap2Pi(x + PI) - PI;
}

//========== PI控制器 ==========
//初始化PI控制器
void PI_Init(PI_Controller_t *pi, float kp, float ki, float out_min, float out_max)
{
  pi->kp = kp;
  pi->ki = ki;
  pi->integral = 0.0f;
  pi->out_min = out_min;
  pi->out_max = out_max;
}

//PI控制器更新（带抗饱和）
float PI_Update(PI_Controller_t *pi, float error, float dt)
{
  float p_term, i_term, output;

  //比例项
  p_term = pi->kp * error;

  //积分项（梯形积分）
  pi->integral += error * dt;
  i_term = pi->ki * pi->integral;

  //输出
  output = p_term + i_term;

  //抗饱和：输出限幅后反算积分项，防止积分饱和
  if (output > pi->out_max)
  {
    output = pi->out_max;
    pi->integral = (output - p_term) / pi->ki;
  }
  else if (output < pi->out_min)
  {
    output = pi->out_min;
    pi->integral = (output - p_term) / pi->ki;
  }

  return output;
}

//复位PI控制器
void PI_Reset(PI_Controller_t *pi)
{
  pi->integral = 0.0f;
}

//由机械角计算编码器电角度：θe = (dir·(θm − θm0)) × 极对数
static float Encoder_ElecAngle(float theta_m)
{
  float d = Wrap2Pi((float)foc_dbg.dir * (theta_m - foc_dbg.theta_m0));
  return Wrap2Pi(d * (float)POLE_PAIRS);
}

static void Enter_State(FocState_t s)
{
  state = s;
  tick = 0;
  foc_dbg.state = (uint8_t)s;
}

static void Fault(uint8_t code)
{
  foc_dbg.fault = code;
  Enter_State(FOC_STATE_FAULT);
}

static void Reset_StateMachine(void)
{
  ol_angle = 0.0f;
  mech_accum = 0.0f;
  diff_sum = 0.0f;
  diff_min = 1e9f;
  diff_max = -1e9f;
  diff_n = 0;
  foc_dbg.fault = 0;
  foc_dbg.dir = 1;
  foc_dbg.pole_ratio = 0.0f;
  foc_dbg.diff_mean = 0.0f;
  foc_dbg.diff_span = 0.0f;
  Enter_State(FOC_STATE_ALIGN);
}

void Driver_Init(void)
{
  //关闭DRV驱动芯片
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
  //初始化ADC2启动ADC触发中断
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_InjectedStart_IT(&hadc2);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dma_buffer, DMA_BUFFER_SIZE);

  //初始化电流环PI控制器，输出限幅为母线电压
  PI_Init(&pi_id, current_kp, current_ki, -U_DC, U_DC);
  PI_Init(&pi_iq, current_kp, current_ki, -U_DC, U_DC);

  Reset_StateMachine();

  //记录复位原因后清除标志：bit26 PIN(复位脚/调试器)、bit27 BOR(掉电)、bit28 SFT(软件/调试器)、bit29 IWDG、bit30 WWDG
  foc_dbg.reset_csr = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();

  //开启DWT周期计数器，用于测量回调执行时间
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Driver_Enable(void)
{
  //使能DRV驱动芯片
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_Delay(2);  //DRV8323RHRGZR芯片上电后需延时至少1ms再启动HRTIM，否则可能无法输出PWM

  //只启动HRTIM计数器以产生ADC触发，桥臂输出保持关闭，此时相电流为0
  HAL_HRTIM_WaveformCounterStart(
    &hhrtim1,
    HRTIM_TIMERID_MASTER |
    HRTIM_TIMERID_TIMER_A |
    HRTIM_TIMERID_TIMER_B |
    HRTIM_TIMERID_TIMER_C
    );

  //等待电流零偏标定完成（ADC2注入回调中累加1024次，约51ms）
  while (!CurrentOffset_Done()) { }

  //再打开桥臂输出
  HAL_HRTIM_WaveformOutputStart(
    &hhrtim1,
    HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
    HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
    HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2);
}
//反Park -> SVPWM -> 写HRTIM比较寄存器
static void Apply_Voltage(float ud, float uq, float theta_e)
{
  ParkTransform_t park = { .u_d = ud, .u_q = uq };
  uint16_t cmp[3];

  InverseParkTransform(&park, theta_e);
  svpwm(park.u_alpha, park.u_beta, U_DC, PWM_FULL_PERIOD, cmp);

  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, Cmp_Limit(cmp[0]));
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, Cmp_Limit(cmp[1]));
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, Cmp_Limit(cmp[2]));
}

//三相电流 -> Clarke -> Park(theta_e)，低通后存入 foc_dbg.id/iq
static void Update_Idq(float theta_e)
{
  ClarkeTransform_t clarke = { .ua = current_sample.Ia, .ub = current_sample.Ib, .uc = current_sample.Ic };
  ParkTransform_t park;

  ClarkeTransform(&clarke);
  park.u_alpha = clarke.u_alpha;
  park.u_beta  = clarke.u_beta;
  ParkTransform(&park, theta_e);

  //一阶低通，时间常数约 5ms，只用于显示
  foc_dbg.id += 0.01f * (park.u_d - foc_dbg.id);
  foc_dbg.iq += 0.01f * (park.u_q - foc_dbg.iq);
}

//CHECK阶段：每段开环转动 CHECK_CYCLES 个电周期
//  段0 正转：统计机械角总增量 -> 方向、极对数
//  段1 反转：统计角度差 -> diff_rev
//  段2 正转：统计角度差 -> diff_fwd
//正转时转子滞后于电压矢量（diff>0），反转时超前（diff<0），两者平均即可抵消负载角，得到零位误差
static uint8_t check_seg = 0;
static float   seg_progress = 0.0f;           //本段已转过的电角度[rad]（绝对值）

static void Check_StartSegment(uint8_t seg)
{
  check_seg = seg;
  seg_progress = 0.0f;
  diff_sum = 0.0f;
  diff_n = 0;
  diff_min = 1e9f;                            //波动范围只统计最后一段（正转）
  diff_max = -1e9f;
}

static void State_Check(float theta_m, float dm)
{
  float step = TWO_PI_F * check_freq * PWM_TS;  //每个控制周期的电角度增量
  float sign = (check_seg == 1U) ? -1.0f : 1.0f;

  ol_angle = Wrap2Pi(ol_angle + sign * step);
  seg_progress += step;
  Apply_Voltage(align_ud, 0.0f, ol_angle);

  if (check_seg == 0U)
  {
    mech_accum += dm;
  }
  else if (seg_progress > TWO_PI_F)           //跳过每段第一个电周期的过渡过程
  {
    float d = WrapPi(ol_angle - Encoder_ElecAngle(theta_m));
    foc_dbg.diff = d;
    diff_sum += d;
    diff_n++;
    if (d < diff_min) diff_min = d;
    if (d > diff_max) diff_max = d;
  }

  if (seg_progress < TWO_PI_F * (float)CHECK_CYCLES)
  {
    return;
  }

  //本段结束
  if (check_seg == 0U)
  {
    float ol_total = TWO_PI_F * (float)CHECK_CYCLES;
    float ratio;

    //编码器几乎没动：转子没跟上或者编码器没读到
    if (fabsf(mech_accum) < 0.5f)
    {
      Fault(2);
      return;
    }
    ratio = ol_total / mech_accum;
    foc_dbg.pole_ratio = ratio;
    foc_dbg.dir = (ratio > 0.0f) ? 1 : -1;
    if ((fabsf(ratio) < 12.0f) || (fabsf(ratio) > 16.0f))
    {
      Fault(2);
      return;
    }
    Check_StartSegment(1);
  }
  else if (check_seg == 1U)
  {
    foc_dbg.diff_rev = (diff_n > 0U) ? diff_sum / (float)diff_n : 0.0f;
    Check_StartSegment(2);
  }
  else
  {
    foc_dbg.diff_fwd  = (diff_n > 0U) ? diff_sum / (float)diff_n : 0.0f;
    foc_dbg.diff_mean = 0.5f * (foc_dbg.diff_fwd + foc_dbg.diff_rev);
    foc_dbg.diff_span = diff_max - diff_min;

    //修正零位：使编码器电角度 + diff_mean = 真实电角度
    foc_dbg.theta_m0 = Wrap2Pi(foc_dbg.theta_m0 -
                               (float)foc_dbg.dir * foc_dbg.diff_mean / (float)POLE_PAIRS);
    Enter_State(FOC_STATE_RUN);
  }
}

//控制主循环：零位对齐 -> 方向/极对数/零位校验 -> 电压模式FOC，20kHz执行
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  float theta_m, dm;
  uint32_t t0 = DWT->CYCCNT;

  if (hadc->Instance != ADC2)
  {
    return;
  }
  foc_dbg.isr_cnt++;

  //零偏标定期间只累加采样值，不做控制计算
  if (!CurrentOffset_Done())
  {
    CurrentOffset_Accumulate();
    return;
  }

  CurrentSample(&current_sample);

  theta_m = AS5048A_ReadAngleRad(&as5048a);
  dm = WrapPi(theta_m - mech_prev);           //本周期机械角增量（编码器1ms更新一次，多数周期为0）
  mech_prev = theta_m;
  foc_dbg.theta_m = theta_m;

  //转速：20ms窗口内机械角增量
  spd_accum += dm;
  if (++spd_n >= SPEED_WINDOW)
  {
    foc_dbg.rpm = spd_accum / ((float)SPEED_WINDOW * PWM_TS) * (60.0f / TWO_PI_F);
    spd_accum = 0.0f;
    spd_n = 0;
  }

  if (foc_restart)
  {
    foc_restart = 0;
    Reset_StateMachine();
  }

  tick++;

  switch (state)
  {
    case FOC_STATE_ALIGN:
      if (tick <= ALIGN_PRE_TICKS)
      {
        //预定位：电压斜坡上升，电角度 π/3
        Apply_Voltage(align_ud * (float)tick / (float)ALIGN_PRE_TICKS, 0.0f, PI / 3.0f);
      }
      else if (tick <= ALIGN_PRE_TICKS + ALIGN_HOLD_TICKS)
      {
        //对齐：电角度0，转子d轴吸到a相轴线
        Apply_Voltage(align_ud, 0.0f, 0.0f);
        Update_Idq(0.0f);
      }
      else
      {
        if (!AS5048A_IsValid(&as5048a) || (as5048a.error != 0U))
        {
          Fault(1);
          break;
        }
        foc_dbg.theta_m0_raw = theta_m;
        foc_dbg.theta_m0 = theta_m;
        ol_angle = 0.0f;
        mech_accum = 0.0f;
        diff_min = 1e9f;
        diff_max = -1e9f;
        Check_StartSegment(0);
        Enter_State(FOC_STATE_CHECK);
        Apply_Voltage(align_ud, 0.0f, 0.0f);
      }
      break;

    case FOC_STATE_CHECK:
      State_Check(theta_m, dm);
      Update_Idq(ol_angle);
      break;

    case FOC_STATE_RUN:
    {
      float theta_e = Encoder_ElecAngle(theta_m);
      float ud_out, uq_out;

      //先采样电流并计算Id、Iq（用于电流环反馈）
      Update_Idq(theta_e);

      //根据控制模式选择电压输出
      if (foc_mode == FOC_MODE_CURRENT)
      {
        //电流闭环模式：PI控制器计算Ud、Uq
        //更新PI参数（支持在线调整）
        pi_id.kp = current_kp;
        pi_id.ki = current_ki;
        pi_iq.kp = current_kp;
        pi_iq.ki = current_ki;

        //电流环PI控制
        float id_err = id_ref - foc_dbg.id;
        float iq_err = iq_ref - foc_dbg.iq;
        ud_out = PI_Update(&pi_id, id_err, PWM_TS);
        uq_out = PI_Update(&pi_iq, iq_err, PWM_TS);

        foc_dbg.mode = FOC_MODE_CURRENT;
      }
      else
      {
        //电压模式：直接使用设定的电压值
        ud_out = 0.0f;
        uq_out = foc_uq;

        //切换到电压模式时复位PI积分项，避免切回电流模式时积分饱和
        PI_Reset(&pi_id);
        PI_Reset(&pi_iq);

        foc_dbg.mode = FOC_MODE_VOLTAGE;
      }

      //记录实际输出电压
      foc_dbg.ud = ud_out;
      foc_dbg.uq = uq_out;

      Apply_Voltage(ud_out, uq_out, theta_e);
      break;
    }

    case FOC_STATE_FAULT:
    default:
      Apply_Voltage(0.0f, 0.0f, 0.0f);
      break;
  }

  foc_dbg.state_tick = tick;
  {
    uint32_t dt = DWT->CYCCNT - t0;
    if (dt > foc_dbg.isr_cycles) foc_dbg.isr_cycles = dt;
  }
}
