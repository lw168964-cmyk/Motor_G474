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
//编码器：AS5048A PWM输出约1kHz，TIM1计数频率42.5MHz（见AS5048A_SPI.c）
#define ENC_TIM_CLK_HZ    42500000.0f
#define ENC_DT_DEFAULT    0.001f              //周期读数异常时使用的帧周期[s]
#define RAD_S_TO_RPM      (60.0f / TWO_PI_F)

//电机：Rs=10.32Ω，Ls=3.26mH，额定0.9A，14对极；2V对应约0.19A，电压不要超过9V
volatile float align_ud   = 2.0f;             //对齐/校验阶段d轴电压[V]
volatile float check_freq = 4.0f;             //校验阶段开环电频率[Hz]，4Hz对应机械约17rpm
volatile float foc_uq     = 1.0f;             //电压模式：RUN阶段q轴电压[V]，正转为正，反转为负
volatile uint8_t foc_restart = 0;             //调试器中置1：重新执行对齐和校验

//电流环参数（可在调试器中在线修改）
volatile uint8_t foc_mode = 3;                //控制模式：0=电压，1=电流，2=速度，3=位置
volatile float id_ref = 0.0f;                 //d轴目标电流[A]，id=0控制（最大转矩/电流比）
volatile float iq_ref = 0.2f;                 //q轴目标电流[A]，控制转矩（额定0.9A）
//电流环增益按零极点对消整定：Kp = Ls·ωc，Ki = Rs·ωc，ωc = 1000 rad/s
//之前反馈用的是5ms低通后的电流，滤波极点让电流环相位裕度只有约25°，所以有超调，
//当时在线调到Kp≈2.5只是压住了超调。现在反馈改为未滤波电流，按公式取值即可
volatile float current_kp = 3.26f;            //电流环Kp = 0.00326 × 1000
volatile float current_ki = 10320.0f;         //电流环Ki = 10.32 × 1000

//速度环参数（可在调试器中在线修改）
volatile float rpm_ref = 50.0f;              //目标转速[rpm]
//位置模式下三环等效为位置PID（θ单位rad，ω单位rad/s，9.55 = 60/2π）：
//  刚度 P = speed_kp·position_kp + 9.55·speed_ki    [A/rad]
//  阻尼 D = 9.55·speed_kp                           [A/(rad/s)]
//  积分 I = speed_ki·position_kp                    [A/(rad·s)]
//阻尼比约正比于 D/√P：来回摆说明阻尼不够，应加大speed_kp、减小speed_ki
volatile float speed_kp = 0.002f;             //速度环Kp，输出单位[A/rpm]
volatile float speed_ki = 0.01f;              //速度环Ki，输出单位[A/(rpm·s)]
volatile float speed_lpf_tau = 0.005f;        //转速低通时间常数[s]，越大越平滑、延迟越大
//手拨偏离时速度误差大，积分会一直攒到电流限幅0.9A，松手后这部分电流要先“放掉”，就会过冲
//限制积分项只够克服齿槽转矩和摩擦即可
volatile float speed_i_limit = 0.3f;          //速度环积分项限幅[A]

//位置环参数（可在调试器中在线修改）
volatile float pos_ref = 0.0f;                //目标位置[rad]，机械角度
volatile float position_kp = 80.0f;           //位置环Kp，输出单位[rpm/rad]
volatile float position_ki = 0.0f;            //位置环Ki（位置环只用P，静差由速度环积分消除）

volatile FocDebug_t foc_dbg;

CurrentSample_t current_sample = { .Ia = 0.0f, .Ib = 0.0f, .Ic = 0.0f };  //三相电流采样值[A]
uint16_t dma_buffer[DMA_BUFFER_SIZE];

extern AS5048A_HandleTypeDef as5048a;          //main.c 中定义

//d轴和q轴电流PI控制器
static PI_Controller_t pi_id;
static PI_Controller_t pi_iq;

//速度环PI控制器
static PI_Controller_t pi_speed;

//位置环PI控制器
static PI_Controller_t pi_position;

//状态机内部变量（只在ADC2注入回调中访问）
static FocState_t state = FOC_STATE_ALIGN;
static uint32_t   tick = 0;                    //当前阶段已运行的控制周期数
static float      ol_angle = 0.0f;             //开环电角度（未回卷，用于统计总增量）
static float      mech_prev = 0.0f;            //上一次机械角
static float      mech_accum = 0.0f;           //CHECK第一段机械角累计增量
static float      diff_sum = 0.0f, diff_min = 0.0f, diff_max = 0.0f;
static uint32_t   diff_n = 0;
//转速估计：每收到一帧编码器数据计算一次
static uint32_t   enc_frames_prev = 0;         //上次处理时的帧计数（有效帧+无效帧）
static float      enc_angle_prev = 0.0f;       //上一帧的机械角[rad]
static uint8_t    enc_started = 0;             //0：还没有基准帧
//外环输出，在两帧编码器数据之间保持不变
static float      iq_cmd_hold = 0.0f;

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
  pi->i_limit = (out_max > -out_min) ? out_max : -out_min;   //默认不额外限制
  pi->out_min = out_min;
  pi->out_max = out_max;
}

//PI控制器更新（条件积分抗饱和）
float PI_Update(PI_Controller_t *pi, float error, float dt)
{
  float p_term, output, integral_new;

  p_term = pi->kp * error;

  //ki=0时清掉积分，避免之后改ki时积分值突然生效
  if (pi->ki == 0.0f)
  {
    pi->integral = 0.0f;
  }

  //先试算本次积分，输出未饱和、或误差方向能让输出退出饱和时才接受
  integral_new = pi->integral + error * dt;
  //积分项限幅：|ki·integral| ≤ i_limit
  if (pi->ki > 0.0f)
  {
    float lim = pi->i_limit / pi->ki;
    if (integral_new > lim) integral_new = lim;
    else if (integral_new < -lim) integral_new = -lim;
  }
  output = p_term + pi->ki * integral_new;

  if (output > pi->out_max)
  {
    if (error < 0.0f) pi->integral = integral_new;
    output = pi->out_max;
  }
  else if (output < pi->out_min)
  {
    if (error > 0.0f) pi->integral = integral_new;
    output = pi->out_min;
  }
  else
  {
    pi->integral = integral_new;
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

  //初始化速度环PI控制器，输出限幅为额定电流0.9A
  PI_Init(&pi_speed, speed_kp, speed_ki, -0.9f, 0.9f);

  //初始化位置环PI控制器，输出限幅为最大转速±500rpm
  PI_Init(&pi_position, position_kp, position_ki, -500.0f, 500.0f);

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

//三相电流 -> Clarke -> Park(theta_e)
//未滤波值存入 foc_dbg.id_raw/iq_raw，作电流环反馈；低通值存入 foc_dbg.id/iq，只用于显示
static void Update_Idq(float theta_e)
{
  ClarkeTransform_t clarke = { .ua = current_sample.Ia, .ub = current_sample.Ib, .uc = current_sample.Ic };
  ParkTransform_t park;

  ClarkeTransform(&clarke);
  park.u_alpha = clarke.u_alpha;
  park.u_beta  = clarke.u_beta;
  ParkTransform(&park, theta_e);

  foc_dbg.id_raw = park.u_d;
  foc_dbg.iq_raw = park.u_q;

  //一阶低通，时间常数约 5ms，只用于显示，不要用作控制反馈
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
  float outer_dt = ENC_DT_DEFAULT;   //本次外环更新的时间间隔[s]
  uint8_t outer_update = 0;          //1：本周期收到新编码器帧，位置环/速度环更新一次
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

  //转速：每来一帧新编码器数据（约1ms）算一次差分，再一阶低通
  //原来按固定窗口统计，窗口内只有10~50帧，12位分辨率下量化噪声大、延迟长，速度环增益上不去
  {
    uint32_t frames = as5048a.frame_count + as5048a.error_count;
    if (!enc_started)
    {
      enc_frames_prev = frames;
      enc_angle_prev = theta_m;
      enc_started = 1;
    }
    else if (frames != enc_frames_prev)
    {
      uint32_t n = frames - enc_frames_prev;   //中间有无效帧时n>1，角度增量跨了n个周期
      uint32_t per = as5048a.period_ticks;
      float frame_dt = (per >= 36000U && per <= 50000U) ? ((float)per / ENC_TIM_CLK_HZ) : ENC_DT_DEFAULT;
      float dt = frame_dt * (float)n;
      float rpm_raw = WrapPi(theta_m - enc_angle_prev) / dt * RAD_S_TO_RPM;
      float alpha = dt / (speed_lpf_tau + dt);

      foc_dbg.rpm += alpha * (rpm_raw - foc_dbg.rpm);
      enc_frames_prev = frames;
      enc_angle_prev = theta_m;
      outer_dt = dt;
      outer_update = 1;
    }
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

      //支持在线调参
      pi_id.kp = current_kp;   pi_id.ki = current_ki;
      pi_iq.kp = current_kp;   pi_iq.ki = current_ki;
      pi_speed.kp = speed_kp;  pi_speed.ki = speed_ki;  pi_speed.i_limit = speed_i_limit;
      pi_position.kp = position_kp;  pi_position.ki = position_ki;

      //不用的外环保持复位，切换模式时从零开始
      if (foc_mode != FOC_MODE_POSITION)
      {
        PI_Reset(&pi_position);
      }
      if ((foc_mode != FOC_MODE_POSITION) && (foc_mode != FOC_MODE_SPEED))
      {
        PI_Reset(&pi_speed);
        iq_cmd_hold = 0.0f;
      }

      //外环（位置、速度）跟随编码器帧率运行，约1kHz；两帧之间保持输出
      //编码器数据1ms才更新一次，20kHz运行外环只会重复用同一个角度，没有意义
      if (outer_update && ((foc_mode == FOC_MODE_POSITION) || (foc_mode == FOC_MODE_SPEED)))
      {
        float rpm_cmd;

        if (foc_mode == FOC_MODE_POSITION)
        {
          float pos_err = WrapPi(pos_ref - theta_m);   //最短路径误差[-π, π)
          foc_dbg.pos_err = pos_err;
          rpm_cmd = PI_Update(&pi_position, pos_err, outer_dt);
        }
        else
        {
          rpm_cmd = rpm_ref;
        }
        foc_dbg.rpm_cmd = rpm_cmd;

        iq_cmd_hold = PI_Update(&pi_speed, rpm_cmd - foc_dbg.rpm, outer_dt);
        foc_dbg.iq_cmd = iq_cmd_hold;
      }

      if (foc_mode == FOC_MODE_VOLTAGE)
      {
        //电压模式：直接使用设定的电压值
        ud_out = 0.0f;
        uq_out = foc_uq;
        PI_Reset(&pi_id);
        PI_Reset(&pi_iq);
      }
      else
      {
        //电流环：电流模式用iq_ref，速度/位置模式用外环输出
        float iq_target = (foc_mode == FOC_MODE_CURRENT) ? iq_ref : iq_cmd_hold;
        ud_out = PI_Update(&pi_id, id_ref - foc_dbg.id_raw, PWM_TS);
        uq_out = PI_Update(&pi_iq, iq_target - foc_dbg.iq_raw, PWM_TS);
      }
      foc_dbg.mode = foc_mode;

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
