#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>

//FOC运行阶段
typedef enum
{
  FOC_STATE_ALIGN = 0,   //零位对齐：Ud加在电角度0上，转子吸到a轴，记录机械角θm0
  FOC_STATE_CHECK,       //校验：开环转14个电周期，测方向、极对数、角度差
  FOC_STATE_RUN,         //电压模式FOC：用编码器电角度，Ud=0，Uq=foc_uq
  FOC_STATE_FAULT        //校验失败，输出零电压
} FocState_t;

//控制模式
typedef enum
{
  FOC_MODE_VOLTAGE = 0,  //电压模式：直接设置Ud、Uq
  FOC_MODE_CURRENT       //电流模式：电流环闭环控制Id、Iq
} FocMode_t;

//PI控制器
typedef struct
{
  float kp;              //比例增益
  float ki;              //积分增益
  float integral;        //积分累积值
  float out_max;         //输出上限
  float out_min;         //输出下限
} PI_Controller_t;

//调试数据，在ADC2注入回调中更新，主循环读取显示
typedef struct
{
  uint8_t state;         //当前阶段 FocState_t
  uint8_t mode;          //控制模式 FocMode_t
  uint8_t fault;         //故障码：0无，1编码器无效，2编码器不动/极对数不符
  float   theta_m;       //当前机械角[rad]
  float   theta_m0;      //机械零位[rad]（对齐值经正反转校验修正后）
  float   theta_m0_raw;  //对齐阶段直接读到的机械零位[rad]
  int8_t  dir;           //编码器方向：+1同向，-1反向
  float   pole_ratio;    //开环电角度增量/编码器机械角增量，应约为±14
  float   diff;          //CHECK阶段实时角度差：开环角度-编码器电角度[rad]
  float   diff_fwd;      //正转时角度差平均值[rad]
  float   diff_rev;      //反转时角度差平均值[rad]
  float   diff_mean;     //正反转平均，即零位误差[rad]，已用于修正theta_m0
  float   diff_span;     //CHECK阶段角度差最大值-最小值[rad]
  float   id;            //d轴电流（低通后）[A]
  float   iq;            //q轴电流（低通后）[A]
  float   rpm;           //机械转速[rpm]
  float   ud;            //d轴电压输出[V]
  float   uq;            //q轴电压输出[V]

  //诊断量
  uint32_t isr_cnt;      //ADC2回调执行次数，正常每秒增加20000
  uint32_t state_tick;   //当前阶段已运行的控制周期数
  uint32_t isr_cycles;   //回调最长执行时间[CPU周期]，应远小于8500（50us）
  uint32_t reset_csr;    //上电时RCC->CSR，用于判断复位原因
} FocDebug_t;

extern volatile FocDebug_t foc_dbg;

//可在调试器中在线修改
extern volatile float align_ud;      //对齐/校验电压[V]
extern volatile float check_freq;    //校验阶段开环电频率[Hz]
extern volatile float foc_uq;        //电压模式：RUN阶段q轴电压[V]
extern volatile uint8_t foc_restart; //置1重新对齐和校验

//电流环控制参数
extern volatile uint8_t foc_mode;    //控制模式：0=电压模式，1=电流模式
extern volatile float id_ref;        //d轴目标电流[A]，通常设为0（id=0控制）
extern volatile float iq_ref;        //q轴目标电流[A]，控制转矩
extern volatile float current_kp;    //电流环比例增益
extern volatile float current_ki;    //电流环积分增益

void Driver_Init(void);
void Driver_Enable(void);
void PI_Init(PI_Controller_t *pi, float kp, float ki, float out_min, float out_max);
float PI_Update(PI_Controller_t *pi, float error, float dt);
void PI_Reset(PI_Controller_t *pi);

#endif
