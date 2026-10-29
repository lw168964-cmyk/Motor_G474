#ifndef __ALGORITHM_H
#define __ALGORITHM_H

//Clarke变换结构体
typedef struct {
    float ua;// A相电压
    float ub;// B相电压
    float uc;// C相电压

    float u_alpha;// Alpha-axis voltage
    float u_beta;// Beta-axis voltage
} ClarkeTransform_t;

//Park变换结构体
typedef struct {
    float u_alpha;// Alpha-axis voltage
    float u_beta;// Beta-axis voltage

    float u_d;// D-axis voltage
    float u_q;// Q-axis voltage
} ParkTransform_t;

// FOC 电角度结构体
typedef struct
{
    float elec_angle;      // 当前电角度 [rad]
    float elec_omega;      // 电角速度 [rad/s]
    float ts;              // 采样周期 [s]
} ElecAngle_t;

void ClarkeTransform(ClarkeTransform_t *clarke);
void InverseClarkeTransform(ClarkeTransform_t *clarke);
void ParkTransform(ParkTransform_t *park, float theta);
void InverseParkTransform(ParkTransform_t *park, float theta);
void ElecAngle_Gen(ElecAngle_t *angle);
#endif
