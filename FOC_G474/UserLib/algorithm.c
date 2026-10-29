#include "algorithm.h"
#include "arm_math.h"

#define SQRT3 1.7320508076f

/**
 * @brief Clarke变换函数,将三相电压转换为两相电压（等幅值变换）
 * @param clarke: 指向ClarkeTransform_t结构体的指针,包含输入和输出值
 */
void ClarkeTransform(ClarkeTransform_t *clarke)
{
    clarke->u_alpha = (2.0f / 3.0f) * (clarke->ua - 0.5f * clarke->ub - 0.5f * clarke->uc);
    clarke->u_beta = (2.0f / 3.0f) * ((SQRT3 / 2.0f) * (clarke->ub - clarke->uc));
}

/**
 * @brief Inverse Clarke变换函数,将两相电压转换为三相电压
 * @param clarke: 指向ClarkeTransform_t结构体的指针,包含输入和输出值
 */
void InverseClarkeTransform(ClarkeTransform_t *clarke)
{
    clarke->ua = clarke->u_alpha;
    clarke->ub = -0.5f * clarke->u_alpha + (SQRT3 / 2.0f) * clarke->u_beta;
    clarke->uc = -0.5f * clarke->u_alpha - (SQRT3 / 2.0f) * clarke->u_beta;
}

/// @brief Park变换函数，将两相电压转换为旋转坐标系下的电压(d轴初始与a轴重合)
/// @param park: 指向ParkTransform_t结构体的指针，包含输入和输出值
/// @param theta: 转子角度
void ParkTransform(ParkTransform_t *park, float theta)
{
    float cos_theta = arm_cos_f32(theta);
    float sin_theta = arm_sin_f32(theta);

    park->u_d = park->u_alpha * cos_theta + park->u_beta * sin_theta;
    park->u_q = -park->u_alpha * sin_theta + park->u_beta * cos_theta;
}

/// @brief Inverse Park变换函数，将旋转坐标系下的电压转换为两相电压
/// @param park: 指向ParkTransform_t结构体的指针，包含输入和输出值
/// @param theta: 转子角度
void InverseParkTransform(ParkTransform_t *park, float theta)
{
    float cos_theta = arm_cos_f32(theta);
    float sin_theta = arm_sin_f32(theta);

    park->u_alpha = park->u_d * cos_theta - park->u_q * sin_theta;
    park->u_beta = park->u_d * sin_theta + park->u_q * cos_theta;
}


/**
 * @brief  生成自旋电角度，每次采样调用
 * @param  angle: 电角度结构体指针
 * @retval 无
 */
void ElecAngle_Gen(ElecAngle_t *angle)
{
    // 角度累加
    angle->elec_angle += angle->elec_omega * angle->ts;

    // 角度回卷到 [0, 2π)
    const float two_pi = 2.0f * PI;
    if(angle->elec_angle >= two_pi)
    {
        angle->elec_angle -= two_pi;
    }
    else if(angle->elec_angle < 0.0f)
    {
        angle->elec_angle += two_pi;
    }
}


