#include "SVPWM.h"

#define SQRT3        1.7320508f
#define SQRT3_DIV_2  0.8660254f

void svpwm(float u_alpha, float u_beta,
            float u_dc, float Ts, uint16_t cmp[3])
{
    /* --- 1. 扇区判断：三个中间量 + N = A + 2B + 4C --- */
    float u_ref1 =  u_beta;
    float u_ref2 =  SQRT3_DIV_2 * u_alpha - 0.5f * u_beta;
    float u_ref3 = -SQRT3_DIV_2 * u_alpha - 0.5f * u_beta;

    int A = (u_ref1 > 0.0f) ? 1 : 0;
    int B = (u_ref2 > 0.0f) ? 1 : 0;
    int C = (u_ref3 > 0.0f) ? 1 : 0;
    int N = A + 2 * B + 4 * C;          /* N ∈ 1..6，查表 1 得扇区号 */

    /* --- 2. 中间量 X / Y / Z（表 2 的原料）--- */
    float k = Ts / u_dc;
    float X =  SQRT3 * k * u_beta;
    float Y =  1.5f * k * u_alpha + SQRT3_DIV_2 * k * u_beta;
    float Z = -1.5f * k * u_alpha + SQRT3_DIV_2 * k * u_beta;

    /* --- 3. 查表 2 取 t1 / t2 --- */
    float t1 = 0.0f, t2 = 0.0f;
    switch (N) {
        case 1: t1 =  Z; t2 =  Y; break;   /* 扇区 Ⅱ */
        case 2: t1 =  Y; t2 = -X; break;   /* 扇区 Ⅵ */
        case 3: t1 = -Z; t2 =  X; break;   /* 扇区 Ⅰ */
        case 4: t1 = -X; t2 =  Z; break;   /* 扇区 Ⅳ */
        case 5: t1 =  X; t2 = -Y; break;   /* 扇区 Ⅲ */
        case 6: t1 = -Y; t2 = -Z; break;   /* 扇区 Ⅴ */
        default: cmp[0] = cmp[1] = cmp[2] = (uint16_t)(Ts * 0.25f); return;   /* 零矢量，取计数中点 */
    }

    /* --- 4. 过调制处理：等比例缩小，保方向、压幅值 --- */
    if (t1 + t2 > Ts) {
        float sum = t1 + t2;
        t1 = t1 / sum * Ts;
        t2 = t2 / sum * Ts;
    }

    /* --- 5. 七段式三个切换点 Ta / Tb / Tc --- */
    float Ta = (Ts - t1 - t2) * 0.25f;   /* = t0 / 4 */
    float Tb = Ta + t1 * 0.5f;
    float Tc = Tb + t2 * 0.5f;

    /* --- 6. 查表 4 装载比较寄存器 --- */
    switch (N) {
        case 1: cmp[0]=Tb; cmp[1]=Ta; cmp[2]=Tc; break;
        case 2: cmp[0]=Ta; cmp[1]=Tc; cmp[2]=Tb; break;
        case 3: cmp[0]=Ta; cmp[1]=Tb; cmp[2]=Tc; break;
        case 4: cmp[0]=Tc; cmp[1]=Tb; cmp[2]=Ta; break;
        case 5: cmp[0]=Tc; cmp[1]=Ta; cmp[2]=Tb; break;
        case 6: cmp[0]=Tb; cmp[1]=Tc; cmp[2]=Ta; break;
    }
}
