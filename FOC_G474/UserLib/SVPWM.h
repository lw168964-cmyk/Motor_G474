#ifndef __SVPWM_H
#define __SVPWM_H

#include <stdint.h>

/* Convert alpha/beta voltage commands to three PWM compare values. */
void svpwm(float u_alpha, float u_beta,
           float u_dc, float Ts, uint16_t cmp[3]);

#endif
