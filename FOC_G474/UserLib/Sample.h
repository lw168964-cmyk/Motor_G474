#ifndef __SAMPLE_H
#define __SAMPLE_H

#include <stdint.h>

typedef struct
{
    float Ia;
    float Ib;
    float Ic;
} CurrentSample_t;

void    CurrentSample(CurrentSample_t *sample);
void    CurrentOffset_Accumulate(void);   // 零偏标定期间在 ADC 注入回调里调用
uint8_t CurrentOffset_Done(void);
float   busVoltage(void);

#endif // __SAMPLE_H
