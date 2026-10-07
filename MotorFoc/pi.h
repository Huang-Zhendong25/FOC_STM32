#ifndef __PI_H
#define __PI_H

#include "main.h"

typedef struct
{
    float Kp;
    float Ki;
    float Ts;
    float Ref;
    float Fbk;
    float Err;
    float Integrator;
    float Out;
    float OutMax;
    float OutMin;
} PI_t;

void PI_Init(PI_t *pi, float kp, float ki, float ts, float out_max, float out_min);
void PI_Reset(PI_t *pi);
void PI_Update(PI_t *pi);

#endif
