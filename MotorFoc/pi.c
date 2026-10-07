#include "pi.h"

void PI_Init(PI_t *pi, float kp, float ki, float ts, float out_max, float out_min)
{
    pi->Kp = kp;
    pi->Ki = ki;
    pi->Ts = ts;
    pi->Ref = 0.0f;
    pi->Fbk = 0.0f;
    pi->Err = 0.0f;
    pi->Integrator = 0.0f;
    pi->Out = 0.0f;
    pi->OutMax = out_max;
    pi->OutMin = out_min;
}

void PI_Reset(PI_t *pi)
{
    pi->Ref = 0.0f;
    pi->Fbk = 0.0f;
    pi->Err = 0.0f;
    pi->Integrator = 0.0f;
    pi->Out = 0.0f;
}

void PI_Update(PI_t *pi)
{
    pi->Err = pi->Ref - pi->Fbk;

    pi->Integrator += pi->Ki * pi->Err * pi->Ts;
    if (pi->Integrator > pi->OutMax)
    {
        pi->Integrator = pi->OutMax;
    }
    else if (pi->Integrator < pi->OutMin)
    {
        pi->Integrator = pi->OutMin;
    }

    pi->Out = pi->Kp * pi->Err + pi->Integrator;
    if (pi->Out > pi->OutMax)
    {
        pi->Out = pi->OutMax;
    }
    else if (pi->Out < pi->OutMin)
    {
        pi->Out = pi->OutMin;
    }
}
