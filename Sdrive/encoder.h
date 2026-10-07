#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

#define ENCODER_PPR         1024
#define ENCODER_CPR         (ENCODER_PPR * 4)
#define MOTOR_POLE_PAIRS    10
#define ENCODER_DIRECTION   1       /* 1 = normal, -1 = reverse angle/speed direction */

typedef struct
{
    uint32_t raw_count;
    int32_t  last_count;
    int32_t  delta_count;
    float    mech_angle_rad;
    float    elec_angle_rad;
    float    speed_rpm;
} Encoder_t;

void Encoder_Init(void);
void Encoder_Update(Encoder_t *enc);
void Encoder_SetZero(Encoder_t *enc);
float Encoder_GetElecAngle(void);

#endif
