#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

#define ENCODER_PPR         1024
#define ENCODER_CPR         (ENCODER_PPR * 4)
#define MOTOR_POLE_PAIRS    10

typedef struct
{
    uint32_t raw_count;        /* TIM3->CNT raw value */
    int32_t  last_count;       /* previous count used for delta */
    int32_t  delta_count;      /* count delta with wrap handling */
    float    mech_angle_rad;   /* mechanical angle, rad */
    float    elec_angle_rad;   /* electrical angle, rad */
    float    speed_rpm;        /* mechanical speed, rpm */
} Encoder_t;

void Encoder_Init(void);
void Encoder_Update(Encoder_t *enc);
void Encoder_SetZero(Encoder_t *enc);

#endif
