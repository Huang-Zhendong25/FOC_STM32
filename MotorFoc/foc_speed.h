#ifndef __FOC_SPEED_H
#define __FOC_SPEED_H

#include "main.h"

#define FOC_SPEED_TS             0.0001f
#define FOC_SPEED_KP             0.0005f
#define FOC_SPEED_KI             0.01f
#define FOC_SPEED_OUT_MAX        2.0f
#define FOC_SPEED_REF_DEFAULT    300.0f
#define FOC_SPEED_STEP           100.0f
#define FOC_SPEED_MAX_RPM        3000.0f
#define FOC_SPEED_RAMP_RPM_PER_S 500.0f

void FOC_Speed_Init(void);
void FOC_Speed_Enable(void);
void FOC_Speed_Disable(void);
uint8_t FOC_Speed_IsEnabled(void);
void FOC_Speed_IncreaseTarget(void);
void FOC_Speed_DecreaseTarget(void);
void FOC_Speed_Run(void);

float FOC_Speed_GetRefRpm(void);
float FOC_Speed_GetFbRpm(void);

#endif
