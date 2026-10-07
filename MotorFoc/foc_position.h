#ifndef __FOC_POSITION_H
#define __FOC_POSITION_H

#include "main.h"

#define FOC_POSITION_KP            0.3f
#define FOC_POSITION_OUT_MAX       3000.0f
#define FOC_POSITION_STEP          100
#define FOC_POSITION_MAX            3999

void FOC_Position_Init(void);
void FOC_Position_Enable(void);
void FOC_Position_Disable(void);
uint8_t FOC_Position_IsEnabled(void);
void FOC_Position_IncreaseTarget(void);
void FOC_Position_DecreaseTarget(void);
void FOC_Position_Run(void);

int32_t FOC_Position_GetTarget(void);
int32_t FOC_Position_GetFbk(void);
float FOC_Position_GetError(void);

#endif
