#ifndef __FOC_CURRENT_H
#define __FOC_CURRENT_H

#include "main.h"

#define FOC_CURRENT_IQ_REF_DEFAULT   0.2f
#define FOC_CURRENT_IQ_STEP          0.05f
#define FOC_CURRENT_IQ_MAX           2.0f

void FOC_Current_Init(void);
void FOC_Current_Enable(void);
void FOC_Current_Disable(void);
uint8_t FOC_Current_IsEnabled(void);
void FOC_Current_SetIqRef(float iq_ref);
void FOC_Current_IncreaseIq(void);
void FOC_Current_DecreaseIq(void);
void FOC_Current_Run(void);

float FOC_Current_GetId(void);
float FOC_Current_GetIq(void);
float FOC_Current_GetVd(void);
float FOC_Current_GetVq(void);
float FOC_Current_GetIqRef(void);

#endif
