#ifndef __FOC_H
#define __FOC_H

#include "main.h"

/* ??1 ?????? */
#define FOC_STAGE1_LOCK_TEST   1       /* 1 = ?????0 = ?????? */
#define FOC_OPENLOOP_U_AMP     1.0f    /* ?? ????????? V */
#define FOC_OPENLOOP_ELEC_HZ   1.0f    /* ?????????? Hz */

void FOC_MotorPwmStart(void);
void FOC_MotorPwmStop(void);
void FOC_SvpwmUpdate(float ualpha, float ubeta);

void FOC_OpenLoop_Init(void);
void FOC_OpenLoop_Enable(void);
void FOC_OpenLoop_Disable(void);
uint8_t FOC_OpenLoop_IsEnabled(void);
void FOC_OpenLoop_ToggleAxis(void);
void FOC_OpenLoop_Run(void);

#endif
