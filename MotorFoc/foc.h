#ifndef __FOC_H
#define __FOC_H

#include "main.h"

/* Stage 1 open-loop test configuration */
#define FOC_STAGE1_LOCK_TEST   0       /* 1 = lock test, 0 = open-loop rotation */
#define FOC_OPENLOOP_U_AMP     1.0f    /* alpha/beta voltage vector magnitude in V */
#define FOC_OPENLOOP_ELEC_HZ   5.0f    /* open-loop electrical frequency in Hz */

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
