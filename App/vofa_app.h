#ifndef __VOFA_APP_H
#define __VOFA_APP_H

#include "main.h"
#include "current_sense.h"

#define VOFA_SEND_PERIOD_MS   5

void VOFA_Init(void);
void VOFA_Task(CurrentSense_t *cur);

#endif
