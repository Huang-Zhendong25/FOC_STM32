#ifndef __CURRENT_SENSE_H
#define __CURRENT_SENSE_H

#include "main.h"

typedef struct
{
    float Iu;        /* U phase current, A */
    float Iv;        /* V phase current, A */
    float Iw;        /* W phase current, A */
    float Ialpha;    /* Clarke alpha-axis current, A */
    float Ibeta;     /* Clarke beta-axis current, A */
    float Vbus;      /* DC bus voltage, V */
} CurrentSense_t;

extern volatile uint16_t ADC_Value[3];

void CurrentSense_StartDma(void);
void CurrentSense_StartInjectedPolling(void);
void CurrentSense_StartInjectedIT(void);
void CurrentSense_CalibrateOffset(void);
void CurrentSense_UpdateFromInjected(CurrentSense_t *cur);

#endif
