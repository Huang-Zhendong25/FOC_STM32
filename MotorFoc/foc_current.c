#include "foc_current.h"
#include "foc.h"
#include "tim.h"
#include "current_sense.h"
#include "clarke.h"
#include "park.h"
#include "encoder.h"
#include "pi.h"

#define FOC_PI_TS        0.00005f
#define FOC_PI_KP        0.3f
#define FOC_PI_KI        1000.0f
#define FOC_PI_MAX_U     6.0f

extern CurrentSense_t g_current;
extern Encoder_t g_encoder;

static uint8_t s_enabled = 0;
static float s_iq_ref = FOC_CURRENT_IQ_REF_DEFAULT;
static PI_t s_pi_id;
static PI_t s_pi_iq;
static CurrentSense_t s_cur;

static float s_id = 0.0f;
static float s_iq = 0.0f;
static float s_vd = 0.0f;
static float s_vq = 0.0f;

void FOC_Current_Init(void)
{
    s_enabled = 0;
    s_iq_ref = FOC_CURRENT_IQ_REF_DEFAULT;

    PI_Init(&s_pi_id, FOC_PI_KP, FOC_PI_KI, FOC_PI_TS, FOC_PI_MAX_U, -FOC_PI_MAX_U);
    PI_Init(&s_pi_iq, FOC_PI_KP, FOC_PI_KI, FOC_PI_TS, FOC_PI_MAX_U, -FOC_PI_MAX_U);
}

void FOC_Current_Enable(void)
{
    if (s_enabled == 0)
    {
        FOC_MotorPwmStart();
        s_enabled = 1;
    }
}

void FOC_Current_Disable(void)
{
    if (s_enabled == 1)
    {
        FOC_MotorPwmStop();
        s_enabled = 0;
        PI_Reset(&s_pi_id);
        PI_Reset(&s_pi_iq);
    }
}

uint8_t FOC_Current_IsEnabled(void)
{
    return s_enabled;
}

void FOC_Current_IncreaseIq(void)
{
    s_iq_ref += FOC_CURRENT_IQ_STEP;
    if (s_iq_ref > FOC_CURRENT_IQ_MAX)
    {
        s_iq_ref = FOC_CURRENT_IQ_MAX;
    }
}

void FOC_Current_DecreaseIq(void)
{
    s_iq_ref -= FOC_CURRENT_IQ_STEP;
    if (s_iq_ref < -FOC_CURRENT_IQ_MAX)
    {
        s_iq_ref = -FOC_CURRENT_IQ_MAX;
    }
}

void FOC_Current_Run(void)
{
    float theta;
    float id;
    float iq;
    float valpha;
    float vbeta;

    if (s_enabled == 0)
    {
        return;
    }

    CurrentSense_UpdateFromInjected(&s_cur);
    Clarke_Update(&s_cur);

    theta = Encoder_GetElecAngle();
    Park_Update(s_cur.Ialpha, s_cur.Ibeta, theta, &id, &iq);

    s_id = id;
    s_iq = iq;

    s_pi_id.Ref = 0.0f;
    s_pi_id.Fbk = id;
    PI_Update(&s_pi_id);

    s_pi_iq.Ref = s_iq_ref;
    s_pi_iq.Fbk = iq;
    PI_Update(&s_pi_iq);

    s_vd = s_pi_id.Out;
    s_vq = s_pi_iq.Out;

    InvPark_Update(s_vd, s_vq, theta, &valpha, &vbeta);
    FOC_SvpwmUpdate(valpha, vbeta);

    g_current = s_cur;
}

float FOC_Current_GetId(void) { return s_id; }
float FOC_Current_GetIq(void) { return s_iq; }
float FOC_Current_GetIqRef(void) { return s_iq_ref; }
float FOC_Current_GetVd(void) { return s_vd; }
float FOC_Current_GetVq(void) { return s_vq; }

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        FOC_Current_Run();
    }
}
