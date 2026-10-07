#include "foc.h"
#include "tim.h"
#include <math.h>

/* SVPWM ????????? TIM1 ???? */
#define FOC_SVPWM_KM_BACKW     0.1443376f          /* 1 / (12V * sqrt(3)/3) */
#define FOC_TIM1_PERIOD        8400
#define FOC_TIM1_PERIOD_HALF   (FOC_TIM1_PERIOD / 2)

static uint8_t s_openloop_enabled = 0;   /* 0 = PWM ????1 = ??? */
static uint8_t s_lock_axis = 0;          /* 0 = ? ????1 = ? ??? */
static float   s_theta_deg = 0.0f;       /* ????????????? */

/* ?? TIM1 ???/?? PWM ????? + ?????? */
void FOC_MotorPwmStart(void)
{
    TIM1->CCER |= 0x5555;                          /* ?? CH1/CH2/CH3 ? CCxE ? CCxNE */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/* ???? PWM ?? */
void FOC_MotorPwmStop(void)
{
    TIM1->CCER &= 0xAAAA;                          /* ?? CH1/CH2/CH3 ? CCxE ? CCxNE */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

/* Alpha/Beta ?? -> ??? SVPWM -> TIM1 ????? */
void FOC_SvpwmUpdate(float ualpha, float ubeta)
{
    float u1, u2, u3;
    float Ta, Tb, Tc;
    uint8_t sector = 3;

    /* ? Clarke?Alpha/Beta ???????????? */
    u1 = ubeta;
    u2 = ubeta * 0.5f + ualpha * 0.8660254f;
    u3 = u2 - u1;

    /* ???? */
    sector = (u2 > 0) ? (sector - 1) : sector;
    sector = (u3 > 0) ? (sector - 1) : sector;
    sector = (u1 < 0) ? (7 - sector) : sector;

    /* ???????????? Ta/Tb/Tc */
    if ((sector == 1) || (sector == 4))
    {
        Ta = u2;
        Tb = u1 - u3;
        Tc = -u2;
    }
    else if ((sector == 2) || (sector == 5))
    {
        Ta = u3 + u2;
        Tb = u1;
        Tc = -u1;
    }
    else if ((sector == 3) || (sector == 6))
    {
        Ta = u3;
        Tb = -u3;
        Tc = -(u1 + u2);
    }
    else
    {
        Ta = 0.0f;
        Tb = 0.0f;
        Tc = 0.0f;
    }

    /* ??? -> ? 50% ??????? CCR ? */
    TIM1->CCR1 = (uint16_t)(Ta * FOC_SVPWM_KM_BACKW * FOC_TIM1_PERIOD_HALF + FOC_TIM1_PERIOD_HALF);
    TIM1->CCR2 = (uint16_t)(Tb * FOC_SVPWM_KM_BACKW * FOC_TIM1_PERIOD_HALF + FOC_TIM1_PERIOD_HALF);
    TIM1->CCR3 = (uint16_t)(Tc * FOC_SVPWM_KM_BACKW * FOC_TIM1_PERIOD_HALF + FOC_TIM1_PERIOD_HALF);
}

void FOC_OpenLoop_Init(void)
{
    s_openloop_enabled = 0;
    s_lock_axis = 0;
    s_theta_deg = 0.0f;
    FOC_SvpwmUpdate(0.0f, 0.0f);   /* ????????? */
}

void FOC_OpenLoop_Enable(void)
{
    if (s_openloop_enabled == 0)
    {
        FOC_MotorPwmStart();
        s_openloop_enabled = 1;
    }
}

void FOC_OpenLoop_Disable(void)
{
    if (s_openloop_enabled == 1)
    {
        FOC_MotorPwmStop();
        s_openloop_enabled = 0;
        s_theta_deg = 0.0f;
        FOC_SvpwmUpdate(0.0f, 0.0f);   /* ????? */
    }
}

uint8_t FOC_OpenLoop_IsEnabled(void)
{
    return s_openloop_enabled;
}

void FOC_OpenLoop_ToggleAxis(void)
{
    s_lock_axis = (s_lock_axis == 0) ? 1 : 0;
}

/* ?? 1ms ????????????????????????????? */
void FOC_OpenLoop_Run(void)
{
    if (s_openloop_enabled == 0)
    {
        FOC_SvpwmUpdate(0.0f, 0.0f);
        return;
    }

#if FOC_STAGE1_LOCK_TEST
    if (s_lock_axis == 0)
    {
        FOC_SvpwmUpdate(FOC_OPENLOOP_U_AMP, 0.0f);   /* ? ??? */
    }
    else
    {
        FOC_SvpwmUpdate(0.0f, FOC_OPENLOOP_U_AMP);   /* ? ??? */
    }
#else
    {
        float rad;
        s_theta_deg += FOC_OPENLOOP_ELEC_HZ * 360.0f * 0.001f;
        if (s_theta_deg >= 360.0f)
        {
            s_theta_deg -= 360.0f;
        }
        rad = s_theta_deg * 3.14159265f / 180.0f;
        FOC_SvpwmUpdate(FOC_OPENLOOP_U_AMP * cosf(rad), FOC_OPENLOOP_U_AMP * sinf(rad));
    }
#endif
}
