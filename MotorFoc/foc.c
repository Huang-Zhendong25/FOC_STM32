#include "foc.h"
#include "tim.h"
#include <math.h>

/* SVPWM per-unit and timer period constants, matching TIM1 configuration */
#define FOC_SVPWM_KM_BACKW     0.1443376f          /* 1 / (12V * sqrt(3)/3) */
#define FOC_TIM1_PERIOD        8400
#define FOC_TIM1_PERIOD_HALF   (FOC_TIM1_PERIOD / 2)

static uint8_t s_openloop_enabled = 0;   /* 0 = PWM disabled, 1 = PWM enabled */
static uint8_t s_lock_axis = 0;          /* 0 = lock to alpha axis, 1 = lock to beta axis */
static float   s_theta_deg = 0.0f;       /* open-loop electrical angle in degrees */

/* Enable TIM1 three-phase complementary PWM outputs */
void FOC_MotorPwmStart(void)
{
    TIM1->CCER |= 0x5555;                          /* enable CH1/CH2/CH3 CCxE and CCxNE */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* Enable TIM1 update interrupt for the fast current loop */
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

/* Disable three-phase PWM outputs */
void FOC_MotorPwmStop(void)
{
    TIM1->CCER &= 0xAAAA;                          /* disable CH1/CH2/CH3 CCxE and CCxNE */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    /* Disable TIM1 update interrupt */
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
}

/* Convert alpha/beta voltage to seven-segment SVPWM and write TIM1 CCRs */
void FOC_SvpwmUpdate(float ualpha, float ubeta)
{
    float u1, u2, u3;
    float Ta, Tb, Tc;
    uint8_t sector = 3;

    u1 = ubeta;
    u2 = ubeta * 0.5f + ualpha * 0.8660254f;
    u3 = u2 - u1;

    /* Determine sector */
    sector = (u2 > 0) ? (sector - 1) : sector;
    sector = (u3 > 0) ? (sector - 1) : sector;
    sector = (u1 < 0) ? (7 - sector) : sector;

    /* Calculate three-phase modulation waves Ta/Tb/Tc */
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

    /* Map per-unit voltage to CCR centered at 50 percent duty */
    TIM1->CCR1 = (uint16_t)(Ta * FOC_SVPWM_KM_BACKW * FOC_TIM1_PERIOD_HALF + FOC_TIM1_PERIOD_HALF);
    TIM1->CCR2 = (uint16_t)(Tb * FOC_SVPWM_KM_BACKW * FOC_TIM1_PERIOD_HALF + FOC_TIM1_PERIOD_HALF);
    TIM1->CCR3 = (uint16_t)(Tc * FOC_SVPWM_KM_BACKW * FOC_TIM1_PERIOD_HALF + FOC_TIM1_PERIOD_HALF);
}

void FOC_OpenLoop_Init(void)
{
    s_openloop_enabled = 0;
    s_lock_axis = 0;
    s_theta_deg = 0.0f;
    FOC_SvpwmUpdate(0.0f, 0.0f);   /* start with zero voltage vector */
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
        FOC_SvpwmUpdate(0.0f, 0.0f);   /* return to zero vector */
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

/* Call every 1 ms: output zero vector when disabled, otherwise lock or rotate */
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
        FOC_SvpwmUpdate(FOC_OPENLOOP_U_AMP, 0.0f);   /* lock to alpha axis */
    }
    else
    {
        FOC_SvpwmUpdate(0.0f, FOC_OPENLOOP_U_AMP);   /* lock to beta axis */
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
