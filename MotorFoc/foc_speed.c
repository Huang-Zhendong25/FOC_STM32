#include "foc_speed.h"
#include "foc_current.h"
#include "encoder.h"
#include "pi.h"
#include "tim.h"

#define FOC_SPEED_LPF_A   0.1f
#define FOC_SPEED_LPF_B   0.9f

static uint8_t s_enabled = 0;
static uint8_t s_last_valid = 0;
static int32_t s_last_count = 0;
static float s_target_ref_mech = FOC_SPEED_REF_DEFAULT;
static float s_ramp_ref_mech = 0.0f;
static float s_speed_fb_mech = 0.0f;
static float s_speed_filtered = 0.0f;
static PI_t s_pi_speed;

void FOC_Speed_Init(void)
{
    s_enabled = 0;
    s_last_valid = 0;
    s_last_count = 0;
    s_target_ref_mech = FOC_SPEED_REF_DEFAULT;
    s_ramp_ref_mech = 0.0f;
    s_speed_fb_mech = 0.0f;
    s_speed_filtered = 0.0f;
    PI_Init(&s_pi_speed, FOC_SPEED_KP, FOC_SPEED_KI, FOC_SPEED_TS, FOC_SPEED_OUT_MAX, -FOC_SPEED_OUT_MAX);
}

void FOC_Speed_Enable(void)
{
    if (s_enabled == 0)
    {
        s_ramp_ref_mech = 0.0f;
        s_speed_filtered = 0.0f;
        s_speed_fb_mech = 0.0f;
        s_last_valid = 0;
        PI_Reset(&s_pi_speed);
        FOC_Current_SetIqRef(0.0f);
        FOC_Current_Enable();
        s_enabled = 1;
    }
}

void FOC_Speed_Disable(void)
{
    if (s_enabled == 1)
    {
        FOC_Current_Disable();
        s_enabled = 0;
        s_target_ref_mech = FOC_SPEED_REF_DEFAULT;
        s_ramp_ref_mech = 0.0f;
        s_speed_filtered = 0.0f;
        s_speed_fb_mech = 0.0f;
        FOC_Current_SetIqRef(0.0f);
    }
}

uint8_t FOC_Speed_IsEnabled(void)
{
    return s_enabled;
}

void FOC_Speed_SetTargetRpm(float rpm)
{
    if (rpm > FOC_SPEED_MAX_RPM)
    {
        rpm = FOC_SPEED_MAX_RPM;
    }
    else if (rpm < -FOC_SPEED_MAX_RPM)
    {
        rpm = -FOC_SPEED_MAX_RPM;
    }
    s_target_ref_mech = rpm;
}

void FOC_Speed_IncreaseTarget(void)
{
    s_target_ref_mech += FOC_SPEED_STEP;
    if (s_target_ref_mech > FOC_SPEED_MAX_RPM)
    {
        s_target_ref_mech = FOC_SPEED_MAX_RPM;
    }
}

void FOC_Speed_DecreaseTarget(void)
{
    s_target_ref_mech -= FOC_SPEED_STEP;
    if (s_target_ref_mech < -FOC_SPEED_MAX_RPM)
    {
        s_target_ref_mech = -FOC_SPEED_MAX_RPM;
    }
}

void FOC_Speed_Run(void)
{
    uint32_t raw;
    int32_t diff;
    int32_t dir_diff;
    float inst_speed;
    float ramp_step;
    float ref_elec;
    float fbk_elec;

    if (s_enabled == 0)
    {
        return;
    }

    /* M-method speed from encoder counts at speed-loop rate */
    raw = TIM3->CNT;
    if (s_last_valid == 0)
    {
        diff = 0;
        s_last_valid = 1;
    }
    else
    {
        diff = (int32_t)(raw - (uint32_t)s_last_count);
        if (diff > (ENCODER_CPR / 2))
        {
            diff -= ENCODER_CPR;
        }
        else if (diff < -(ENCODER_CPR / 2))
        {
            diff += ENCODER_CPR;
        }
    }
    s_last_count = (int32_t)raw;

    dir_diff = diff * ENCODER_DIRECTION;
    inst_speed = ((float)dir_diff * 60.0f) / ((float)ENCODER_CPR * FOC_SPEED_TS);
    s_speed_filtered = FOC_SPEED_LPF_B * s_speed_filtered + FOC_SPEED_LPF_A * inst_speed;
    s_speed_fb_mech = s_speed_filtered;

    /* Simple speed reference ramp */
    ramp_step = FOC_SPEED_RAMP_RPM_PER_S * FOC_SPEED_TS;
    if (s_ramp_ref_mech < (s_target_ref_mech - ramp_step))
    {
        s_ramp_ref_mech += ramp_step;
    }
    else if (s_ramp_ref_mech > (s_target_ref_mech + ramp_step))
    {
        s_ramp_ref_mech -= ramp_step;
    }
    else
    {
        s_ramp_ref_mech = s_target_ref_mech;
    }

    ref_elec = s_ramp_ref_mech * (float)MOTOR_POLE_PAIRS;
    fbk_elec = s_speed_fb_mech * (float)MOTOR_POLE_PAIRS;

    s_pi_speed.Ref = ref_elec;
    s_pi_speed.Fbk = fbk_elec;
    PI_Update(&s_pi_speed);

    FOC_Current_SetIqRef(s_pi_speed.Out);
}

float FOC_Speed_GetRefRpm(void)
{
    return s_ramp_ref_mech;
}

float FOC_Speed_GetFbRpm(void)
{
    return s_speed_fb_mech;
}
