#include "foc_position.h"
#include "foc_speed.h"
#include "encoder.h"
#include "tim.h"

static uint8_t s_enabled = 0;
static int32_t s_target = 0;
static int32_t s_fbk = 0;
static float s_error = 0.0f;
static float s_speed_ref = 0.0f;

void FOC_Position_Init(void)
{
    s_enabled = 0;
    s_target = (int32_t)TIM3->CNT;
    s_fbk = s_target;
    s_error = 0.0f;
    s_speed_ref = 0.0f;
}

void FOC_Position_Enable(void)
{
    if (s_enabled == 0)
    {
        s_target = (int32_t)TIM3->CNT;
        s_fbk = s_target;
        s_error = 0.0f;
        s_speed_ref = 0.0f;
        FOC_Speed_SetTargetRpm(0.0f);
        FOC_Speed_Enable();
        s_enabled = 1;
    }
}

void FOC_Position_Disable(void)
{
    if (s_enabled == 1)
    {
        FOC_Speed_Disable();
        s_enabled = 0;
        s_target = (int32_t)TIM3->CNT;
        s_fbk = s_target;
        s_error = 0.0f;
        s_speed_ref = 0.0f;
        FOC_Speed_SetTargetRpm(0.0f);
    }
}

uint8_t FOC_Position_IsEnabled(void)
{
    return s_enabled;
}

void FOC_Position_IncreaseTarget(void)
{
    s_target += FOC_POSITION_STEP;
    if (s_target > FOC_POSITION_MAX)
    {
        s_target = FOC_POSITION_MAX;
    }
}

void FOC_Position_DecreaseTarget(void)
{
    s_target -= FOC_POSITION_STEP;
    if (s_target < 0)
    {
        s_target = 0;
    }
}

void FOC_Position_Run(void)
{
    int32_t err;

    if (s_enabled == 0)
    {
        return;
    }

    s_fbk = (int32_t)TIM3->CNT;
    err = s_target - s_fbk;

    /* shortest path across encoder rollover */
    if (err > (ENCODER_CPR / 2))
    {
        err -= ENCODER_CPR;
    }
    else if (err < -(ENCODER_CPR / 2))
    {
        err += ENCODER_CPR;
    }

    s_error = (float)err;

    s_speed_ref = FOC_POSITION_KP * s_error;
    if (s_speed_ref > FOC_POSITION_OUT_MAX)
    {
        s_speed_ref = FOC_POSITION_OUT_MAX;
    }
    else if (s_speed_ref < -FOC_POSITION_OUT_MAX)
    {
        s_speed_ref = -FOC_POSITION_OUT_MAX;
    }

    FOC_Speed_SetTargetRpm(s_speed_ref);
}

int32_t FOC_Position_GetTarget(void) { return s_target; }
int32_t FOC_Position_GetFbk(void)   { return s_fbk; }
float FOC_Position_GetError(void)   { return s_error; }
