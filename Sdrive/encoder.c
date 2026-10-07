#include "encoder.h"
#include "tim.h"

#define ENCODER_TWO_PI      6.28318530718f
#define ENCODER_SPEED_LPF_A 0.1f
#define ENCODER_SPEED_LPF_B 0.9f

static float s_speed_filtered = 0.0f;

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    TIM3->CNT = 0;
    s_speed_filtered = 0.0f;
}

/* Call at a fixed rate, for example every 1 ms. */
void Encoder_Update(Encoder_t *enc)
{
    uint32_t raw = TIM3->CNT;
    int32_t diff = (int32_t)(raw - (uint32_t)enc->last_count);
    float mech;
    float inst_speed;

    if (diff > (ENCODER_CPR / 2))
    {
        diff -= ENCODER_CPR;
    }
    else if (diff < -(ENCODER_CPR / 2))
    {
        diff += ENCODER_CPR;
    }

    enc->raw_count = raw;
    enc->delta_count = diff;
    enc->last_count = (int32_t)raw;

    mech = ((float)raw * ENCODER_TWO_PI) / (float)ENCODER_CPR;
    if (mech >= ENCODER_TWO_PI)
    {
        mech -= ENCODER_TWO_PI;
    }
    enc->mech_angle_rad = mech;

    enc->elec_angle_rad = mech * (float)MOTOR_POLE_PAIRS;
    while (enc->elec_angle_rad >= ENCODER_TWO_PI)
    {
        enc->elec_angle_rad -= ENCODER_TWO_PI;
    }

    /* M-method speed: rpm = delta_count / CPR * 60 / Ts. Ts = 0.001 s. */
    inst_speed = ((float)diff * 60000.0f) / (float)ENCODER_CPR;
    s_speed_filtered = ENCODER_SPEED_LPF_B * s_speed_filtered + ENCODER_SPEED_LPF_A * inst_speed;
    enc->speed_rpm = s_speed_filtered;
}

/* Call after the rotor has been aligned to electrical angle zero. */
void Encoder_SetZero(Encoder_t *enc)
{
    TIM3->CNT = 0;
    enc->raw_count = 0;
    enc->last_count = 0;
    enc->delta_count = 0;
    enc->mech_angle_rad = 0.0f;
    enc->elec_angle_rad = 0.0f;
    enc->speed_rpm = 0.0f;
    s_speed_filtered = 0.0f;
}
