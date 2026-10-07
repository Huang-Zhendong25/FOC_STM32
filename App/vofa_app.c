#include "vofa_app.h"
#include "usart.h"

#define VOFA_CHANNEL_COUNT   16
#define VOFA_FRAME_SIZE      (VOFA_CHANNEL_COUNT * 4U + 4U)

static uint8_t  s_tx_buf[VOFA_FRAME_SIZE];
static volatile uint8_t s_tx_busy = 0;
static uint32_t s_last_send_tick = 0;

static void VOFA_FloatToBytes(float f, uint8_t *dst)
{
    union
    {
        float    f;
        uint32_t u;
    } v;

    v.f = f;
    dst[0] = (uint8_t)(v.u & 0xFFU);
    dst[1] = (uint8_t)((v.u >> 8U) & 0xFFU);
    dst[2] = (uint8_t)((v.u >> 16U) & 0xFFU);
    dst[3] = (uint8_t)((v.u >> 24U) & 0xFFU);
}

void VOFA_Init(void)
{
    s_tx_busy = 0;
    s_last_send_tick = HAL_GetTick();
}

void VOFA_Task(CurrentSense_t *cur, Encoder_t *enc)
{
    uint32_t now = HAL_GetTick();

    if ((now - s_last_send_tick) < VOFA_SEND_PERIOD_MS)
    {
        return;
    }
    s_last_send_tick = now;

    if (s_tx_busy != 0U)
    {
        return;
    }

    VOFA_FloatToBytes(cur->Iu, &s_tx_buf[0]);
    VOFA_FloatToBytes(cur->Iv, &s_tx_buf[4]);
    VOFA_FloatToBytes(cur->Iw, &s_tx_buf[8]);
    VOFA_FloatToBytes(cur->Ialpha, &s_tx_buf[12]);
    VOFA_FloatToBytes(cur->Ibeta, &s_tx_buf[16]);
    VOFA_FloatToBytes(cur->Vbus, &s_tx_buf[20]);
    VOFA_FloatToBytes(enc->mech_angle_rad, &s_tx_buf[24]);
    VOFA_FloatToBytes(enc->elec_angle_rad, &s_tx_buf[28]);
    VOFA_FloatToBytes(enc->speed_rpm, &s_tx_buf[32]);
    VOFA_FloatToBytes(FOC_Current_GetId(), &s_tx_buf[36]);
    VOFA_FloatToBytes(FOC_Current_GetIq(), &s_tx_buf[40]);
    VOFA_FloatToBytes(FOC_Current_GetVd(), &s_tx_buf[44]);
    VOFA_FloatToBytes(FOC_Current_GetVq(), &s_tx_buf[48]);
    VOFA_FloatToBytes(FOC_Current_GetIqRef(), &s_tx_buf[52]);
    VOFA_FloatToBytes(FOC_Speed_GetRefRpm(), &s_tx_buf[56]);
    VOFA_FloatToBytes(FOC_Speed_GetFbRpm(), &s_tx_buf[60]);

    /* JustFloat tail: 00 00 80 7f */
    s_tx_buf[64] = 0x00U;
    s_tx_buf[65] = 0x00U;
    s_tx_buf[66] = 0x80U;
    s_tx_buf[67] = 0x7fU;

    s_tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&huart1, s_tx_buf, VOFA_FRAME_SIZE) != HAL_OK)
    {
        s_tx_busy = 0U;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        s_tx_busy = 0U;
    }
}
