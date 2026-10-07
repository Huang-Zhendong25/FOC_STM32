#include "current_sense.h"
#include "adc.h"

/* Regular group, DMA:
 * ADC_Value[0] = temperature   (ADC_CHANNEL_14)
 * ADC_Value[1] = DC bus voltage (ADC_CHANNEL_10)
 * ADC_Value[2] = VR / spare     (ADC_CHANNEL_4)
 *
 * Injected group, triggered by TIM1_CC4 at PWM center:
 * Injected rank 1 = U phase current (ADC_CHANNEL_15)
 * Injected rank 2 = W phase current (ADC_CHANNEL_9)
 */
#define CURRENT_SENSE_GAIN      0.0061f    /* CC6903 sensitivity 0.132 V/A */
#define CURRENT_SENSE_LPF_A     0.37699f
#define CURRENT_SENSE_LPF_B     0.62301f
#define CURRENT_SENSE_VBUS_GAIN 0.0088653554f

volatile uint16_t ADC_Value[3];

static float s_offset_u = 0.0f;
static float s_offset_w = 0.0f;
static float s_iu_filtered = 0.0f;
static float s_iw_filtered = 0.0f;

static void CurrentSense_UpdateRawCurrents(uint32_t adc_u, uint32_t adc_w)
{
    float iu_raw = (s_offset_u - (float)adc_u) * CURRENT_SENSE_GAIN;
    float iw_raw = (s_offset_w - (float)adc_w) * CURRENT_SENSE_GAIN;

    s_iu_filtered = CURRENT_SENSE_LPF_B * s_iu_filtered + CURRENT_SENSE_LPF_A * iu_raw;
    s_iw_filtered = CURRENT_SENSE_LPF_B * s_iw_filtered + CURRENT_SENSE_LPF_A * iw_raw;
}

void CurrentSense_StartDma(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Value, 3);
}

void CurrentSense_StartInjectedPolling(void)
{
    HAL_ADCEx_InjectedStart(&hadc1);
}

void CurrentSense_StartInjectedIT(void)
{
    HAL_ADCEx_InjectedStop(&hadc1);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

/* Must be called while TIM1 outputs zero voltage vector, so motor current is zero. */
void CurrentSense_CalibrateOffset(void)
{
    uint32_t sum_u = 0;
    uint32_t sum_w = 0;
    uint32_t count = 0;

    while (count < 1000U)
    {
        if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOC))
        {
            __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
            sum_u += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
            sum_w += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
            count++;
        }
    }

    s_offset_u = (float)sum_u / (float)count;
    s_offset_w = (float)sum_w / (float)count;
}

void CurrentSense_UpdateFromInjected(CurrentSense_t *cur)
{
    CurrentSense_UpdateRawCurrents(
        HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1),
        HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2));

    cur->Iu = s_iu_filtered;
    cur->Iw = s_iw_filtered;
    cur->Iv = -(cur->Iu + cur->Iw);
    cur->Vbus = (float)ADC_Value[1] * CURRENT_SENSE_VBUS_GAIN;
}
