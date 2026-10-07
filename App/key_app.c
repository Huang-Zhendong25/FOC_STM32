#include "key_app.h"

#define KEY_APP_GPIO_PORT        GPIOB
#define KEY_APP_DEBOUNCE_MS      20
#define KEY_APP_DOUBLE_INTERVAL  300

static const uint16_t s_key_pins[3] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14};

static uint8_t  s_raw_last[3]       = {1U, 1U, 1U};
static uint8_t  s_stable[3]         = {1U, 1U, 1U};
static uint8_t  s_last_stable[3]    = {1U, 1U, 1U};
static uint32_t s_change_tick[3]    = {0U, 0U, 0U};
static uint8_t  s_single_event[3]   = {0U, 0U, 0U};
static uint8_t  s_double_event[3]   = {0U, 0U, 0U};
static uint8_t  s_click_count[3]    = {0U, 0U, 0U};
static uint32_t s_first_release[3]  = {0U, 0U, 0U};

void KEY_AppInit(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
    HAL_GPIO_Init(KEY_APP_GPIO_PORT, &gpio);
}

void KEY_AppScan(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t i;

    for (i = 0; i < 3; i++)
    {
        uint8_t raw = (HAL_GPIO_ReadPin(KEY_APP_GPIO_PORT, s_key_pins[i]) == GPIO_PIN_RESET) ? 0U : 1U;

        if (raw != s_raw_last[i])
        {
            s_raw_last[i] = raw;
            s_change_tick[i] = now;
        }

        if ((now - s_change_tick[i]) >= KEY_APP_DEBOUNCE_MS)
        {
            s_stable[i] = raw;
        }

        s_single_event[i] = 0U;
        s_double_event[i] = 0U;

        /* stable press edge */
        if ((s_stable[i] == 0U) && (s_last_stable[i] == 1U))
        {
            if ((s_click_count[i] == 1U) &&
                (s_first_release[i] != 0U) &&
                ((now - s_first_release[i]) <= KEY_APP_DOUBLE_INTERVAL))
            {
                s_double_event[i] = 1U;
                s_click_count[i] = 0U;
                s_first_release[i] = 0U;
            }
            else
            {
                s_click_count[i] = 1U;
                s_first_release[i] = 0U;
            }
        }

        /* stable release edge */
        if ((s_stable[i] == 1U) && (s_last_stable[i] == 0U))
        {
            if (s_click_count[i] == 1U)
            {
                s_first_release[i] = now;
            }
        }

        /* single click confirmed after double-click window expires */
        if ((s_click_count[i] == 1U) &&
            (s_stable[i] == 1U) &&
            (s_first_release[i] != 0U) &&
            ((now - s_first_release[i]) > KEY_APP_DOUBLE_INTERVAL))
        {
            s_single_event[i] = 1U;
            s_click_count[i] = 0U;
            s_first_release[i] = 0U;
        }

        s_last_stable[i] = s_stable[i];
    }
}

uint8_t KEY_AppGetSingleClick(uint8_t keyIndex)
{
    if (keyIndex >= 3U)
    {
        return 0U;
    }
    return s_single_event[keyIndex];
}

uint8_t KEY_AppGetDoubleClick(uint8_t keyIndex)
{
    if (keyIndex >= 3U)
    {
        return 0U;
    }
    return s_double_event[keyIndex];
}
