/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ================= 阶段1：SVPWM 电压开环测试 ================= */
#define SVPWM_KM_BACKW     0.1443376f          /* 1 / (12V * sqrt(3)/3)，例程的标幺反算系数 */
#define TIM1_PERIOD        8400                /* TIM1 ARR 周期计数值 */
#define TIM1_PERIOD_HALF  (TIM1_PERIOD / 2)    /* 50% 中点，对应零电压矢量 */

/* 测试模式：1 = 锁轴测试；0 = 开环旋转测试 */
#define STAGE1_LOCK_TEST   0
#define OPENLOOP_U_AMP     1.0f                /* Alpha/Beta 电压幅值，单位 V，满幅约 6.93V */
#define OPENLOOP_ELEC_HZ   1.0f                /* 开环旋转电频率，Hz */

/* 按键定义：KEY1/2/3 在 GPIOB，低电平有效 */
#define KEY_GPIO_PORT       GPIOB
#define KEY1_PIN            GPIO_PIN_12
#define KEY2_PIN            GPIO_PIN_13
#define KEY3_PIN            GPIO_PIN_14
#define KEY_DEBOUNCE_MS     20
/* ============================================================== */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static float g_theta_deg = 0.0f;   /* 开环旋转当前电角度，单位度 */
static uint8_t g_motor_enabled = 0;    /* 0 = PWM 未使能，1 = 已使能 */
static uint8_t g_lock_axis = 0;        /* 0 = α 轴锁轴，1 = β 轴锁轴 */

static uint8_t  g_key_raw_last[3]  = {1U, 1U, 1U};
static uint8_t  g_key_stable[3]    = {1U, 1U, 1U};
static uint8_t  g_key_last_stable[3] = {1U, 1U, 1U};
static uint32_t g_key_change_tick[3] = {0U, 0U, 0U};
static uint8_t  g_key_event[3] = {0U, 0U, 0U};
static const uint16_t g_key_pins[3] = {KEY1_PIN, KEY2_PIN, KEY3_PIN};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Motor_Pwm_Start(void);
static void Motor_Pwm_Stop(void);
static void SVPWM_Update(float ualpha, float ubeta);
static void Key_Init(void);
static void Key_Scan(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 使能 TIM1 三相上/下桥 PWM 输出（互补 + 主输出使能） */
static void Motor_Pwm_Start(void)
{
    TIM1->CCER |= 0x5555;                          /* 使能 CH1/CH2/CH3 的 CCxE 与 CCxNE */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/* 关闭三相 PWM 输出 */
static void Motor_Pwm_Stop(void)
{
    TIM1->CCER &= 0xAAAA;                          /* 关闭 CH1/CH2/CH3 的 CCxE 与 CCxNE */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

/* Alpha/Beta 电压 -> 七段式 SVPWM -> TIM1 比较寄存器 */
static void SVPWM_Update(float ualpha, float ubeta)
{
    float u1, u2, u3;
    float Ta, Tb, Tc;
    uint8_t sector = 3;

    /* 反 Clarke：Alpha/Beta 电压到三相反电动势等效值 */
    u1 = ubeta;
    u2 = ubeta * 0.5f + ualpha * 0.8660254f;
    u3 = u2 - u1;

    /* 判断扇区 */
    sector = (u2 > 0) ? (sector - 1) : sector;
    sector = (u3 > 0) ? (sector - 1) : sector;
    sector = (u1 < 0) ? (7 - sector) : sector;

    /* 根据扇区计算三相作用时间 Ta/Tb/Tc（-1 ~ +1 标幺） */
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

    /* 标幺值 -> 以 50% 占空比为中心的 CCR 值 */
    TIM1->CCR1 = (uint16_t)(Ta * SVPWM_KM_BACKW * TIM1_PERIOD_HALF + TIM1_PERIOD_HALF);
    TIM1->CCR2 = (uint16_t)(Tb * SVPWM_KM_BACKW * TIM1_PERIOD_HALF + TIM1_PERIOD_HALF);
    TIM1->CCR3 = (uint16_t)(Tc * SVPWM_KM_BACKW * TIM1_PERIOD_HALF + TIM1_PERIOD_HALF);
}

/* 初始化 KEY1/2/3 为上拉输入 */
static void Key_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = KEY1_PIN | KEY2_PIN | KEY3_PIN;
    HAL_GPIO_Init(KEY_GPIO_PORT, &gpio);
}

/* 扫描 KEY1/2/3，在按键稳定按下时产生一次事件 */
static void Key_Scan(void)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < 3; i++)
    {
        uint8_t raw = (HAL_GPIO_ReadPin(KEY_GPIO_PORT, g_key_pins[i]) == GPIO_PIN_RESET) ? 0U : 1U;

        if (raw != g_key_raw_last[i])
        {
            g_key_raw_last[i] = raw;
            g_key_change_tick[i] = now;
        }

        if ((now - g_key_change_tick[i]) >= KEY_DEBOUNCE_MS)
        {
            g_key_stable[i] = raw;
        }

        g_key_event[i] = 0U;
        if ((g_key_stable[i] == 0U) && (g_key_last_stable[i] == 1U))
        {
            g_key_event[i] = 1U;   /* 检测到一次按下事件 */
        }
        g_key_last_stable[i] = g_key_stable[i];
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);

  /* 阶段1：先把三相 CCR 放到 50% 中点（零电压矢量），避免启动瞬间误输出 */
  TIM1->CCR1 = TIM1_PERIOD_HALF;
  TIM1->CCR2 = TIM1_PERIOD_HALF;
  TIM1->CCR3 = TIM1_PERIOD_HALF;

  /* 确认板级电源/驱动使能脚状态 */
  HAL_GPIO_WritePin(PWR_GPIO_Port, PWR_Pin, GPIO_PIN_SET);   /* 电源使能 */
  HAL_GPIO_WritePin(SD1_GPIO_Port, SD1_Pin, GPIO_PIN_RESET); /* SD1 关断脚使能 */

  /* 初始化按键，初始不使能 PWM，等待 KEY1 启动 */
  Key_Init();
  g_motor_enabled = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Key_Scan();

    /* KEY1：使能电机 */
    if ((g_key_event[0] == 1U) && (g_motor_enabled == 0U))
    {
        Motor_Pwm_Start();
        g_motor_enabled = 1U;
    }

    /* KEY2：失能电机 */
    if ((g_key_event[1] == 1U) && (g_motor_enabled == 1U))
    {
        Motor_Pwm_Stop();
        g_motor_enabled = 0U;
        g_theta_deg = 0.0f;
        SVPWM_Update(0.0f, 0.0f);   /* 回到零矢量 */
    }

    /* KEY3：切换锁轴方向（α 轴 / β 轴） */
    if (g_key_event[2] == 1U)
    {
        g_lock_axis = (g_lock_axis == 0U) ? 1U : 0U;
    }

    if (g_motor_enabled == 0U)
    {
        /* 未使能时保持零矢量 */
        SVPWM_Update(0.0f, 0.0f);
    }
    else
    {
#if STAGE1_LOCK_TEST
        /* 锁轴测试：可按键在 α 轴与 β 轴之间切换 */
        if (g_lock_axis == 0U)
        {
            SVPWM_Update(OPENLOOP_U_AMP, 0.0f);   /* α 轴锁轴 */
        }
        else
        {
            SVPWM_Update(0.0f, OPENLOOP_U_AMP);   /* β 轴锁轴 */
        }
#else
        /* 开环旋转测试：电压矢量按固定电频率缓慢旋转 */
        {
            float rad;
            g_theta_deg += OPENLOOP_ELEC_HZ * 360.0f * 0.001f;
            if (g_theta_deg >= 360.0f)
            {
                g_theta_deg -= 360.0f;
            }
            rad = g_theta_deg * 3.14159265f / 180.0f;
            SVPWM_Update(OPENLOOP_U_AMP * cosf(rad), OPENLOOP_U_AMP * sinf(rad));
        }
#endif
    }

    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
