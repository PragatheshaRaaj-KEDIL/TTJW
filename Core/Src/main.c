/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : TTJW application entry point and STM32 board initialization
  *
  * Application modules are separated into dedicated .c/.h pairs:
  *   - ADS1220 driver
  *   - PT100 conversion
  *   - Heater/PID control
  *   - Matrix keypad
  *   - SH1106 OLED
  *   - Shared application state and configuration
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "ttjw_config.h"
#include "ttjw_state.h"
#include "ads1220.h"
#include "pt100.h"
#include "heater_control.h"
#include "keypad.h"
#include "oled_sh1106.h"
#include <stdint.h>
#include <limits.h>

/* HAL peripheral handles are owned by this board/application layer. */
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;

/* CubeMX-generated board setup functions remain in main.c. */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);

int main(void)
{
    int32_t raw;
    float resistance;
    float temperature;


    /* HAL initialization */
    HAL_Init();


    /* System clock */
    SystemClock_Config();


    /* GPIO */
    MX_GPIO_Init();


    /* I2C */
    MX_I2C1_Init();


    /* SPI */
    MX_SPI1_Init();


    /* TIM1 / heater PWM */
    MX_TIM1_Init();
    Heater_Init();


    /* --------------------------------------------------------------------- */
    /* OLED                                                                  */
    /* --------------------------------------------------------------------- */

    OLED_Init();


    /* V1: no startup screen. Show the live main display immediately. */
    /* --------------------------------------------------------------------- */
    /* ADS1220                                                               */
    /* --------------------------------------------------------------------- */

    ADS1220_Reset();


    /*
     * Datasheet requires a delay after RESET before another command.
     */
    HAL_Delay(5);


    /*
     * Configure:
     *
     * REG0 = 66
     * REG1 = 04
     * REG2 = 55
     * REG3 = 70
     */
    ADS1220_WriteRegisters();


    HAL_Delay(5);


    /*
     * Start continuous conversion.
     */
    ADS1220_StartConversion();


    HAL_Delay(100);


    /*
     * Matrix keypad starts with both columns HIGH.
     */
    Keypad_Init();


    /* --------------------------------------------------------------------- */
    /* MAIN LOOP                                                             */
    /* --------------------------------------------------------------------- */

    while (1)
    {
        /*
         * Process any keypad EXTI event from the main loop.
         * The EXTI callback itself remains very short.
         */
        Keypad_Process();

        /*
         * Immediately redraw setting/navigation changes using the most recent
         * PV. Previously the display was refreshed only after ADS1220 returned
         * a conversion, making held-key changes appear delayed.
         */
        if (g_displayNeedsRefresh != 0U)
        {
            OLED_DisplayTemperature(g_latestTemperature);
            g_displayNeedsRefresh = 0U;
        }

        raw = ADS1220_ReadConversion();


        /*
         * INT32_MIN means DRDY timeout.
         *
         * Safety: if the ADC does not provide a valid conversion,
         * turn the heater OFF.
         */
        if (raw != INT32_MIN)
        {
            /*
             * KEEP THE EXISTING RESISTANCE CALCULATION UNCHANGED.
             */
            resistance = CalculateResistance(raw);


            /*
             * Convert PT100 resistance to temperature.
             */
            temperature = ResistanceToTemperature(resistance);


            /*
             * Apply the signed calibration offset exactly once.
             * Positive ERROR raises PV; negative ERROR lowers PV.
             */
            g_latestTemperature = temperature + g_errorAdjustment;

            /*
             * Alarm 1 backend:
             * AL-1 = 0.0 disables Alarm 1.
             * For AL-1 > 0.0, activate when PV reaches/exceeds AL-1.
             * The indicator is steady (no blinking) and clears below threshold.
             */
            g_alarmActive = ((g_alarm1Value > 0.0f) &&
                             (g_latestTemperature >= g_alarm1Value)) ? 1U : 0U;

            /*
             * Heater control. Keep the existing zero-SV and at/above-SV
             * shutoff behavior. Below SV, calculate PID demand. The output
             * is clamped and converted by the existing PWM function.
             */
            if (g_setValue <= 0.0f)
            {
                PID_Reset();
                Heater_SetDuty(0U);
            }
            else if (g_latestTemperature >= g_setValue)
            {
                PID_Reset();
                Heater_SetDuty(0U);
            }
            else
            {
                Heater_SetDuty(Heater_CalculateDuty(g_latestTemperature));
            }

            OLED_DisplayTemperature(g_latestTemperature);
            g_displayNeedsRefresh = 0U;
        }
        else
        {
            /* ADC timeout = heater OFF and clear accumulated PID state. */
            PID_Reset();
            Heater_SetDuty(0U);
            g_alarmActive = 0U; /* No valid fresh PV to evaluate. */
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    /*
     * Configure regulator.
     */
    if (HAL_PWREx_ControlVoltageScaling(
            PWR_REGULATOR_VOLTAGE_SCALE1)
        != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * HSI oscillator.
     */
    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;


    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;


    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;


    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;


    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSI;


    RCC_OscInitStruct.PLL.PLLM =
        1;


    RCC_OscInitStruct.PLL.PLLN =
        10;


    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV7;


    RCC_OscInitStruct.PLL.PLLQ =
        RCC_PLLQ_DIV2;


    RCC_OscInitStruct.PLL.PLLR =
        RCC_PLLR_DIV2;


    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct)
        != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * Clock configuration.
     */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;


    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;


    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;


    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV1;


    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;


    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_4)
        != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance =
        I2C1;


    hi2c1.Init.Timing =
        0x10D19CE4;


    hi2c1.Init.OwnAddress1 =
        0;


    hi2c1.Init.AddressingMode =
        I2C_ADDRESSINGMODE_7BIT;


    hi2c1.Init.DualAddressMode =
        I2C_DUALADDRESS_DISABLE;


    hi2c1.Init.OwnAddress2 =
        0;


    hi2c1.Init.OwnAddress2Masks =
        I2C_OA2_NOMASK;


    hi2c1.Init.GeneralCallMode =
        I2C_GENERALCALL_DISABLE;


    hi2c1.Init.NoStretchMode =
        I2C_NOSTRETCH_DISABLE;


    if (HAL_I2C_Init(&hi2c1)
        != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_I2CEx_ConfigAnalogFilter(
            &hi2c1,
            I2C_ANALOGFILTER_ENABLE)
        != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_I2CEx_ConfigDigitalFilter(
            &hi2c1,
            0)
        != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance =
        SPI1;


    hspi1.Init.Mode =
        SPI_MODE_MASTER;


    hspi1.Init.Direction =
        SPI_DIRECTION_2LINES;


    hspi1.Init.DataSize =
        SPI_DATASIZE_8BIT;


    /*
     * SPI Mode 1:
     *
     * CPOL = 0
     * CPHA = 1
     */
    hspi1.Init.CLKPolarity =
        SPI_POLARITY_LOW;


    hspi1.Init.CLKPhase =
        SPI_PHASE_2EDGE;


    hspi1.Init.NSS =
        SPI_NSS_SOFT;


    hspi1.Init.BaudRatePrescaler =
        SPI_BAUDRATEPRESCALER_64;


    hspi1.Init.FirstBit =
        SPI_FIRSTBIT_MSB;


    hspi1.Init.TIMode =
        SPI_TIMODE_DISABLE;


    hspi1.Init.CRCCalculation =
        SPI_CRCCALCULATION_DISABLE;


    hspi1.Init.CRCPolynomial =
        7;


    hspi1.Init.CRCLength =
        SPI_CRC_LENGTH_DATASIZE;


    hspi1.Init.NSSPMode =
        SPI_NSS_PULSE_DISABLE;


    if (HAL_SPI_Init(&hspi1)
        != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 7999;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 19999;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim1);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();


    /* Initial keypad columns HIGH. */
    HAL_GPIO_WritePin(
        GPIOC,
        KEY_L1_Pin | KEY_L2_Pin,
        GPIO_PIN_SET
    );


    /* ADS1220 CS HIGH. */
    HAL_GPIO_WritePin(
        ADS_CS_GPIO_Port,
        ADS_CS_Pin,
        GPIO_PIN_SET
    );


    /* B1. */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(
        B1_GPIO_Port,
        &GPIO_InitStruct
    );


    /* Keypad columns: PC0 / PC1. */
    GPIO_InitStruct.Pin =
        KEY_L1_Pin |
        KEY_L2_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct
    );


    /* Keypad rows: PC2 / PC3. */
    GPIO_InitStruct.Pin =
        KEY_R1_Pin |
        KEY_R2_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_IT_FALLING;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct
    );


    /* ADS1220 CS. */
    GPIO_InitStruct.Pin =
        ADS_CS_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        ADS_CS_GPIO_Port,
        &GPIO_InitStruct
    );


    /* Existing push button. */
    GPIO_InitStruct.Pin =
        PUSH_BUTTON_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_IT_FALLING;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    HAL_GPIO_Init(
        PUSH_BUTTON_GPIO_Port,
        &GPIO_InitStruct
    );


    /* ADS1220 DRDY. */
    GPIO_InitStruct.Pin =
        ADS_DRDY_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        ADS_DRDY_GPIO_Port,
        &GPIO_InitStruct
    );


    /* EXTI. */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Error_Handler(void)
{
    __disable_irq();


    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* Add board-specific assertion reporting here if needed. */
    (void)file;
    (void)line;
}
#endif
