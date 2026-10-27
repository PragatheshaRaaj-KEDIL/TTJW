/* ADS1220 SPI driver. Keep command/register details isolated from application logic. */
#include "main.h"
#include "ttjw_config.h"
#include "ads1220.h"
#include "keypad.h"
#include <stdint.h>
#include <limits.h>

extern SPI_HandleTypeDef hspi1;

static void ADS1220_Select(void);
static void ADS1220_Deselect(void);
static uint8_t ADS1220_SPI_Transfer(uint8_t data);

static void ADS1220_Select(void)
{
    HAL_GPIO_WritePin(
        ADS_CS_GPIO_Port,
        ADS_CS_Pin,
        GPIO_PIN_RESET
    );
}
static void ADS1220_Deselect(void)
{
    HAL_GPIO_WritePin(
        ADS_CS_GPIO_Port,
        ADS_CS_Pin,
        GPIO_PIN_SET
    );
}
static uint8_t ADS1220_SPI_Transfer(uint8_t data)
{
    uint8_t rxData = 0;


    if (HAL_SPI_TransmitReceive(
            &hspi1,
            &data,
            &rxData,
            1,
            HAL_MAX_DELAY) != HAL_OK)
    {
        return 0;
    }


    return rxData;
}
void ADS1220_Reset(void)
{
    ADS1220_Select();


    ADS1220_SPI_Transfer(
        ADS1220_CMD_RESET
    );


    ADS1220_Deselect();


    /*
     * Give ADS1220 time to complete reset.
     */
    HAL_Delay(5);
}
void ADS1220_WriteRegisters(void)
{
    /*
     * IMPORTANT:
     *
     * Correct WREG command for registers 0-3 is:
     *
     * 0x43
     *
     * Complete SPI sequence:
     *
     * 43 66 04 55 70
     */


    ADS1220_Select();


    /* WREG command */
    ADS1220_SPI_Transfer(
        ADS1220_CMD_WREG_4REG
    );


    /* Register 0 */
    ADS1220_SPI_Transfer(
        ADS1220_REG0
    );


    /* Register 1 */
    ADS1220_SPI_Transfer(
        ADS1220_REG1
    );


    /* Register 2 */
    ADS1220_SPI_Transfer(
        ADS1220_REG2
    );


    /* Register 3 */
    ADS1220_SPI_Transfer(
        ADS1220_REG3
    );


    ADS1220_Deselect();


    /*
     * Configuration registers are updated on the final
     * SCLK falling edge.
     */
    HAL_Delay(2);
}
void ADS1220_StartConversion(void)
{
    ADS1220_Select();


    ADS1220_SPI_Transfer(
        ADS1220_CMD_START
    );


    ADS1220_Deselect();
}
int32_t ADS1220_ReadConversion(void)
{
    uint32_t startTime;

    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;

    int32_t raw;


    /*
     * Wait for DRDY to go LOW.
     *
     * DRDY LOW = conversion result available.
     */
    startTime = HAL_GetTick();


    while (HAL_GPIO_ReadPin(
               ADS_DRDY_GPIO_Port,
               ADS_DRDY_Pin) == GPIO_PIN_SET)
    {
        /* Keep keypad scanning alive during the ~20 SPS conversion wait. */
        Keypad_Process();

        if ((HAL_GetTick() - startTime) >
            ADS_DRDY_TIMEOUT_MS)
        {
            /*
             * No valid conversion.
             */
            return INT32_MIN;
        }
    }


    /*
     * DRDY is LOW.
     *
     * Read 24-bit conversion result.
     */
    ADS1220_Select();


    /* Read MSB */
    byte1 = ADS1220_SPI_Transfer(0x00);


    /* Read middle byte */
    byte2 = ADS1220_SPI_Transfer(0x00);


    /* Read LSB */
    byte3 = ADS1220_SPI_Transfer(0x00);


    ADS1220_Deselect();


    /*
     * Build 24-bit signed result.
     */
    raw =
        ((int32_t)byte1 << 16) |
        ((int32_t)byte2 << 8) |
        ((int32_t)byte3);


    /*
     * Sign extension.
     */
    if (raw & 0x00800000)
    {
        raw |= 0xFF000000;
    }


    return raw;
}
