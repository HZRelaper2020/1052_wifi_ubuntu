/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */
#pragma once

#include "platform_sleep.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/* UART port used for standard I/O */
#ifdef PLATFORM_WL_UART_ENABLED
#define STDIO_UART  ( WICED_UART_3 )
#else
#define STDIO_UART  ( WICED_UART_1 )
#endif /* PLATFORM_WL_UART_ENABLED */

/******************************************************
 *                   Enumerations
 ******************************************************/


/*
BCM943909WCD1_3 platform pin definitions ...
+-------------------------------------------------------------------------------------------------------------------+
| Enum ID       |Pin  |   Pin Name on    |  Module                           |  Conn  |  Peripheral  |  Peripheral  |
|               | #   |      43909       |  Alias                            |  Pin   |  Available   |    Alias     |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_1  | J16 | GPIO_0           | GPIO_0_USB_CLT                    |  U8:19 |  GPIO        |              |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_2  | D18 | GPIO_1           | GPIO_1_GSPI_MODE_EXT_PLL_F0       |  U8:21 |  GPIO        |              |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_3  | J3  | GPIO_7           | GPIO_7_WCPU_BOOT_MODE             | U10:41 |  GPIO        |              |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_4  | E15 | GPIO_8           | GPIO_8_TAP_SEL                    | U10:43 |  GPIO        |              |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_5  | J2  | GPIO_9           | GPIO_9_USB_HSIC_SEL_ETH_PHY_RESET | U10:45 |  GPIO        |              |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_6  | E18 | GPIO_10          | GPIO_10_DEEP_RST                  | U10:49 |  GPIO        |              |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_7  | E17 | GPIO_11          | GPIO_11_ACPU_BOOT_MODE            | U10:53 |  GPIO        |              |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_8  | F16 | GPIO_12          | GPIO_12_CODEC_PDN                 | U10:55 |  GPIO        |              |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_9  | C16 | GPIO_13          | GPIO_13_SDIO_MODE_EXT_PLL_F0      | U10:57 |  GPIO        |              |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_10 | A15 | GPIO_14          | GPIO_14_BT_HOST_WAKE              | U10:59 |  GPIO        |              | *
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_11 | B16 | GPIO_15          | GPIO_15_BT_RESET                  | U10:63 |  GPIO        |              |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_12 | D16 | GPIO_16          | GPIO_16_USB_EN_AUD_SW             | U10:65 |  GPIO        |              | *
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_13 | G22 | PWM_0            | PWM_0                             |  U8:60 |  PWM, GPIO   | GPIO_18      |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_14 | F22 | PWM_1            | PWM_1                             |  U8:62 |  PWM, GPIO   | GPIO_19      |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_15 | C24 | PWM_2            | PWM_2                             |  U8:64 |  PWM, GPIO   | GPIO_20      |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_16 | C23 | PWM_3            | PWM_3                             |  U8:66 |  PWM, GPIO   | GPIO_21      |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_17 | K19 | PWM_4            | PWM_4                             |  U8:68 |  PWM, GPIO   | GPIO_22      |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_18 | E24 | PWM_5            | PWM_5                             |  U8:70 |  PWM, GPIO   | GPIO_23      |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_19 | N8  | SPI_0_MISO       | SPI_0_MISO                        |  U8:36 |  SPI, GPIO   | GPIO_24      |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_20 | M3  | SPI_0_CLK        | SPI_0_CLK                         |  U8:34 |  SPI, GPIO   | GPIO_25      |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_21 | M7  | SPI_0_MOSI       | SPI_0_MOSI                        |  U8:38 |  SPI, GPIO   | GPIO_26      |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_22 | B15 | SPI_0_CS         | SPI_0_CS                          |  U8:40 |  SPI, GPIO   | GPIO_27      |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_23 | G15 | GPIO_2           | GPIO_2_JTAG_TCK                   | U10:19 |  GPIO        |              |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_24 | E16 | GPIO_3           | GPIO_3_JTAG_TMS                   | U10:33 |  GPIO        |              |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_25 | H15 | GPIO_4           | GPIO_4_JTAG_TDI                   | U10:21 |  GPIO        |              |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_26 | F14 | GPIO_5           | GPIO_5_JTAG_TDO                   | U10:23 |  GPIO        |              |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_27 | D15 | GPIO_6           | GPIO_6_JTAG_TRST_L                | U10:35 |  GPIO        |              |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_28 | H1  | I2C0_SDATA       | I2C_0_SDATA                       | U10:68 |  I2C, GPIO   | GPIO_28      |
|---------------+-----+------------------+-----------------------------------+--------+--------------+--------------|
| WICED_GPIO_29 | A14 | I2C0_CLK         | I2C_0_CLK                         | U10:70 |  I2C, GPIO   | GPIO_29      |
+-------------------------------------------------------------------------------------------------------------------+

+--------------------------------------------------------------------------------------------------------------------------------+
| Enum ID                 |Pin  |   Pin Name on |    Board                                 |  Conn  |  Peripheral |  Peripheral  |
|                         | #   |      43909    |  Connection                              |  Pin   |  Available  |    Alias     |
|-------------------------+-----+---------------+------------------------------------------+--------+-------------+--------------|
| WICED_PERIPHERAL_PIN_1  | G24 | RF_SW_CTRL_6  | RF_SW_CTRL_6_UART1_RX_IN                 |  U8:3  |  UART1      |              |
|-------------------------+-----+---------------+------------------------------------------+--------+-------------+--------------|
| WICED_PERIPHERAL_PIN_2  | H23 | RF_SW_CTRL_7  | RF_SW_CTRL_7_RSRC_INIT_MODE_UART1_TX_OUT |  U8:5  |  UART1      |              |
|-------------------------+-----+---------------+------------------------------------------+--------+-------------+--------------|
| WICED_PERIPHERAL_PIN_3  | L4  | UART0_RXD     | BT_UART_TXD                              |  -     |  UART2      |              |
|-------------------------+-----+---------------+------------------------------------------+--------+-------------+--------------|
| WICED_PERIPHERAL_PIN_4  | J5  | UART0_TXD     | BT_UART_RXD                              |  -     |  UART2      |              |
|-------------------------+-----+---------------+------------------------------------------+--------+-------------+--------------|
| WICED_PERIPHERAL_PIN_5  | K1  | UART0_CTS     | BT_UART_RTS_L                            |  -     |  UART2      |              |
|-------------------------+-----+---------------+------------------------------------------+--------+-------------+--------------|
| WICED_PERIPHERAL_PIN_6  | L6  | UART0_RTS     | BT_UART_CTS_L                            |  -     |  UART2      |              |
|-------------------------+-----+---------------+------------------------------------------+--------+-------------+--------------|
| WICED_PERIPHERAL_PIN_7  | J24 | RF_SW_CTRL_8  | BT_GPIO_7_SECI_IN                        |  -     |  UART3      |              |
|-------------------------+-----+---------------+------------------------------------------+--------+-------------+--------------|
| WICED_PERIPHERAL_PIN_8  | K20 | RF_SW_CTRL_9  | BT_GPI0_6_SECI_OUT                       |  -     |  UART3      |              |
|-------------------------+-----+---------------+------------------------------------------+--------+-------------+--------------+
*/

typedef enum
{
    WICED_GPIO_1,
    WICED_GPIO_2,
    WICED_GPIO_3,
    WICED_GPIO_4,
    WICED_GPIO_5,
    WICED_GPIO_6,
    WICED_GPIO_7,
    WICED_GPIO_8,
    WICED_GPIO_9,
    WICED_GPIO_10,
    WICED_GPIO_11,
    WICED_GPIO_12,
    WICED_GPIO_13,
    WICED_GPIO_14,
    WICED_GPIO_15,
    WICED_GPIO_16,
    WICED_GPIO_17,
    WICED_GPIO_18,
    WICED_GPIO_19,
    WICED_GPIO_20,
    WICED_GPIO_21,
    WICED_GPIO_22,
    WICED_GPIO_23,
    WICED_GPIO_24,
    WICED_GPIO_25,
    WICED_GPIO_26,
    WICED_GPIO_27,
    WICED_GPIO_28,
    WICED_GPIO_29,
    WICED_GPIO_MAX, /* Denotes the total number of GPIO port aliases. Not a valid GPIO alias */
} wiced_gpio_t;

typedef enum
{
    WICED_PERIPHERAL_PIN_1 = WICED_GPIO_MAX,
    WICED_PERIPHERAL_PIN_2,
    WICED_PERIPHERAL_PIN_3,
    WICED_PERIPHERAL_PIN_4,
    WICED_PERIPHERAL_PIN_5,
    WICED_PERIPHERAL_PIN_6,
    WICED_PERIPHERAL_PIN_7,
    WICED_PERIPHERAL_PIN_8,
    WICED_PERIPHERAL_PIN_MAX, /* Denotes the total number of GPIO and peripheral pins. Not a valid pin alias */
} wiced_peripheral_pin_t;

typedef enum
{
    WICED_SPI_1,
    WICED_SPI_2,
    WICED_SPI_MAX, /* Denotes the total number of SPI port aliases. Not a valid SPI alias */
} wiced_spi_t;

typedef enum
{
    WICED_I2C_1,
    WICED_I2C_2,
    WICED_I2C_MAX, /* Denotes the total number of I2C port aliases. Not a valid I2C alias */
} wiced_i2c_t;

typedef enum
{
    WICED_PWM_1,
    WICED_PWM_2,
    WICED_PWM_3,
    WICED_PWM_4,
    WICED_PWM_5,
    WICED_PWM_6,
    WICED_PWM_MAX, /* Denotes the total number of PWM port aliases. Not a valid PWM alias */
} wiced_pwm_t;

typedef enum
{
   WICED_ADC_NONE = 0xff, /* No ADCs */
} wiced_adc_t;


typedef enum
{
    WICED_UART_1,   /* ChipCommon Slow UART */
    WICED_UART_2,   /* ChipCommon Fast UART */
    WICED_UART_3,   /* GCI UART */
    WICED_UART_MAX, /* Denotes the total number of UART port aliases. Not a valid UART alias */
} wiced_uart_t;

typedef enum
{
    WICED_I2S_1,
    WICED_I2S_2,
    WICED_I2S_3,
    WICED_I2S_4,
    WICED_I2S_5,
    WICED_I2S_MAX, /* Denotes the total number of I2S port aliases. Not a valid I2S alias */
} wiced_i2s_t;

/* Logical Button-ids which map to phyiscal buttons on the board */
typedef enum
{
    PLATFORM_BUTTON_1,      /* Back         */
    PLATFORM_BUTTON_2,      /* Volume Down  */
    PLATFORM_BUTTON_3,      /* Volume Up    */
    PLATFORM_BUTTON_4,      /* Pause / Play */
    PLATFORM_BUTTON_5,      /* Multi        */
    PLATFORM_BUTTON_6,      /* Forward      */
    PLATFORM_BUTTON_MAX, /* Denotes the total number of Buttons on the board. Not a valid Button alias */
} platform_button_t;

#define WICED_PLATFORM_BUTTON_COUNT  ( 6 )

#define WICED_PLATFORM_INCLUDES_SPI_FLASH
#define WICED_SPI_FLASH_PORT (WICED_SPI_2)
#define WICED_SPI_FLASH_CS   (WICED_GPIO_NONE) /* No GPIO available, must use controller */

#define WICED_GMAC_PHY_RESET (WICED_GPIO_5)

#define WICED_USB_HOST_OVERCURRENT      (WICED_GPIO_1)
#define WICED_USB_HOST_POWER_ENABLE     (WICED_GPIO_12)

/*No external GPIO LEDS are available*/
#define GPIO_LED_NOT_SUPPORTED

/* Components connected to external I/Os*/
/* Note: The BCM943909WCD1_3 hardware does not have physical LEDs or buttons so the #defines are mapped to PWM 0,1,2 on the expansion header */
#define WICED_LED1       (WICED_GPIO_18)
#define WICED_LED2       (WICED_GPIO_19)
#define WICED_BUTTON1    (WICED_GPIO_20)
#define WICED_BUTTON2    (WICED_GPIO_2)
//#define WICED_THERMISTOR (WICED_GPIO_4)
#define WICED_GPIO_AUTH_RST  ( WICED_GPIO_6 )
#define WICED_GPIO_AKM_PDN   ( WICED_GPIO_8 )

/* I/O connection <-> Peripheral Connections */
//#define WICED_LED1_JOINS_PWM       (WICED_PWM_1)
//#define WICED_LED2_JOINS_PWM       (WICED_PWM_2)
//#define WICED_THERMISTOR_JOINS_ADC (WICED_ADC_3)

#define AUTH_IC_I2C_PORT (WICED_I2C_2)

/* common audio device defines */
#define PLATFORM_DEFAULT_AUDIO_INPUT    AUDIO_DEVICE_ID_AK4954_ADC_LINE
#define PLATFORM_AUDIO_NUM_INPUTS       3
#define PLATFORM_DEFAULT_AUDIO_OUTPUT   AUDIO_DEVICE_ID_AK4954_DAC_LINE
#define PLATFORM_AUDIO_NUM_OUTPUTS      3

#define WICED_BUTTON_MULTI_FUNC         ( WICED_GPIO_5 )
#define WICED_BUTTON_PAUSE_PLAY         ( WICED_GPIO_18 )
#define WICED_BUTTON_VOLUME_UP          ( WICED_GPIO_17 )
#define WICED_BUTTON_VOLUME_DOWN        ( WICED_GPIO_12 )
#define WICED_BUTTON_BACK               ( WICED_GPIO_15 )
#define WICED_BUTTON_FORWARD            ( WICED_GPIO_16 )

/* Logical Button-ids which map to phyiscal buttons on the board (see platform_gpio_buttons[] in platform.c)*/
#define PLATFORM_BUTTON_BACK                    ( PLATFORM_BUTTON_1 )
#define PLATFORM_BUTTON_VOLUME_DOWN             ( PLATFORM_BUTTON_2 )
#define PLATFORM_BUTTON_VOLUME_UP               ( PLATFORM_BUTTON_3 )
#define PLATFORM_BUTTON_PAUSE_PLAY              ( PLATFORM_BUTTON_4 )
#define PLATFORM_BUTTON_MULTI_FUNC              ( PLATFORM_BUTTON_5 )
#define PLATFORM_BUTTON_FORWARD                 ( PLATFORM_BUTTON_6 )

/* Bootloader OTA and OTA2 factory reset during settings */
#define PLATFORM_FACTORY_RESET_BUTTON_INDEX     ( PLATFORM_BUTTON_1 )
#define PLATFORM_FACTORY_RESET_TIMEOUT          ( 10000 )

/* Generic button checking defines */
#define PLATFORM_BUTTON_PRESS_CHECK_PERIOD      ( 100 )
#define PLATFORM_BUTTON_PRESSED_STATE           (   0 )

#define PLATFORM_GREEN_LED_INDEX                ( WICED_LED_INDEX_1 )
#define PLATFORM_RED_LED_INDEX                  ( WICED_LED_INDEX_2 )

#ifdef __cplusplus
} /*extern "C" */
#endif
