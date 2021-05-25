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
QuickSilver platform pin definitions ...
+--------------------------------------------------------------------------------------------------------+--------------+-------------------------------+
| Enum ID       |Pin  |   Pin Name   |  SIP Pin  Name          | SIP |  Module Pin              | Module | Function     | Board Connection              |
|               | #   |   on 43907   |                         | Pin#|  Name                    | Pin#   |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_1  | 141 | GPIO_0       | GPIO_0                  | C11 | PINW                     | 28     |  GPIO        | PINW                          |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_2  | 142 | GPIO_1       | GPIO_1_GSPI_MODE        | A30 |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_3  | 95  | GPIO_7       | GPIO_7_WCPU_BOOT_MODE   | A8  |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_4  | 134 | GPIO_8       | GPIO_8_TAP_SEL          | C16 | PINN                     | 32     |  GPIO        | PINN                          |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_5  | 94  | GPIO_9       | GPIO_9_USB_SEL          | C15 |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_6  | 146 | GPIO_10      | GPIO_10                 | A45 | PINH                     | 41     |  GPIO        | PINH                          |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_7  | 140 | GPIO_11      | GPIO_11_ACPU_BOOT_MODE  | C4  |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_8  | 143 | GPIO_12      | GPIO_12                 | C14 | PINY                     | 31     |  GPIO        | PINY                          |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_9  | 131 | GPIO_13      | GPIO_13_SDIO_MODE       | B28 |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_10 | 130 | GPIO_14      | GPIO_14                 | D8  | PINT                     | 33     |  GPIO        | PINT                          |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_11 | 145 | GPIO_15      | GPIO_15                 | B26 |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_12 | 137 | GPIO_16      | GPIO_16                 | B19 | PINR                     | 48     |  GPIO        | PINR                          |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_13 | 185 | PWM0         | PWM_0                   | C10 |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_14 | 186 | PWM1         | PWM_1                   | C9  |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_15 | 190 | PWM2         | PWM_2                   | A27 | PINF                     | 17     |  GPIO        | PINF                          |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_16 | 189 | PWM3         | PWM_3                   | A26 | PINE                     | 16     |  GPIO        | PINE                          |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_17 | 187 | PWM4         | PWM_4                   | A31 | PING                     | 27     |  GPIO        | PING                          |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_18 | 188 | PWM5         | PWM_5                   | D7  |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_19 | 78  | SPI0_MISO    | SPI_0_MISO              | A12 | SPI_ABCD_MISO            | 12     |  SPI         | SPIABCD_MISO                  |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_20 | 76  | SPI0_CLK     | SPI_0_CLK               | A11 | SPI_ABCD_SCLK            | 11     |  SPI         | SPIABCD_CLK                   |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_21 | 81  | SPI0_SISO    | SPI_0_MOSI              | A9  | SPI_ABCD_MOSI            | 9      |  SPI         | SPIABCD_MOSI                  |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_22 | 82  | SPI0_CS      | SPI_0_CS                | A10 | SPI_ABCD_CS              | 10     |  SPI         | SPIABCD_CS                    |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_23 | 139 | GPIO_2       | GPIO_2_JTAG_TCK         | C13 | JTAG_TCK                 | 61     |  JTAG        | JTAG_TCK                      |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_24 | 144 | GPIO_3       | GPIO_3_JTAG_TMS         | B20 | JTAG_TMS                 | 58     |  JTAG        | JTAG_TMS                      |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_25 | 136 | GPIO_4       | GPIO_4_JTAG_TDI         | C3  | JTAG_TDI                 | 60     |  JTAG        | JTAG_TDI                      |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_26 | 132 | GPIO_5       | GPIO_5_JTAG_TDO         | B4  | JTAG_TDO                 | 57     |  JTAG        | JTAG_TDO                      |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_27 | 133 | GPIO_6       | GPIO_6_JTAG_TRST_L      | C2  | JTAG_TRST                | 59     |  JTAG        | JTAG_TRST                     |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_28 | 153 | I2S_MCLK0    | I2S0_MCK                | B36 | LED_GRN                  | 46     |  GPIO        | LED_GREEN                     |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_29 | 154 | I2S_SCLK0    | I2S0_SCK_BCLK           | B35 | LED_RED                  | 45     |  GPIO        | LED_RED                       |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_30 | 163 | I2S0_WS_LRCLK| I2S0_WS_LRCLK           | A51 | BLINKUP_DIN              | 29     |  GPIO        | xx                            |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_31 | 164 | I2S0_SD_IN   | I2S0_SD_IN              | A50 | BLINKUP_EN_L             | 30     |  GPIO        | xx                            |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_32 | 161 | I2S_SDATAO0  | I2S0_SD_OUT             | A48 |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_33 | 158 | I2S_MCLK1    | I2S1_MCK                | D5  |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_34 | 160 | I2S_SCLK1    | I2S1_SCK_BCLK           | B1  | PINL                     | 49     |  GPIO        | PINL                          |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_35 | 162 | I2S_LRCLK1   | I2S1_WS_LRCLK           | A49 | PINM                     | 47     |  GPIO        | PINM                          |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_36 | 156 | I2S_SDATAI1  | I2S1_SD_IN              | B38 | PINP                     | 48     |  GPIO        | PINP                          |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_37 | 155 | I2S_SDATAO1  | I2S1_SD_OUT             | B37 |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_38 | 83  | SPI1_CLK     | SPI_1_CLK               | B32 | SPI0_SCLK                | 39     |  SPI         | SPI0_CLK                      |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_39 | 84  | SPI1_MISO    | SPI_1_MISO              | B31 | SPI0_MISO                | 38     |  SPI         | SPI0_MISO                     |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_40 | 86  | SPI1_SISO    | SPI_1_MOSI              | B30 | SPI0_MOSI                | 37     |  SPI         | SPI0_MOSI                     |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_41 | 80  | SPI1_CS      | SPI_1_CS                | B33 | SPI0_CS_L                | 40     |  SPI         | SPI0_CS                       |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_42 | 152 | SDIO_CLK     | SDIO_CLK                | B25 |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_43 | 151 | SDIO_CMD     | SDIO_CMD                | B24 |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_44 | 150 | SDIO_DATA0   | SDIO_DATA0              | B23 |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_45 | 149 | SDIO_DATA1   | SDIO_DATA1              | B22 |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_46 | 148 | SDIO_DATA2   | SDIO_DATA2              | B21 |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_47 | 147 | SDIO_DATA3   | SDIO_DATA3              | B27 |                          | xx     |              |                               |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_48 | 93  | I2C0_SDATA   | I2C_0_SDA               | A46 | PINK                     | 42     |  GPIO        | PINK                          |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_49 | 92  | I2C0_CLK     | I2C_0_SCL               | A47 | PINJ                     | 43     |  GPIO        | PINJ                          |
|---------------+-----+--------------+-------------------------+-----+--------------------------+------- +--------------+-------------------------------|
| WICED_GPIO_50 | 90  | I2C1_SDATA   | I2C_1_SDA               | C1  | I2C0_SDA                 | 55     |  I2C         | I2C0_SDA                      |
|---------------+-----+--------------+-------------------------+-----+--------------------------+--------+--------------+-------------------------------|
| WICED_GPIO_51 | 89  | I2C1_CLK     | I2C_1_SCL               | B2  | I2C0_SCL                 | 54     |  I2C         | I2C0_SCL                      |
+-------------------------------------------------------------------------------------------------------------------------------------------------------+


+--------------------------------------------------------------------------------------------------------------------------------------+
| Enum ID                 |Pin  |   Pin Name on |  SIP Pin Name                            | SIP |  Module Pin                 | Module|
|                         | #   |      43907    |                                          | Pin#|  Name                       | Pin#  |
|-------------------------+-----+---------------+------------------------------------------+-----+-----------------------------+-------|
| WICED_PERIPHERAL_PIN_1  | 202 | UART1_TXD     | UART1_TXD_OUT                            | B18 |  UART1_TXD_OUT              | 22    |
|-------------------------+-----+---------------+------------------------------------------+-----+-----------------------------+-------|
| WICED_PERIPHERAL_PIN_2  | 201 | UART1_RXD     | UART1_RXD_IN                             | B15 |  UART1_RXD_IN               | 21    |
|-------------------------+-----+---------------+------------------------------------------+-----+-----------------------------+-------|
| WICED_PERIPHERAL_PIN_3  | 88  | UART0_RXD     | UART0_RXD_IN                             | A52 |  UART0_RXD_IN               | 50    |
|-------------------------+-----+---------------+------------------------------------------+-----+-----------------------------+-------|
| WICED_PERIPHERAL_PIN_4  | 87  | UART0_TXD     | UART0_TXD_OUT                            | A55 |  UART0_TXD_OUT              | 53    |
|-------------------------+-----+---------------+------------------------------------------+-----+-----------------------------+-------|
| WICED_PERIPHERAL_PIN_5  | 85  | UART0_CTS     | UART0_CTS_IN                             | A54 |  UART0_CTS_IN               | 52    |
|-------------------------+-----+---------------+------------------------------------------+-----+-----------------------------+-------|
| WICED_PERIPHERAL_PIN_6  | 91  | UART0_RTS     | UART0_RTS_OUT                            | A53 |  UART0_RTS_OUT              | 51    |
|-------------------------+-----+---------------+------------------------------------------+-----+-----------------------------+-------|
| WICED_PERIPHERAL_PIN_7  | 200 | UART2_RXD     | UART2_RXD_IN                             | B14 |  UART2_RXD_IN               | 25    |
|-------------------------+-----+---------------+------------------------------------------+-----+-----------------------------+-------|
| WICED_PERIPHERAL_PIN_8  | 196 | UART2_TXD     | UART2_TXD_OUT                            | C7  |  UART2_TXD_OUT              | 24    |
|-------------------------+-----+---------------+------------------------------------------+-----+-----------------------------+-------+
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
    WICED_GPIO_30,
    WICED_GPIO_31,
    WICED_GPIO_32,
    WICED_GPIO_33,
    WICED_GPIO_34,
    WICED_GPIO_35,
    WICED_GPIO_36,
    WICED_GPIO_37,
    WICED_GPIO_38,
    WICED_GPIO_39,
    WICED_GPIO_40,
    WICED_GPIO_41,
    WICED_GPIO_42,
    WICED_GPIO_43,
    WICED_GPIO_44,
    WICED_GPIO_45,
    WICED_GPIO_46,
    WICED_GPIO_47,
    WICED_GPIO_48,
    WICED_GPIO_49,
    WICED_GPIO_50,
    WICED_GPIO_51,

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
#ifdef USING_EXTERNAL_ADC
   WICED_ADC_1,
   WICED_ADC_2,
   WICED_ADC_3,
   WICED_ADC_4,
   WICED_ADC_5,
   WICED_ADC_6,
   WICED_ADC_MAX,
#else
   WICED_ADC_NONE = 0xff, /* No ADCs */
#endif /* ifdef USING_EXTERNAL_ADC */
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
    WICED_I2S_MAX, /* Denotes the total number of I2S port aliases. Not a valid I2S alias */
} wiced_i2s_t;

/* Logical Button-ids which map to phyiscal buttons on the board */
typedef enum
{
    PLATFORM_BUTTON_1,
    PLATFORM_BUTTON_2,
    PLATFORM_BUTTON_MAX, /* Denotes the total number of Buttons on the board. Not a valid Button alias */
} platform_button_t;

/* Logical LED-ids which map to phyiscal gpio LEDs on the board (see platform_gpio_leds[] in platform.c)*/
typedef enum
{
    PLATFORM_LED_1,
    PLATFORM_LED_2,
    PLATFORM_LED_MAX, /* Denotes the total number of LEDs on the board. Not a valid alias */
} platform_led_t;

#define ARD_GPIO0  (WICED_GPIO_1)
#define ARD_GPIO1  (WICED_GPIO_2)
#define ARD_GPIO2  (WICED_GPIO_9)
#define ARD_GPIO3  (WICED_GPIO_3)
#define ARD_GPIO4  (WICED_GPIO_10)
#define ARD_GPIO5  (WICED_GPIO_12)
#define ARD_GPIO6  (WICED_GPIO_11)
#define ARD_GPIO7  (WICED_GPIO_31)
#define ARD_GPIO8  (WICED_GPIO_36)
#define ARD_GPIO9  (WICED_GPIO_17)

#define ARD_MISO   (WICED_GPIO_8)
#define ARD_SCK    (WICED_GPIO_5)
#define ARD_MOSI   (WICED_GPIO_6)
#define ARD_SS     (WICED_GPIO_7)

#define PLATFORM_LED_COUNT           ( 2 )

#define WICED_PLATFORM_BUTTON_COUNT  ( 2 )

#define WICED_LED1       ( WICED_GPIO_28 )
#define WICED_LED2       ( WICED_GPIO_29 )
#define WICED_BUTTON1    ( WICED_GPIO_18 )
#define WICED_BUTTON2    ( WICED_GPIO_4 )

#define WICED_RGB_CLOCK  ( WICED_GPIO_10 )
#define WICED_RGB_DATA   ( WICED_GPIO_8 )

#define WICED_GMAC_PHY_RESET (WICED_GPIO_15)

#define WICED_GPIO_AUTH_RST  (WICED_GPIO_14)

#define AUTH_IC_I2C_PORT     (WICED_I2C_1)

#define WICED_USB_HOST_OVERCURRENT      (WICED_GPIO_1)
#define WICED_USB_HOST_POWER_ENABLE     (WICED_GPIO_12)

/* no audio on this platform */
#define PLATFORM_DEFAULT_AUDIO_INPUT    AUDIO_DEVICE_ID_NONE
#define PLATFORM_AUDIO_NUM_INPUTS       (0)
#define PLATFORM_DEFAULT_AUDIO_OUTPUT   AUDIO_DEVICE_ID_OUTPUT_NONE
#define PLATFORM_AUDIO_NUM_OUTPUTS      (0)

/* Bootloader OTA and OTA2 factory reset during settings */
#define PLATFORM_FACTORY_RESET_BUTTON_INDEX     ( PLATFORM_BUTTON_1 )
#define PLATFORM_FACTORY_RESET_TIMEOUT          ( 10000 )

/* Generic button checking defines */
#define PLATFORM_BUTTON_PRESS_CHECK_PERIOD      ( 100 )
#define PLATFORM_BUTTON_PRESSED_STATE           (   0 )

#define PLATFORM_GREEN_LED_INDEX                ( WICED_LED_INDEX_2 )
#define PLATFORM_RED_LED_INDEX                  ( WICED_LED_INDEX_1 )

#ifdef USING_EXTERNAL_ADC
#define ADC_Channel_0                               ((uint8_t)0x00)
#define ADC_Channel_1                               ((uint8_t)0x01)
#define ADC_Channel_2                               ((uint8_t)0x02)
#define ADC_Channel_3                               ((uint8_t)0x03)
#define ADC_Channel_4                               ((uint8_t)0x04)
#define ADC_Channel_5                               ((uint8_t)0x05)
#endif /* ifdef USING_EXTERNAL_ADC */

#ifdef __cplusplus
} /*extern "C" */
#endif
