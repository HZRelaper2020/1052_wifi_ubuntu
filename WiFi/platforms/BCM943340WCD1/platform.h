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

/** @file
 * Defines peripherals available for use on BCM943340WCD1 board
 */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif
/*
BCM943340WCD0 platform pin definitions ...
+----------------------------------------------------------------------------------------------------+
|Pin |   Pin Name on           |    Module     | STM32| Peripheral      |    Board     | Peripheral  |
| #  |      Module             |  GPIO Alias   | Port | Available       |  Connection  |   Alias     |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 4  | MODULE_MICRO_JTAG_TDI   | WICED_GPIO_1  | A 15 | JTDI            |              |             |
|    |                         |               |      | SPI3_NSS        |              |             |
|    |                         |               |      | I2S3_WS         |              |             |
|    |                         |               |      | TIM2_CH1_ETR    |              |             |
|    |                         |               |      | SPI1_NSS        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 5  | MODULE_MICRO_JTAG_TDO   | WICED_GPIO_2  | B 3  | JTDO            |              |             |
|    |                         |               |      | TRACESWO        |              |             |
|    |                         |               |      | SPI3_SCK        |              |             |
|    |                         |               |      | I2S3_CK         |              |             |
|    |                         |               |      | TIM2_CH2        |              |             |
|    |                         |               |      | SPI1_SCK        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 6  |MODULE_MICRO_JTAG_TRSTN  | WICED_GPIO_3  | B 4  | NJTRST          |              |             |
|    |                         |               |      | SPI3_MISO       |              |             |
|    |                         |               |      | TIM3_CH1        |              |             |
|    |                         |               |      | SPI1_MISO       |              |             |
|    |                         |               |      | I2S3ext_SD      |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 8  | MODULE_MICRO_UART1_TXD  | WICED_GPIO_4  | G 14 | FSMC_A25        |              |             |
|    |                         |               |      | USART6_TX       |              |             |
|    |                         |               |      | ETH_MII_TXD1    |              |             |
|    |                         |               |      | ETH_RMII_TXD1   |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 9  | MODULE_MICRO_UART1_RXD  | WICED_GPIO_5  | G 9  | USART6_RX       |              |             |
|    |                         |               |      | FSMC_NE2        |              |             |
|    |                         |               |      | FSMC_NCE3       |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 10 | MODULE_MICRO_UART1_RTS  | WICED_GPIO_6  | G 12 | FSMC_NE4        |              |             |
|    |                         |               |      | USART6_RTS      |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 11 | MODULE_MICRO_UART1_CTS  | WICED_GPIO_7  | G 13 | FSMC_A24        |              |             |
|    |                         |               |      | USART6_CTS      |              |             |
|    |                         |               |      | ETH_MII_TXD0    |              |             |
|    |                         |               |      | ETH_RMII_TXD0   |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 12 | MODULE_MICRO_GPIO_1     | WICED_GPIO_8  | B 5  | I2C1_SMBA       |   LED D5     |             |
|    |                         |               |      | CAN2_RX         |              |             |
|    |                         |               |      | OTG_HS_ULPI_D7  |              |             |
|    |                         |               |      | ETH_PPS_OUT     |              |             |
|    |                         |               |      | TIM3_CH2        |              |             |
|    |                         |               |      | SPI1_MOSI       |              |             |
|    |                         |               |      | SPI3_MOSI       |              |             |
|    |                         |               |      | DCMI_D10        |              |             |
|    |                         |               |      | I2S3_SD         |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 13 | MODULE_MICRO_GPIO_0     | WICED_GPIO_9  | B 6  | I2C1_SCL        |   LED D7     |             |
|    |                         |               |      | TIM4_CH1        |              |             |
|    |                         |               |      | CAN2_TX         |              |             |
|    |                         |               |      | DCMI_D5         |              |             |
|    |                         |               |      | USART1_TX       |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 19 | MODULE_MICRO_ADC1       | WICED_GPIO_10 | A 1  | USART2_RTS      | BUTTON SW2   | WICED_ADC_1 |
|    |                         |               |      | UART4_RX        |              |             |
|    |                         |               |      | ETH_RMII_REF_CLK|              |             |
|    |                         |               |      | ETH_MII_RX_CLK  |              |             |
|    |                         |               |      | TIM5_CH2        |              |             |
|    |                         |               |      | TIM2_CH2        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|    |                         |               |      | ADC123_IN1      |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 20 | MODULE_MICRO_ADC2       | WICED_GPIO_11 | A 2  | USART2_TX       | BUTTON SW3   | WICED_ADC_2 |
|    |                         |               |      | TIM5_CH3        |              |             |
|    |                         |               |      | TIM9_CH1        |              |             |
|    |                         |               |      | TIM2_CH3        |              |             |
|    |                         |               |      | ETH_MDIO        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|    |                         |               |      | ADC123_IN2      |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 21 | MODULE_MICRO_ADC3       | WICED_GPIO_12 | A 3  | USART2_RX       | THERMISTOR   | WICED_ADC_3 |
|    |                         |               |      | TIM5_CH4        |              |             |
|    |                         |               |      | TIM9_CH2        |              |             |
|    |                         |               |      | TIM2_CH4        |              |             |
|    |                         |               |      | OTG_HS_ULPI_D0  |              |             |
|    |                         |               |      | ETH_MII_COL     |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|    |                         |               |      | ADC123_IN3      |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 22 | MODULE_MICRO_UART2_TXD  | WICED_GPIO_13 | D 5  | FSMC_NWE        |              |             |
|    |                         |               |      | USART2_TX       |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 23 | MODULE_MICRO_UART2_RXD  | WICED_GPIO_14 | D 6  | FSMC_NWAIT      |              |             |
|    |                         |               |      | USART2_RX       |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 24 | MODULE_MICRO_UART2_CTS  | WICED_GPIO_15 | D 3  | FSMC_CLK        |              |             |
|    |                         |               |      | USART2_CTS      |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 25 | MODULE_MICRO_UART2_RTS  | WICED_GPIO_16 | D 4  | FSMC_NOE        |              |             |
|    |                         |               |      | USART2_RTS      |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 26 | MODULE_MICRO_SPI_MISO   | WICED_GPIO_17 | A 6  | SPI1_MISO       |              |             |
|    |                         |               |      | TIM8_BKIN       |              |             |
|    |                         |               |      | TIM13_CH1       |              |             |
|    |                         |               |      | DCMI_PIXCLK     |              |             |
|    |                         |               |      | TIM3_CH1        |              |             |
|    |                         |               |      | TIM1_BKIN       |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|    |                         |               |      | ADC12_IN6       |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 27 | MODULE_MICRO_SPI_MOSI   | WICED_GPIO_18 | A 7  | SPI1_MOSI       |              |             |
|    |                         |               |      | TIM8_CH1N       |              |             |
|    |                         |               |      | TIM14_CH1       |              |             |
|    |                         |               |      | TIM3_CH2        |              |             |
|    |                         |               |      | ETH_MII_RX_DV   |              |             |
|    |                         |               |      | TIM1_CH1N       |              |             |
|    |                         |               |      | ETH_RMII_CRS_DV |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|    |                         |               |      | ADC12_IN7       |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 28 | MODULE_MICRO_SPI_CLK    | WICED_GPIO_19 | A 5  | SPI1_SCK        |              |             |
|    |                         |               |      | OTG_HS_ULPI_CK  |              |             |
|    |                         |               |      | TIM2_CH1_ETR    |              |             |
|    |                         |               |      | TIM8_CH1N       |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|    |                         |               |      | ADC12_IN5       |              |             |
|    |                         |               |      | DAC_OUT2        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 29 | MODULE_MICRO_WAKE       | WICED_GPIO_20 | A 0  | USART2_CTS      |              |             |
|    |                         |               |      | UART4_TX        |              |             |
|    |                         |               |      | ETH_MII_CRS     |              |             |
|    |                         |               |      | TIM2_CH1_ETR    |              |             |
|    |                         |               |      | TIM5_CH1        |              |             |
|    |                         |               |      | TIM8_ETR        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|    |                         |               |      | ADC123_IN0      |              |             |
|    |                         |               |      | WKUP            |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 31 | MODULE_MICRO_SPI_CS     | WICED_GPIO_21 | A 4  | SPI1_NSS        |              |             |
|    |                         |               |      | SPI3_NSS        |              |             |
|    |                         |               |      | USART2_CK       |              |             |
|    |                         |               |      | DCMI_HSYNC      |              |             |
|    |                         |               |      | OTG_HS_SOF      |              |             |
|    |                         |               |      | I2S3_WS         |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|    |                         |               |      | ADC12_IN4       |              |             |
|    |                         |               |      | DAC_OUT1        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 32 | MODULE_MICRO_GPIO7      | WICED_GPIO_22 | B 10 | SPI2_SCK        |              |             |
|    |                         |               |      | I2S2_CK         |              |             |
|    |                         |               |      | I2C2_SCL        |              |             |
|    |                         |               |      | USART3_TX       |              |             |
|    |                         |               |      | OTG_HS_ULPI_D3  |              |             |
|    |                         |               |      | ETH_MII_RX_ER   |              |             |
|    |                         |               |      | TIM2_CH3        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 33 | MODULE_MICRO_GPIO6      | WICED_GPIO_23 | B 9  | SPI2_NSS        |              |             |
|    |                         |               |      | I2S2_WS         |              |             |
|    |                         |               |      | TIM4_CH4        |              |             |
|    |                         |               |      | TIM11_CH1       |              |             |
|    |                         |               |      | SDIO_D5         |              |             |
|    |                         |               |      | DCMI_D7         |              |             |
|    |                         |               |      | I2C1_SDA        |              |             |
|    |                         |               |      | CAN1_TX         |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 34 | MODULE_MICRO_GPIO5      | WICED_GPIO_24 | C 3  | SPI2_MOSI       |              |             |
|    |                         |               |      | I2S2_SD         |              |             |
|    |                         |               |      | OTG_HS_ULPI_NXT |              |             |
|    |                         |               |      | ETH_MII_TX_CLK  |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|    |                         |               |      | ADC123_IN13     |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 35 | MODULE_MICRO_GPIO4      | WICED_GPIO_25 | C 6  | I2S2_MCK        |              |             |
|    |                         |               |      | TIM8_CH1        |              |             |
|    |                         |               |      | SDIO_D6         |              |             |
|    |                         |               |      | USART6_TX       |              |             |
|    |                         |               |      | DCMI_D0         |              |             |
|    |                         |               |      | TIM3_CH1        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 36 | MODULE_MICRO_GPIO3      | WICED_GPIO_26 | H 5  | I2C2_SDA        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|    |                         |               | H 6  | I2C2_SMBA       |              |             |
|    |                         |               |      | TIM12_CH1       |              |             |
|    |                         |               |      | ETH_MII_RXD2    |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 37 | MODULE_MICRO_GPIO2      | WICED_GPIO_27 | H 4  | I2C2_SCL        |              |             |
|    |                         |               |      | OTG_HS_ULPI_NXT |              |             |
+----+-------------------------+---------------+------+-----------------+--------------+-------------+
Notes
The 43340 platform is still under development. The UART and GPIO are the only peripherals that has been implemented
and lightly tested.

1. The mappings in the table above are defined in <WICED-SDK>/platforms/BCM943340WCD1/platform.c
2. STM32F4xx Datasheet  ->
3. STM32F4xx Ref Manual ->
*/


/******************************************************
 *                   Enumerations
 ******************************************************/

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
    WICED_GPIO_MAX, /* Denotes the total number of GPIO port aliases. Not a valid GPIO alias */
    WICED_GPIO_32BIT = 0x7FFFFFFF
} wiced_gpio_t;

typedef enum
{
    WICED_SPI_NONE, /* SPI unavailable */
    WICED_SPI_MAX,  /* Denotes the total number of SPI port aliases. Not a valid SPI alias */
    WICED_SPI_32BIT = 0x7FFFFFFF,
} wiced_spi_t;

typedef enum
{
    WICED_I2C_1, /* I2C available */
    WICED_I2C_2,
    WICED_I2C_MAX,  /* Denotes the total number of I2C port aliases. Not a valid I2C alias */
    WICED_I2C_32BIT = 0x7FFFFFFF,
} wiced_i2c_t;

typedef enum
{
    WICED_I2S_NONE, /* I2S unavailable */
    WICED_I2S_MAX,  /* Denotes the total number of I2S port aliases. Not a valid I2S alias */
    WICED_I2S_32BIT = 0x7FFFFFFF,
} wiced_i2s_t;

typedef enum
{
    WICED_PWM_1,
    WICED_PWM_2,
    WICED_PWM_MAX,  /* Denotes the total number of PWM port aliases. Not a valid PWM alias */
    WICED_PWM_32BIT = 0x7FFFFFFF,
} wiced_pwm_t;

typedef enum
{
    WICED_ADC_1,
    WICED_ADC_2,
    WICED_ADC_3,
    WICED_ADC_MAX,  /* Denotes the total number of ADC port aliases. Not a valid ADC alias */
    WICED_ADC_32BIT = 0x7FFFFFFF,
} wiced_adc_t;

typedef enum
{
    WICED_UART_1,
    WICED_UART_MAX, /* Denotes the total number of UART port aliases. Not a valid UART alias */
    WICED_UART_32BIT = 0x7FFFFFFF,
} wiced_uart_t;

/* Logical Button-ids which map to phyiscal buttons on the board */
typedef enum
{
    PLATFORM_BUTTON_1,
    PLATFORM_BUTTON_2,
    PLATFORM_BUTTON_MAX, /* Denotes the total number of Buttons on the board. Not a valid Button Alias */
} platform_button_t;

/******************************************************
 *                    Constants
 ******************************************************/

#define WICED_PLATFORM_BUTTON_COUNT  ( 2 )

/* UART port used for standard I/O */
#define STDIO_UART                       ( WICED_UART_1 )

/* SPI Flash. Unimplemented ... */
/* #define WICED_PLATFORM_INCLUDES_SPI_FLASH              */
/* #define WICED_SPI_FLASH_CS            ( WICED_GPIO_5 ) */
/* #define WICED_SPI_FLASH_MOSI          ( WICED_GPIO_8 ) */
/* #define WICED_SPI_FLASH_MISO          ( WICED_GPIO_7 ) */
/* #define WICED_SPI_FLASH_CLK           ( WICED_GPIO_6 ) */

/* Components connected to external I/Os */
#define PLATFORM_LED_COUNT               ( 2 )
#define WICED_LED1                       ( WICED_GPIO_8 )
#define WICED_LED2                       ( WICED_GPIO_9 )
#define WICED_LED1_ON_STATE              ( WICED_ACTIVE_HIGH )
#define WICED_LED2_ON_STATE              ( WICED_ACTIVE_HIGH )
#define WICED_BUTTON1                    ( WICED_GPIO_10 )
#define WICED_BUTTON2                    ( WICED_GPIO_11 )
#define WICED_THERMISTOR                 ( WICED_GPIO_12 )

#define WICED_GPIO_AUTH_RST              ( WICED_GPIO_7 )
#define WICED_GPIO_AUTH_SCL              ( WICED_GPIO_27 )
#define WICED_GPIO_AUTH_SDA              ( WICED_GPIO_11 )

/* Authentication Chip <-> I2C Peripheral */
#define WICED_I2C_AUTH                   ( WICED_I2C_1 )
#define WICED_I2C_AUTH_DMA               ( I2C_DEVICE_USE_DMA )

/* I/O connection <-> Peripheral Connections */
#define WICED_LED1_JOINS_PWM          ( WICED_PWM_1 )
#define WICED_LED2_JOINS_PWM          ( WICED_PWM_2 )
#define WICED_THERMISTOR_JOINS_ADC    ( WICED_ADC_3 )

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
