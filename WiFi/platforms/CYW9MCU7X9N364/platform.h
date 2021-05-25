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
 * Defines peripherals available for use on CYW9MCU7X9N364
 */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    WICED_GPIO_0,
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
    WICED_GPIO_MAX, /* Denotes the total number of GPIO port aliases. Not a valid GPIO alias */
} wiced_gpio_t;


typedef enum
{
    WICED_SPI_1,
    WICED_SPI_2,
    WICED_SPI_MAX, /* Denotes the total number of SPI port aliases. Not a valid SPI alias */
    WICED_SPI_32BIT = 0x7FFFFFFF,
} wiced_spi_t;

typedef enum
{
    WICED_I2C_1,
    WICED_I2C_MAX,
    WICED_I2C_32BIT = 0x7FFFFFFF,
} wiced_i2c_t;

typedef enum
{
    WICED_PWM_1,
    WICED_PWM_2,
    WICED_PWM_3,
    WICED_PWM_4,
    WICED_PWM_MAX, /* Denotes the total number of PWM port aliases. Not a valid PWM alias */
    WICED_PWM_32BIT = 0x7FFFFFFF,
} wiced_pwm_t;
/*ADC channel nos below are mapped to be in sync with the gpio pins corresponding to it
 * Ex - WICED_ADC_1 maps to GPIO 0 for input */
typedef enum
{
    WICED_ADC_1,
    WICED_ADC_2,
    WICED_ADC_3,
    WICED_ADC_4,
    WICED_ADC_5,
    WICED_ADC_6,
    WICED_ADC_7,
    WICED_ADC_8,
    WICED_ADC_9,
    WICED_ADC_10,
    WICED_ADC_11,
    WICED_ADC_12,
    WICED_ADC_13,
    WICED_ADC_14,
    WICED_ADC_15,
    WICED_ADC_16,
    WICED_ADC_17,
    WICED_ADC_18,
    WICED_ADC_19,
    WICED_ADC_20,
    WICED_ADC_21,
    WICED_ADC_22,
    WICED_ADC_23,
    WICED_ADC_24,
    WICED_ADC_25,
    WICED_ADC_26,
    WICED_ADC_27,
    WICED_ADC_28,
    WICED_ADC_MAX, /* Denotes the total number of ADC port aliases. Not a valid ADC alias */
    WICED_ADC_32BIT = 0x7FFFFFFF,
} wiced_adc_t;

typedef enum
{
    WICED_UART_0,
    WICED_UART_MAX, /* Denotes the total number of UART port aliases. Not a valid UART alias */
} wiced_uart_t;


/* Logical Button-ids which map to phyiscal buttons on the board */
typedef enum
{
    PLATFORM_BUTTON_1,
    PLATFORM_BUTTON_MAX, /* Denotes the total number of Buttons on the board. Not a valid Button Alias */
} platform_button_t;
/******************************************************
 *                    Constants
 ******************************************************/

/* UART port used for standard I/O */
#define STDIO_UART                          ( WICED_UART_0 )

#define WICED_BT_WIFI_REG_ON_PIN            ( WICED_GPIO_7 )
#define WICED_WIFI_BT_SPI_IRQ_PIN           ( WICED_GPIO_1 )
/* Components connected to external I/Os */
#define WICED_BUTTON_TOGGLE                 ( WICED_GPIO_24 )
#define WICED_BUTTON_SELECT                 ( WICED_GPIO_25 )

#define WICED_PLATFORM_BUTTON_COUNT  ( 1 )
#define WICED_PLATFORM_MAX_BUTTON  (1)
/* Only BUTTON1 is VALID */
#define WICED_BUTTON1              ( WICED_GPIO_25 )
#define WICED_BUTTON2              ( WICED_GPIO_38 )

/* Components connected to external I/Os */
#define PLATFORM_LED_COUNT                  ( 2 )
#define WICED_LED1                          ( WICED_GPIO_29 )
#define WICED_LED2                          ( WICED_GPIO_28 )

#define WICED_I2C_SCL                       ( WICED_GPIO_10 )
#define WICED_I2C_SDA                       ( WICED_GPIO_11 )
#define WICED_I2C_RST                       ( WICED_GPIO_12 )
/*Define platform heap size in bytes*/
#ifndef PLATFORM_HEAP_SIZE
#define PLATFORM_HEAP_SIZE (30*1024)
#endif

/*default wiced app stack is 6k*/
#ifndef APPLICATION_STACK_SIZE
#define APPLICATION_STACK_SIZE   (6*1024)
#endif

/* PIN to wake BT MCU from sleep */
#ifndef WICED_BT_MCU_WAKE_PIN
#define WICED_BT_MCU_WAKE_PIN   WICED_WIFI_BT_SPI_IRQ_PIN
#endif

#ifndef WICED_BT_MCU_WAKE_MODE
#define WICED_BT_MCU_WAKE_MODE  ( WICED_SLEEP_WAKE_ACTIVE_LOW )
#endif

#define WICED_LED1_JOINS_PWM        ( WICED_PWM_3 )
#define WICED_LED2_JOINS_PWM        ( WICED_PWM_4 )
#define WICED_THERMISTOR_JOINS_ADC  ( WICED_ADC_3 )

#define GCI_SECI_IN_PIN ( WICED_GPIO_17 )
#define GCI_SECI_OUT_PIN ( WICED_GPIO_16 )

/* Bootloader OTA and OTA2 factory reset during settings */
#define PLATFORM_FACTORY_RESET_BUTTON_INDEX     ( PLATFORM_BUTTON_1 )
#define PLATFORM_FACTORY_RESET_TIMEOUT          ( 10000 )

/* Generic button checking defines */
#define PLATFORM_BUTTON_PRESS_CHECK_PERIOD      ( 100 )
#define PLATFORM_BUTTON_PRESSED_STATE           (   1 )

#define PLATFORM_GREEN_LED_INDEX                ( WICED_LED_INDEX_1 )
#define PLATFORM_RED_LED_INDEX                  ( WICED_LED_INDEX_2 )

#define PLATFORM_SWDCK_PIN                      ( WICED_GPIO_34 )
#define PLATFORM_SWDIO_PIN                      ( WICED_GPIO_33 )

#ifdef BT_CHIP_REVISION_B0
/*  This is required for 20739B0 based platforms*/
#define wiced_hal_puart_select_uart_pads wiced_hal_puart_select_uart_pads_patch

#endif

#ifdef __cplusplus
} /*extern "C" */
#endif
