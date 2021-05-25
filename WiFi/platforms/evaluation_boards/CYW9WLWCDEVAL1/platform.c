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
 * Defines board support package for CYW9WLWCDEVAL1 board
 */
#include "platform.h"
#include "platform_config.h"
#include "platform_init.h"
#include "platform_isr.h"
#include "platform_peripheral.h"
#include "platform_bluetooth.h"
#include "wwd_platform_common.h"
#include "wwd_rtos_isr.h"
#include "wiced_defaults.h"
#include "wiced_platform.h"
#include "platform_mfi.h"
#include "platform_button.h"
#include "gpio_button.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/* GPIO pin table. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_gpio_t platform_gpio_pins[] =
{
    [WICED_GPIO_1]   = { GPIOA,  0 },
    [WICED_GPIO_2]   = { GPIOA,  1 },
    [WICED_GPIO_3]   = { GPIOA,  2 },
    [WICED_GPIO_4]   = { GPIOA,  3 },
    [WICED_GPIO_5]   = { GPIOA,  4 },
    [WICED_GPIO_6]   = { GPIOA,  5 },
    [WICED_GPIO_7]   = { GPIOA,  6 },
    [WICED_GPIO_8]   = { GPIOA,  7 },
    [WICED_GPIO_9]   = { GPIOA,  8 },
    [WICED_GPIO_10]  = { GPIOA,  9 },
    [WICED_GPIO_11]  = { GPIOA, 10 },
    [WICED_GPIO_12]  = { GPIOA, 11 },
    [WICED_GPIO_13]  = { GPIOA, 12 },
    [WICED_GPIO_14]  = { GPIOA, 13 },
    [WICED_GPIO_15]  = { GPIOA, 14 },
    [WICED_GPIO_16]  = { GPIOA, 15 },
    [WICED_GPIO_17]  = { GPIOB,  0 },
    [WICED_GPIO_18]  = { GPIOB,  1 },
    [WICED_GPIO_19]  = { GPIOB,  2 },
    [WICED_GPIO_20]  = { GPIOB,  3 },
    [WICED_GPIO_21]  = { GPIOB,  4 },
    [WICED_GPIO_22]  = { GPIOB,  5 },
    [WICED_GPIO_23]  = { GPIOB,  6 },
    [WICED_GPIO_24]  = { GPIOB,  7 },
    [WICED_GPIO_25]  = { GPIOB,  8 },
    [WICED_GPIO_26]  = { GPIOB,  9 },
    [WICED_GPIO_27]  = { GPIOB, 10 },
    [WICED_GPIO_28]  = { GPIOB, 11 },
    [WICED_GPIO_29]  = { GPIOB, 12 },
    [WICED_GPIO_30]  = { GPIOB, 13 },
    [WICED_GPIO_31]  = { GPIOB, 14 },
    [WICED_GPIO_32]  = { GPIOB, 15 },
    [WICED_GPIO_33]  = { GPIOC,  0 },
    [WICED_GPIO_34]  = { GPIOC,  1 },
    [WICED_GPIO_35]  = { GPIOC,  2 },
    [WICED_GPIO_36]  = { GPIOC,  3 },
    [WICED_GPIO_37]  = { GPIOC,  4 },
    [WICED_GPIO_38]  = { GPIOC,  5 },
    [WICED_GPIO_39]  = { GPIOC,  6 },
    [WICED_GPIO_40]  = { GPIOC,  7 },
    [WICED_GPIO_41]  = { GPIOC,  8 },
    [WICED_GPIO_42]  = { GPIOC,  9 },
    [WICED_GPIO_43]  = { GPIOC, 10 },
    [WICED_GPIO_44]  = { GPIOC, 11 },
    [WICED_GPIO_45]  = { GPIOC, 12 },
    [WICED_GPIO_46]  = { GPIOC, 13 },
    [WICED_GPIO_47]  = { GPIOC, 14 },
    [WICED_GPIO_48]  = { GPIOC, 15 },
    [WICED_GPIO_49]  = { GPIOD,  0 },
    [WICED_GPIO_50]  = { GPIOD,  1 },
    [WICED_GPIO_51]  = { GPIOD,  2 },
    [WICED_GPIO_52]  = { GPIOD,  3 },
    [WICED_GPIO_53]  = { GPIOD,  4 },
    [WICED_GPIO_54]  = { GPIOD,  5 },
    [WICED_GPIO_55]  = { GPIOD,  6 },
    [WICED_GPIO_56]  = { GPIOD,  7 },
    [WICED_GPIO_57]  = { GPIOD,  8 },
    [WICED_GPIO_58]  = { GPIOD,  9 },
    [WICED_GPIO_59]  = { GPIOD, 10 },
    [WICED_GPIO_60]  = { GPIOD, 11 },
    [WICED_GPIO_61]  = { GPIOD, 12 },
    [WICED_GPIO_62]  = { GPIOD, 13 },
    [WICED_GPIO_63]  = { GPIOD, 14 },
    [WICED_GPIO_64]  = { GPIOD, 15 },
    [WICED_GPIO_65]  = { GPIOE,  0 },
    [WICED_GPIO_66]  = { GPIOE,  1 },
    [WICED_GPIO_67]  = { GPIOE,  2 },
    [WICED_GPIO_68]  = { GPIOE,  3 },
    [WICED_GPIO_69]  = { GPIOE,  4 },
    [WICED_GPIO_70]  = { GPIOE,  5 },
    [WICED_GPIO_71]  = { GPIOE,  6 },
    [WICED_GPIO_72]  = { GPIOE,  7 },
    [WICED_GPIO_73]  = { GPIOE,  8 },
    [WICED_GPIO_74]  = { GPIOE,  9 },
    [WICED_GPIO_75]  = { GPIOE, 10 },
    [WICED_GPIO_76]  = { GPIOE, 11 },
    [WICED_GPIO_77]  = { GPIOE, 12 },
    [WICED_GPIO_78]  = { GPIOE, 13 },
    [WICED_GPIO_79]  = { GPIOE, 14 },
    [WICED_GPIO_80]  = { GPIOE, 15 },
    [WICED_GPIO_81]  = { GPIOF,  0 },
    [WICED_GPIO_82]  = { GPIOF,  1 },
    [WICED_GPIO_83]  = { GPIOF,  2 },
    [WICED_GPIO_84]  = { GPIOF,  3 },
    [WICED_GPIO_85]  = { GPIOF,  4 },
    [WICED_GPIO_86]  = { GPIOF,  5 },
    [WICED_GPIO_87]  = { GPIOF,  6 },
    [WICED_GPIO_88]  = { GPIOF,  7 },
    [WICED_GPIO_89]  = { GPIOF,  8 },
    [WICED_GPIO_90]  = { GPIOF,  9 },
    [WICED_GPIO_91]  = { GPIOF, 10 },
    [WICED_GPIO_92]  = { GPIOF, 11 },
    [WICED_GPIO_93]  = { GPIOF, 12 },
    [WICED_GPIO_94]  = { GPIOF, 13 },
    [WICED_GPIO_95]  = { GPIOF, 14 },
    [WICED_GPIO_96]  = { GPIOF, 15 },
    [WICED_GPIO_97]  = { GPIOG,  0 },
    [WICED_GPIO_98]  = { GPIOG,  1 },
    [WICED_GPIO_99]  = { GPIOG,  2 },
    [WICED_GPIO_100] = { GPIOG,  3 },
    [WICED_GPIO_101] = { GPIOG,  4 },
    [WICED_GPIO_102] = { GPIOG,  5 },
    [WICED_GPIO_103] = { GPIOG,  6 },
    [WICED_GPIO_104] = { GPIOG,  7 },
    [WICED_GPIO_105] = { GPIOG,  8 },
    [WICED_GPIO_106] = { GPIOG,  9 },
    [WICED_GPIO_107] = { GPIOG, 10 },
    [WICED_GPIO_108] = { GPIOG, 11 },
    [WICED_GPIO_109] = { GPIOG, 12 },
    [WICED_GPIO_110] = { GPIOG, 13 },
    [WICED_GPIO_111] = { GPIOG, 14 },
    [WICED_GPIO_112] = { GPIOG, 15 },
    [WICED_GPIO_113] = { GPIOH,  0 },
    [WICED_GPIO_114] = { GPIOH,  1 },
    [WICED_GPIO_115] = { GPIOH,  2 },
    [WICED_GPIO_116] = { GPIOH,  3 },
    [WICED_GPIO_117] = { GPIOH,  4 },
    [WICED_GPIO_118] = { GPIOH,  5 },
    [WICED_GPIO_119] = { GPIOH,  6 },
    [WICED_GPIO_120] = { GPIOH,  7 },
    [WICED_GPIO_121] = { GPIOH,  8 },
    [WICED_GPIO_122] = { GPIOH,  9 },
    [WICED_GPIO_123] = { GPIOH, 10 },
    [WICED_GPIO_124] = { GPIOH, 11 },
    [WICED_GPIO_125] = { GPIOH, 12 },
    [WICED_GPIO_126] = { GPIOH, 13 },
    [WICED_GPIO_127] = { GPIOH, 14 },
    [WICED_GPIO_128] = { GPIOH, 15 },
    [WICED_GPIO_129] = { GPIOI,  0 },
    [WICED_GPIO_130] = { GPIOI,  1 },
    [WICED_GPIO_131] = { GPIOI,  2 },
    [WICED_GPIO_132] = { GPIOI,  3 },
    [WICED_GPIO_133] = { GPIOI,  4 },
    [WICED_GPIO_134] = { GPIOI,  5 },
    [WICED_GPIO_135] = { GPIOI,  6 },
    [WICED_GPIO_136] = { GPIOI,  7 },
    [WICED_GPIO_137] = { GPIOI,  8 },
    [WICED_GPIO_138] = { GPIOI,  9 },
    [WICED_GPIO_139] = { GPIOI, 10 },
    [WICED_GPIO_140] = { GPIOI, 11 },
    [WICED_GPIO_141] = { GPIOI, 12 },
    [WICED_GPIO_142] = { GPIOI, 13 },
    [WICED_GPIO_143] = { GPIOI, 14 },
    [WICED_GPIO_144] = { GPIOI, 15 },
    [WICED_GPIO_129] = { GPIOJ,  0 },
    [WICED_GPIO_130] = { GPIOJ,  1 },
    [WICED_GPIO_131] = { GPIOJ,  2 },
    [WICED_GPIO_132] = { GPIOJ,  3 },
    [WICED_GPIO_133] = { GPIOJ,  4 },
    [WICED_GPIO_134] = { GPIOJ,  5 },
    [WICED_GPIO_135] = { GPIOJ,  6 },
    [WICED_GPIO_136] = { GPIOJ,  7 },
    [WICED_GPIO_137] = { GPIOJ,  8 },
    [WICED_GPIO_138] = { GPIOJ,  9 },
    [WICED_GPIO_139] = { GPIOJ, 10 },
    [WICED_GPIO_140] = { GPIOJ, 11 },
    [WICED_GPIO_141] = { GPIOJ, 12 },
    [WICED_GPIO_142] = { GPIOJ, 13 },
    [WICED_GPIO_143] = { GPIOJ, 14 },
    [WICED_GPIO_144] = { GPIOJ, 15 },
    [WICED_GPIO_145] = { GPIOK,  0 },
    [WICED_GPIO_146] = { GPIOK,  1 },
    [WICED_GPIO_147] = { GPIOK,  2 },
    [WICED_GPIO_148] = { GPIOK,  3 },
    [WICED_GPIO_149] = { GPIOK,  4 },
    [WICED_GPIO_150] = { GPIOK,  5 },
    [WICED_GPIO_151] = { GPIOK,  6 },
    [WICED_GPIO_152] = { GPIOK,  7 },
};

/* Bluetooth UART pins. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
static const platform_gpio_t internal_bt_uart_pins[] =
{
    [WICED_BT_PIN_UART_TX ] = { GPIOE, 1 },
    [WICED_BT_PIN_UART_RX ] = { GPIOE, 0 },
    [WICED_BT_PIN_UART_CTS] = { GPIOD, 14 },
    [WICED_BT_PIN_UART_RTS] = { GPIOD, 15 },
};
const platform_gpio_t* wiced_bt_uart_pins[] =
{
    [WICED_BT_PIN_UART_TX ] = &internal_bt_uart_pins[WICED_BT_PIN_UART_TX ],
    [WICED_BT_PIN_UART_RX ] = &internal_bt_uart_pins[WICED_BT_PIN_UART_RX ],
    [WICED_BT_PIN_UART_CTS] = &internal_bt_uart_pins[WICED_BT_PIN_UART_CTS],
    [WICED_BT_PIN_UART_RTS] = &internal_bt_uart_pins[WICED_BT_PIN_UART_RTS],
};

/* UART peripherals and runtime drivers. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_uart_t platform_uart_peripherals[] =
{
    [WICED_UART_1] =
    {
        .port               = USART1,
        .tx_pin             = &platform_gpio_pins[WICED_GPIO_10],
        .rx_pin             = &platform_gpio_pins[WICED_GPIO_11],
        .cts_pin            = NULL,
        .rts_pin            = NULL,
        .tx_dma_config      = { NULL },
        .rx_dma_config =
        {
            .controller     = DMA1,
            .stream         = DMA1_Stream5,
            .channel        = DMA_REQUEST_USART1_RX,
            .irq_vector     = DMA1_Stream5_IRQn,
        },
    },
    [WICED_UART_2] =
    {
        .port               = UART8,
        .tx_pin             = &internal_bt_uart_pins[WICED_BT_PIN_UART_TX ],
        .rx_pin             = &internal_bt_uart_pins[WICED_BT_PIN_UART_RX ],
        .cts_pin            = &internal_bt_uart_pins[WICED_BT_PIN_UART_CTS],
        .rts_pin            = &internal_bt_uart_pins[WICED_BT_PIN_UART_RTS],
        .tx_dma_config      = { NULL },
        .rx_dma_config      =
         {
            .controller     = DMA2,
            .stream         = DMA2_Stream5,
            .channel        = DMA_REQUEST_UART8_RX,
            .irq_vector     = DMA2_Stream5_IRQn,
         },
    },
    [WICED_UART_3] =
    {
        .port               = USART3,
        .tx_pin             = NULL,
        .rx_pin             = NULL,
        .cts_pin            = NULL,
        .rts_pin            = NULL,
        .tx_dma_config      = { NULL },
        .rx_dma_config      = { NULL },

    },
    [WICED_UART_4] =
    {
        .port               = UART4,
        .tx_pin             = NULL,
        .rx_pin             = NULL,
        .cts_pin            = NULL,
        .rts_pin            = NULL,
        .tx_dma_config      = { NULL },
        .rx_dma_config      = { NULL },

    },
    [WICED_UART_5] =
    {
        .port               = UART5,
        .tx_pin             = NULL,
        .rx_pin             = NULL,
        .cts_pin            = NULL,
        .rts_pin            = NULL,
        .tx_dma_config      = { NULL },
        .rx_dma_config      = { NULL },

    },
    [WICED_UART_6] =
    {
        .port               = USART6,
        .tx_pin             = NULL,
        .rx_pin             = NULL,
        .cts_pin            = NULL,
        .rts_pin            = NULL,
        .tx_dma_config      = { NULL },
        .rx_dma_config      = { NULL },

    },
    [WICED_UART_7] =
    {
        .port               = UART7,
        .tx_pin             = NULL,
        .rx_pin             = NULL,
        .cts_pin            = NULL,
        .rts_pin            = NULL,
        .tx_dma_config      = { NULL },
        .rx_dma_config      = { NULL },

    },
    [WICED_UART_8] =
    {
        .port               = USART2,
        .tx_pin             = NULL,
        .rx_pin             = NULL,
        .cts_pin            = NULL,
        .rts_pin            = NULL,
        .tx_dma_config      = { NULL },
        .rx_dma_config      = { NULL },

    },
};
platform_uart_driver_t platform_uart_drivers[WICED_UART_MAX];

/* UART standard I/O configuration */
#ifndef WICED_DISABLE_STDIO
static const platform_uart_config_t stdio_config =
{
    .baud_rate    = 115200,
    .data_width   = DATA_WIDTH_8BIT,
    .parity       = NO_PARITY,
    .stop_bits    = STOP_BITS_1,
    .flow_control = FLOW_CONTROL_DISABLED,
};
#endif

const platform_i2c_t platform_i2c_peripherals[] =
{
    [WICED_I2C_1] =
    {
        .port                    = I2C1,
        .pin_scl                 = &platform_gpio_pins[WICED_GPIO_25],
        .pin_sda                 = &platform_gpio_pins[WICED_GPIO_26],
    },
    [WICED_I2C_2] =
    {
        .port                    = I2C2,
        .pin_scl                 = NULL,
        .pin_sda                 = NULL,
    },
    [WICED_I2C_3] =
    {
        .port                    = I2C3,
        .pin_scl                 = NULL,
        .pin_sda                 = NULL,
    },
    [WICED_I2C_4] =
    {
        .port                    = I2C4,
        .pin_scl                 = NULL,
        .pin_sda                 = NULL,
    },
};

/* PWM peripherals. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_pwm_t platform_pwm_peripherals[] =
{
    [WICED_PWM_1]  = {TIM4,  TIM_CHANNEL_2, &platform_gpio_pins[WICED_GPIO_24]},
    [WICED_PWM_2]  = {TIM1,  TIM_CHANNEL_1, NULL},
    [WICED_PWM_3]  = {TIM2,  TIM_CHANNEL_1, NULL},
    [WICED_PWM_4]  = {TIM3,  TIM_CHANNEL_1, NULL},
    [WICED_PWM_5]  = {TIM5,  TIM_CHANNEL_1, NULL},
    [WICED_PWM_6]  = {TIM8,  TIM_CHANNEL_1, NULL},
    [WICED_PWM_7]  = {TIM12, TIM_CHANNEL_1, NULL},
    [WICED_PWM_8]  = {TIM13, TIM_CHANNEL_1, NULL},
    [WICED_PWM_9]  = {TIM14, TIM_CHANNEL_1, NULL},
    [WICED_PWM_10] = {TIM15, TIM_CHANNEL_1, NULL},
    [WICED_PWM_11] = {TIM16, TIM_CHANNEL_1, NULL},
    [WICED_PWM_12] = {TIM17, TIM_CHANNEL_1, NULL},
};

/* Wi-Fi control pins. Used by WICED/platform/MCU/wwd_platform_common.c */
const platform_gpio_t wifi_control_pins[] =
{
    [WWD_PIN_POWER      ] = { GPIOD,  3 },
    [WWD_PIN_RESET      ] = { NULL },
    [WWD_PIN_32K_CLK    ] = { NULL },
    [WWD_PIN_BOOTSTRAP_0] = { NULL },
    [WWD_PIN_BOOTSTRAP_1] = { NULL },
};

/* Wi-Fi SDIO bus pins. Used by WICED/platform/STM32H7xx/WWD/wwd_SDIO.c */
const platform_gpio_t wifi_sdio_pins[] =
{
    [WWD_PIN_SDIO_OOB_IRQ] = { GPIOD, 9  },
    [WWD_PIN_SDIO_CLK    ] = { GPIOC, 12 },
    [WWD_PIN_SDIO_CMD    ] = { GPIOD, 2  },
    [WWD_PIN_SDIO_D0     ] = { GPIOC, 8  },
    [WWD_PIN_SDIO_D1     ] = { GPIOC, 9  },
    [WWD_PIN_SDIO_D2     ] = { GPIOC, 10 },
    [WWD_PIN_SDIO_D3     ] = { GPIOC, 11 },
};

/* SPI peripherals. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_spi_t platform_spi_peripherals[] =
{
};

const platform_gpio_t wiced_qspi_flash[] =
{
    [WICED_QSPI_PIN_CS]  = { GPIOB, 10 },
    [WICED_QSPI_PIN_CLK] = { GPIOB, 2 },
    [WICED_QSPI_PIN_D0]  = { GPIOD, 11 },
    [WICED_QSPI_PIN_D1]  = { GPIOD, 12 },
    [WICED_QSPI_PIN_D2]  = { GPIOE, 2 },
    [WICED_QSPI_PIN_D3]  = { GPIOD, 13 },
};

/* Bluetooth control pins. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
static const platform_gpio_t internal_bt_control_pins[] =
{
    /* Reset pin unavailable */
    [WICED_BT_PIN_POWER      ] = { GPIOD,  1  },
    [WICED_BT_PIN_HOST_WAKE  ] = { GPIOD,  10 },
    [WICED_BT_PIN_DEVICE_WAKE] = { GPIOD,  0  }
};
const platform_gpio_t* wiced_bt_control_pins[] =
{
    /* Reset pin unavailable */
    [WICED_BT_PIN_POWER      ] = &internal_bt_control_pins[WICED_BT_PIN_POWER      ],
    [WICED_BT_PIN_HOST_WAKE  ] = &internal_bt_control_pins[WICED_BT_PIN_HOST_WAKE  ],
    [WICED_BT_PIN_DEVICE_WAKE] = &internal_bt_control_pins[WICED_BT_PIN_DEVICE_WAKE],
    [WICED_BT_PIN_RESET      ] = NULL,
};

const platform_uart_t*        wiced_bt_uart_peripheral = &platform_uart_peripherals[WICED_UART_2];
platform_uart_driver_t*       wiced_bt_uart_driver     = &platform_uart_drivers[WICED_UART_2];

/* Bluetooth UART configuration. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
const platform_uart_config_t wiced_bt_uart_config =
{
    .baud_rate    = 115200,
    .data_width   = DATA_WIDTH_8BIT,
    .parity       = NO_PARITY,
    .stop_bits    = STOP_BITS_1,
    .flow_control = FLOW_CONTROL_CTS_RTS,
};

/*BT chip specific configuration information*/
const platform_bluetooth_config_t wiced_bt_config =
{
    .patchram_download_mode      = PATCHRAM_DOWNLOAD_MODE_MINIDRV_CMD,
    .patchram_download_baud_rate = 115200,
    .featured_baud_rate          = 115200
};

const gpio_button_t platform_gpio_buttons[] =
{
    [PLATFORM_BUTTON_1] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON1,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },

    [PLATFORM_BUTTON_2] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON2,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },
};

const wiced_gpio_t platform_gpio_leds[PLATFORM_LED_COUNT] =
{
     [WICED_LED_INDEX_1] = WICED_LED1,
     [WICED_LED_INDEX_2] = WICED_LED2,
     [WICED_LED_INDEX_3] = WICED_LED3,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

void platform_init_peripheral_irq_priorities( void )
{
    /* Interrupt priority setup. Called by WICED/platform/MCU/STM32H7xx/platform_init.c */
    NVIC_SetPriority( USART1_IRQn      ,  6 ); /* WICED_UART_1        */
    NVIC_SetPriority( USART2_IRQn      ,  6 ); /* WICED_UART_2        */
    NVIC_SetPriority( USART3_IRQn      ,  6 ); /* WICED_UART_3        */
    NVIC_SetPriority( UART4_IRQn       ,  6 ); /* WICED_UART_4        */
    NVIC_SetPriority( UART5_IRQn       ,  6 ); /* WICED_UART_5        */
    NVIC_SetPriority( USART6_IRQn      ,  6 ); /* WICED_UART_6        */
    NVIC_SetPriority( UART7_IRQn       ,  6 ); /* WICED_UART_7        */
    NVIC_SetPriority( UART8_IRQn       ,  6 ); /* WICED_UART_8        */
    NVIC_SetPriority( EXTI0_IRQn       , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI1_IRQn       , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI2_IRQn       , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI3_IRQn       , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI4_IRQn       , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI9_5_IRQn     , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI15_10_IRQn   , 14 ); /* GPIO                */
}

void platform_led_init( void )
{
    /* Initialise LEDs and turn off by default */
    platform_gpio_init( &platform_gpio_pins[WICED_LED1], OUTPUT_PUSH_PULL );
    platform_gpio_init( &platform_gpio_pins[WICED_LED2], OUTPUT_PUSH_PULL );
    platform_gpio_init( &platform_gpio_pins[WICED_LED3], OUTPUT_PUSH_PULL );
    platform_led_set_state(WICED_LED_INDEX_1, WICED_LED_OFF);
    platform_led_set_state(WICED_LED_INDEX_2, WICED_LED_OFF);
    platform_led_set_state(WICED_LED_INDEX_3, WICED_LED_OFF);
 }

/* LEDs on this platform are active HIGH */
platform_result_t platform_led_set_state(int led_index, int off_on )
{
    if ((led_index >= 0) && (led_index < PLATFORM_LED_COUNT))
    {
        /* LED2 is active high */
        if (led_index == WICED_LED_INDEX_2)
        {
            switch (off_on)
            {
                case WICED_LED_OFF:
                    platform_gpio_output_low( &platform_gpio_pins[platform_gpio_leds[led_index]] );
                    break;
                case WICED_LED_ON:
                    platform_gpio_output_high( &platform_gpio_pins[platform_gpio_leds[led_index]] );
                    break;
            }
        }
        else
        /* LED 1, 3 are active low */
        {
            switch (off_on)
            {
                case WICED_LED_OFF:
                    platform_gpio_output_high( &platform_gpio_pins[platform_gpio_leds[led_index]] );
                    break;
                case WICED_LED_ON:
                    platform_gpio_output_low( &platform_gpio_pins[platform_gpio_leds[led_index]] );
                    break;
            }
        }
        return PLATFORM_SUCCESS;
    }
    return PLATFORM_BADARG;
}

void platform_init_external_devices( void )
{

    platform_gpio_init( &platform_gpio_pins[WICED_BUTTON1], INPUT_PULL_UP );
    platform_gpio_init( &platform_gpio_pins[WICED_BUTTON2], INPUT_PULL_UP );
#ifndef WICED_DISABLE_STDIO
    /* Initialise UART standard I/O */
    platform_stdio_init( &platform_uart_drivers[STDIO_UART], &platform_uart_peripherals[STDIO_UART], &stdio_config );
#endif
}

/* TODO: the following functions are defined for the WICED build to pass
 * they need to be properly implemented.
 */

uint32_t  platform_get_factory_reset_button_time ( uint32_t max_time )
{
    UNUSED_PARAMETER( max_time );
    return 0;
}

platform_result_t platform_spi_init( const platform_spi_t* spi, const platform_spi_config_t* config )
{
    UNUSED_PARAMETER( spi );
    UNUSED_PARAMETER( config );
    return PLATFORM_UNSUPPORTED;
}

platform_result_t platform_spi_deinit( const platform_spi_t* spi )
{
    UNUSED_PARAMETER( spi );
    return PLATFORM_UNSUPPORTED;
}

platform_result_t platform_spi_transfer( const platform_spi_t* spi, const platform_spi_config_t* config, const platform_spi_message_segment_t* segments, uint16_t number_of_segments )
{
    UNUSED_PARAMETER( spi );
    UNUSED_PARAMETER( config );
    UNUSED_PARAMETER( segments );
    UNUSED_PARAMETER( number_of_segments );
    return PLATFORM_UNSUPPORTED;
}

platform_result_t platform_mcu_powersave_disable( void )
{
    return PLATFORM_UNSUPPORTED;
}

platform_result_t platform_mcu_powersave_enable( void )
{
    return PLATFORM_UNSUPPORTED;
}


void platform_mcu_powersave_exit_notify( void )
{
}

platform_result_t platform_mcu_powersave_init(void)
{
    return PLATFORM_SUCCESS;
}

void platform_idle_hook( void )
{
    __asm("wfi");
}

/* END TODO */

/******************************************************
 *           Interrupt Handler Definitions
 ******************************************************/

WWD_RTOS_DEFINE_ISR( usart1_irq )
{
    platform_uart_irq( &platform_uart_drivers[WICED_UART_1] );
}

WWD_RTOS_DEFINE_ISR( usart2_irq )
{
    platform_uart_irq( &platform_uart_drivers[WICED_UART_8] );
}

WWD_RTOS_DEFINE_ISR( usart3_irq )
{
    platform_uart_irq( &platform_uart_drivers[WICED_UART_3] );
}

WWD_RTOS_DEFINE_ISR( uart4_irq )
{
    platform_uart_irq( &platform_uart_drivers[WICED_UART_4] );
}

WWD_RTOS_DEFINE_ISR( uart5_irq )
{
    platform_uart_irq( &platform_uart_drivers[WICED_UART_5] );
}

WWD_RTOS_DEFINE_ISR( usart6_irq )
{
    platform_uart_irq( &platform_uart_drivers[WICED_UART_6] );
}

WWD_RTOS_DEFINE_ISR( uart7_irq )
{
    platform_uart_irq( &platform_uart_drivers[WICED_UART_7] );
}

WWD_RTOS_DEFINE_ISR( uart8_irq )
{
    platform_uart_irq( &platform_uart_drivers[WICED_UART_2] );
}

WWD_RTOS_DEFINE_ISR( gpio_irq )
{
    platform_gpio_irq();
}

/******************************************************
 *            Interrupt Handlers Mapping
 ******************************************************/

WWD_RTOS_MAP_ISR( usart1_irq       , USART1_IRQHandler       )
WWD_RTOS_MAP_ISR( usart2_irq       , USART2_IRQHandler       )
WWD_RTOS_MAP_ISR( usart3_irq       , USART3_IRQHandler       )
WWD_RTOS_MAP_ISR( uart4_irq        , UART4_IRQHandler        )
WWD_RTOS_MAP_ISR( uart5_irq        , UART5_IRQHandler        )
WWD_RTOS_MAP_ISR( usart6_irq       , USART6_IRQHandler       )
WWD_RTOS_MAP_ISR( uart7_irq        , UART7_IRQHandler        )
WWD_RTOS_MAP_ISR( uart8_irq        , UART8_IRQHandler        )
WWD_RTOS_MAP_ISR( gpio_irq         , EXTI0_IRQHandler        )
WWD_RTOS_MAP_ISR( gpio_irq         , EXTI1_IRQHandler        )
WWD_RTOS_MAP_ISR( gpio_irq         , EXTI2_IRQHandler        )
WWD_RTOS_MAP_ISR( gpio_irq         , EXTI3_IRQHandler        )
WWD_RTOS_MAP_ISR( gpio_irq         , EXTI4_IRQHandler        )
WWD_RTOS_MAP_ISR( gpio_irq         , EXTI9_5_IRQHandler      )
WWD_RTOS_MAP_ISR( gpio_irq         , EXTI15_10_IRQHandler    )
