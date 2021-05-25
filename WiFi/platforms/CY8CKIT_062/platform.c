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
 * Defines board support package for CY8CKIT_062 board
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
#include "CapSense_Configuration.h"
#include "gpio_button.h"
#include "platform_capsense.h"


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
    [WICED_GPIO_1]  = { .port_num = 0,  .pin_num = (0U), .hsiom = P0_0_GPIO},
    [WICED_GPIO_2]  = { .port_num = 0,  .pin_num = (1U), .hsiom = P0_1_GPIO},
    [WICED_GPIO_3]  = { .port_num = 0,  .pin_num = (2U), .hsiom = P0_2_GPIO},
    [WICED_GPIO_4]  = { .port_num = 0,  .pin_num = (3U), .hsiom = P0_3_GPIO},  /* RGB LED Red light control */
    [WICED_GPIO_5]  = { .port_num = 0,  .pin_num = (4U), .hsiom = P0_4_GPIO},
    [WICED_GPIO_6]  = { .port_num = 0,  .pin_num = (5U), .hsiom = P0_5_GPIO},
    [WICED_GPIO_7]  = { .port_num = 1,  .pin_num = (0U), .hsiom = P1_0_GPIO},
    [WICED_GPIO_8]  = { .port_num = 1,  .pin_num = (1U), .hsiom = P1_1_GPIO},  /* RGB LED Green light control */
    [WICED_GPIO_9]  = { .port_num = 1,  .pin_num = (2U), .hsiom = P1_2_GPIO},
    [WICED_GPIO_10] = { .port_num = 1,  .pin_num = (3U), .hsiom = P1_3_GPIO},
    [WICED_GPIO_11] = { .port_num = 1,  .pin_num = (4U), .hsiom = P1_4_GPIO},
    [WICED_GPIO_12] = { .port_num = 1,  .pin_num = (5U), .hsiom = P1_5_GPIO},
    [WICED_GPIO_13] = { .port_num = 2,  .pin_num = (0U), .hsiom = P2_0_GPIO},
    [WICED_GPIO_14] = { .port_num = 2,  .pin_num = (1U), .hsiom = P2_1_GPIO},
    [WICED_GPIO_15] = { .port_num = 2,  .pin_num = (2U), .hsiom = P2_2_GPIO},
    [WICED_GPIO_16] = { .port_num = 2,  .pin_num = (3U), .hsiom = P2_3_GPIO},
    [WICED_GPIO_17] = { .port_num = 2,  .pin_num = (4U), .hsiom = P2_4_GPIO},
    [WICED_GPIO_18] = { .port_num = 2,  .pin_num = (5U), .hsiom = P2_5_GPIO},
    [WICED_GPIO_19] = { .port_num = 2,  .pin_num = (6U), .hsiom = P2_6_GPIO},
    [WICED_GPIO_20] = { .port_num = 2,  .pin_num = (7U), .hsiom = P2_7_GPIO},
    [WICED_GPIO_21] = { .port_num = 3,  .pin_num = (0U), .hsiom = P3_0_SCB2_UART_RX},
    [WICED_GPIO_22] = { .port_num = 3,  .pin_num = (1U), .hsiom = P3_1_SCB2_UART_TX},
    [WICED_GPIO_23] = { .port_num = 3,  .pin_num = (2U), .hsiom = P3_2_SCB2_UART_RTS},
    [WICED_GPIO_24] = { .port_num = 3,  .pin_num = (3U), .hsiom = P3_3_SCB2_UART_CTS},
    [WICED_GPIO_25] = { .port_num = 3,  .pin_num = (4U), .hsiom = P3_4_GPIO},
    [WICED_GPIO_26] = { .port_num = 3,  .pin_num = (5U), .hsiom = P3_5_GPIO},
    [WICED_GPIO_27] = { .port_num = 4,  .pin_num = (0U), .hsiom = P4_0_GPIO},
    [WICED_GPIO_28] = { .port_num = 4,  .pin_num = (1U), .hsiom = P4_1_GPIO},
#ifdef WICED_USE_AUDIO
    [WICED_GPIO_29] = { .port_num = 5,  .pin_num = (0U), .hsiom = P5_0_GPIO},
    [WICED_GPIO_30] = { .port_num = 5,  .pin_num = (1U), .hsiom = P5_1_AUDIOSS_TX_SCK},
    [WICED_GPIO_31] = { .port_num = 5,  .pin_num = (2U), .hsiom = P5_2_AUDIOSS_TX_WS},
    [WICED_GPIO_32] = { .port_num = 5,  .pin_num = (3U), .hsiom = P5_3_AUDIOSS_TX_SDO},
    [WICED_GPIO_33] = { .port_num = 5,  .pin_num = (4U), .hsiom = P5_4_AUDIOSS_RX_SCK},
    [WICED_GPIO_34] = { .port_num = 5,  .pin_num = (5U), .hsiom = P5_5_AUDIOSS_RX_WS},
    [WICED_GPIO_35] = { .port_num = 5,  .pin_num = (6U), .hsiom = P5_6_AUDIOSS_RX_SDI},
#else
    [WICED_GPIO_29] = { .port_num = 5,  .pin_num = (0U), .hsiom = P5_0_SCB5_UART_RX},
    [WICED_GPIO_30] = { .port_num = 5,  .pin_num = (1U), .hsiom = P5_1_SCB5_UART_TX},
    [WICED_GPIO_31] = { .port_num = 5,  .pin_num = (2U), .hsiom = P5_2_GPIO},
    [WICED_GPIO_32] = { .port_num = 5,  .pin_num = (3U), .hsiom = P5_3_GPIO},
    [WICED_GPIO_33] = { .port_num = 5,  .pin_num = (4U), .hsiom = P5_4_GPIO},
    [WICED_GPIO_34] = { .port_num = 5,  .pin_num = (5U), .hsiom = P5_5_GPIO},
    [WICED_GPIO_35] = { .port_num = 5,  .pin_num = (6U), .hsiom = P5_6_GPIO},
#endif
    [WICED_GPIO_36] = { .port_num = 5,  .pin_num = (7U), .hsiom = P5_7_GPIO},
    [WICED_GPIO_37] = { .port_num = 6,  .pin_num = (0U), .hsiom = P6_0_SCB3_I2C_SCL},
    [WICED_GPIO_38] = { .port_num = 6,  .pin_num = (1U), .hsiom = P6_1_SCB3_I2C_SDA},
    [WICED_GPIO_39] = { .port_num = 6,  .pin_num = (2U), .hsiom = P6_2_GPIO},
    [WICED_GPIO_40] = { .port_num = 6,  .pin_num = (3U), .hsiom = P6_3_GPIO},
    [WICED_GPIO_41] = { .port_num = 6,  .pin_num = (4U), .hsiom = P6_4_GPIO},
    [WICED_GPIO_42] = { .port_num = 6,  .pin_num = (5U), .hsiom = P6_5_GPIO},
    [WICED_GPIO_43] = { .port_num = 6,  .pin_num = (6U), .hsiom = P6_6_GPIO},
    [WICED_GPIO_44] = { .port_num = 6,  .pin_num = (7U), .hsiom = P6_7_GPIO},
    [WICED_GPIO_45] = { .port_num = 7,  .pin_num = (0U), .hsiom = P7_0_GPIO},
    [WICED_GPIO_46] = { .port_num = 7,  .pin_num = (1U), .hsiom = P7_1_GPIO},
    [WICED_GPIO_47] = { .port_num = 7,  .pin_num = (2U), .hsiom = P7_2_GPIO},
    [WICED_GPIO_48] = { .port_num = 7,  .pin_num = (3U), .hsiom = P7_3_GPIO},
    [WICED_GPIO_49] = { .port_num = 7,  .pin_num = (4U), .hsiom = P7_4_GPIO},
    [WICED_GPIO_50] = { .port_num = 7,  .pin_num = (5U), .hsiom = P7_5_GPIO},
    [WICED_GPIO_51] = { .port_num = 7,  .pin_num = (6U), .hsiom = P7_6_GPIO},
    [WICED_GPIO_52] = { .port_num = 7,  .pin_num = (7U), .hsiom = P7_7_GPIO},
    [WICED_GPIO_53] = { .port_num = 8,  .pin_num = (0U), .hsiom = P8_0_GPIO},
    [WICED_GPIO_54] = { .port_num = 8,  .pin_num = (1U), .hsiom = P8_1_GPIO},
    [WICED_GPIO_55] = { .port_num = 8,  .pin_num = (2U), .hsiom = P8_2_GPIO},
    [WICED_GPIO_56] = { .port_num = 8,  .pin_num = (3U), .hsiom = P8_3_GPIO},
    [WICED_GPIO_57] = { .port_num = 8,  .pin_num = (4U), .hsiom = P8_4_GPIO},
    [WICED_GPIO_58] = { .port_num = 8,  .pin_num = (5U), .hsiom = P8_5_GPIO},
    [WICED_GPIO_59] = { .port_num = 8,  .pin_num = (6U), .hsiom = P8_6_GPIO},
    [WICED_GPIO_60] = { .port_num = 8,  .pin_num = (7U), .hsiom = P8_7_GPIO},
    [WICED_GPIO_61] = { .port_num = 9,  .pin_num = (0U), .hsiom = P9_0_GPIO},
    [WICED_GPIO_62] = { .port_num = 9,  .pin_num = (1U), .hsiom = P9_1_GPIO},
    [WICED_GPIO_63] = { .port_num = 9,  .pin_num = (2U), .hsiom = P9_2_GPIO},
    [WICED_GPIO_64] = { .port_num = 9,  .pin_num = (3U), .hsiom = P9_3_GPIO},
    [WICED_GPIO_65] = { .port_num = 9,  .pin_num = (4U), .hsiom = P9_4_GPIO},
    [WICED_GPIO_66] = { .port_num = 9,  .pin_num = (5U), .hsiom = P9_5_GPIO},
    [WICED_GPIO_67] = { .port_num = 9,  .pin_num = (6U), .hsiom = P9_6_GPIO},
    [WICED_GPIO_68] = { .port_num = 9,  .pin_num = (7U), .hsiom = P9_7_GPIO},
    [WICED_GPIO_69] = { .port_num = 10, .pin_num = (0U), .hsiom = P10_0_GPIO},
    [WICED_GPIO_70] = { .port_num = 10, .pin_num = (1U), .hsiom = P10_1_GPIO},
    [WICED_GPIO_71] = { .port_num = 10, .pin_num = (2U), .hsiom = P10_2_GPIO},
    [WICED_GPIO_72] = { .port_num = 10, .pin_num = (3U), .hsiom = P10_3_GPIO},
    [WICED_GPIO_73] = { .port_num = 10, .pin_num = (4U), .hsiom = P10_4_AUDIOSS_PDM_CLK},
    [WICED_GPIO_74] = { .port_num = 10, .pin_num = (5U), .hsiom = P10_5_AUDIOSS_PDM_DATA},
    [WICED_GPIO_75] = { .port_num = 10, .pin_num = (6U), .hsiom = P10_6_GPIO},
    [WICED_GPIO_76] = { .port_num = 10, .pin_num = (7U), .hsiom = P10_7_GPIO},
    [WICED_GPIO_77] = { .port_num = 11, .pin_num = (0U), .hsiom = P11_0_GPIO},
    [WICED_GPIO_78] = { .port_num = 11, .pin_num = (1U), .hsiom = P11_1_GPIO}, /* RGB LED Blue light control */
    [WICED_GPIO_79] = { .port_num = 11, .pin_num = (2U), .hsiom = P11_2_SMIF_SPI_SELECT0}, /* SMIF Select0 - External SPI Flash */
    [WICED_GPIO_80] = { .port_num = 11, .pin_num = (3U), .hsiom = P11_3_SMIF_SPI_DATA3},   /* SMIF Data3 - External SPI Flash */
    [WICED_GPIO_81] = { .port_num = 11, .pin_num = (4U), .hsiom = P11_4_SMIF_SPI_DATA2},   /* SMIF Data2 - External SPI Flash */
    [WICED_GPIO_82] = { .port_num = 11, .pin_num = (5U), .hsiom = P11_5_SMIF_SPI_DATA1},   /* SMIF Data1 - External SPI Flash */
    [WICED_GPIO_83] = { .port_num = 11, .pin_num = (6U), .hsiom = P11_6_SMIF_SPI_DATA0},   /* SMIF Data0 - External SPI Flash */
    [WICED_GPIO_84] = { .port_num = 11, .pin_num = (7U), .hsiom = P11_7_SMIF_SPI_CLK},     /* SMIF CLK   - External SPI Flash */
    [WICED_GPIO_85] = { .port_num = 12, .pin_num = (0U), .hsiom = P12_0_SCB6_SPI_MOSI},
    [WICED_GPIO_86] = { .port_num = 12, .pin_num = (1U), .hsiom = P12_1_SCB6_SPI_MISO},
    [WICED_GPIO_87] = { .port_num = 12, .pin_num = (2U), .hsiom = P12_2_SCB6_SPI_CLK},
    [WICED_GPIO_88] = { .port_num = 12, .pin_num = (3U), .hsiom = P12_3_SCB6_SPI_SELECT0},
    [WICED_GPIO_89] = { .port_num = 12, .pin_num = (4U), .hsiom = P12_4_GPIO},
    [WICED_GPIO_90] = { .port_num = 12, .pin_num = (5U), .hsiom = P12_5_GPIO},
    [WICED_GPIO_91] = { .port_num = 12, .pin_num = (6U), .hsiom = P12_6_GPIO},
    [WICED_GPIO_92] = { .port_num = 12, .pin_num = (7U), .hsiom = P12_7_GPIO},
    [WICED_GPIO_93] = { .port_num = 13, .pin_num = (0U), .hsiom = P13_0_GPIO},
    [WICED_GPIO_94] = { .port_num = 13, .pin_num = (1U), .hsiom = P13_1_GPIO},
    [WICED_GPIO_95] = { .port_num = 13, .pin_num = (2U), .hsiom = P13_2_GPIO},
    [WICED_GPIO_96] = { .port_num = 13, .pin_num = (3U), .hsiom = P13_3_GPIO},
    [WICED_GPIO_97] = { .port_num = 13, .pin_num = (4U), .hsiom = P13_4_GPIO},
    [WICED_GPIO_98] = { .port_num = 13, .pin_num = (5U), .hsiom = P13_5_GPIO},
    [WICED_GPIO_99] = { .port_num = 13, .pin_num = (6U), .hsiom = P13_6_GPIO},
    [WICED_GPIO_100]= { .port_num = 13, .pin_num = (7U), .hsiom = P13_7_GPIO},
};

/* ADC peripherals. Used WICED/platform/MCU/wiced_platform_common.c */
const platform_adc_t platform_adc_peripherals[] =
{
    [WICED_ADC_1] =
    {
        .channel         = 0,
        .enabled         = 1,
        .input_mode      = CY_SAR_CHAN_SINGLE_ENDED,
        .vplus_port_addr = CY_SAR_POS_PORT_ADDR_SARMUX,
        .vplus_pin_addr  = CY_SAR_CHAN_POS_PIN_ADDR_0,
        .vplus_pin       = &platform_gpio_pins[WICED_GPIO_69],
    },
    [WICED_ADC_2] =
    {
        .channel         = 1,
        .enabled         = 0,
    },
    [WICED_ADC_3] =
    {
        .channel         = 2,
        .enabled         = 0,
    },
    [WICED_ADC_4] =
    {
        .channel         = 3,
        .enabled         = 0,
    },
    [WICED_ADC_5] =
    {
        .channel         = 4,
        .enabled         = 0,
    },
    [WICED_ADC_6] =
    {
        .channel         = 5,
        .enabled         = 0,
    },
    [WICED_ADC_7] =
    {
        .channel         = 6,
        .enabled         = 0,
    },
    [WICED_ADC_8] =
    {
        .channel         = 7,
        .enabled         = 0,
    },
    [WICED_ADC_9] =
    {
        .channel         = 8,
        .enabled         = 0,
    },
    [WICED_ADC_10] =
    {
        .channel         = 9,
        .enabled         = 0,
    },
    [WICED_ADC_11] =
    {
        .channel         = 10,
        .enabled         = 0,
    },
    [WICED_ADC_12] =
    {
        .channel         = 11,
        .enabled         = 0,
    },
    [WICED_ADC_13] =
    {
        .channel         = 12,
        .enabled         = 0,
    },
    [WICED_ADC_14] =
    {
        .channel         = 13,
        .enabled         = 0,
    },
    [WICED_ADC_15] =
    {
        .channel         = 14,
        .enabled         = 0,
    },
    [WICED_ADC_16] =
    {
        .channel         = 15,
        .enabled         = 0,
    },
};

/* Peripheral Clock Divider Assignments */

/* 8-bit */
DEFINE_PLATFORM_CLOCK_DIV_8( PCLK_UDB_CLOCKS0, 0 );
DEFINE_PLATFORM_CLOCK_DIV_8( PCLK_PASS_CLOCK_SAR, 1 );
DEFINE_PLATFORM_CLOCK_DIV_8( PCLK_CSD_CLOCK, 2 );

/* 16-bit */
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_SCB0_CLOCK, 0 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_SCB1_CLOCK, 1 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_SCB2_CLOCK, 2 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_SCB3_CLOCK, 3 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_SCB4_CLOCK, 4 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_SCB5_CLOCK, 5 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_SCB6_CLOCK, 6 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_SCB7_CLOCK, 7 );

/* PWM Clock Source assignments - Currently same clock source for All PWM blocks */
/* const platform_peripheral_clock_t pwm_clk = 16-bit */
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM0_CLOCKS0,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM0_CLOCKS1,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM0_CLOCKS2,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM0_CLOCKS3,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM0_CLOCKS4,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM0_CLOCKS5,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM0_CLOCKS6,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM0_CLOCKS7,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS0,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS1,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS2,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS3,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS4,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS5,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS6,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS7,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS8,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS9,  14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS10, 14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS11, 14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS12, 14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS13, 14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS14, 14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS15, 14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS16, 14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS17, 14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS18, 14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS19, 14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS20, 14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS21, 14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS22, 14 );
DEFINE_PLATFORM_CLOCK_DIV_16( PCLK_TCPWM1_CLOCKS23, 14 );

#define PWM_PIN_ELEM( _block_, _cntr_, _port_, _pin_ ) \
       .hsiom = P ## _port_ ## _ ## _pin_ ## _TCPWM ## _block_ ## _LINE ## _cntr_

#define PWM_PIN_ELEM_COMPL( _block_, _cntr_, _port_, _pin_ ) \
       .hsiom = P ## _port_ ## _ ## _pin_ ## _TCPWM ## _block_ ## _LINE_COMPL ## _cntr_


#define PLATFORM_PWM( _block_, _cntr_, _wiced_gpio_, _port_, _pin_ ) \
    { .block = _block_, \
      .cntnum = _cntr_, \
      .pclk = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM ## _block_ ## _CLOCKS ## _cntr_), \
      .out_pin = &platform_gpio_pins[_wiced_gpio_], \
      .hsiom = P ## _port_ ## _ ## _pin_ ## _TCPWM ## _block_ ## _LINE ## _cntr_, \
    }

#define PLATFORM_PWM_COMPL( _block_, _cntr_, _wiced_gpio_, _wiced_gpio_port_, _wiced_gpio_pin_ ) \
    { .block = _block_, \
      .cntnum = _cntr_, \
      .pclk = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM ## _block_ ## _CLOCKS ## _cntr_), \
      .out_pin = &platform_gpio_pins[_wiced_gpio_], \
      PWM_PIN_ELEM_COMPL(_block_, _cntr_, _wiced_gpio_port_, _wiced_gpio_pin_) \
    }

/* PWM peripherals. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_pwm_t platform_pwm_peripherals[] =
{
    [WICED_PWM_1]  = PLATFORM_PWM_COMPL(0, 1, WICED_GPIO_4, 0, 3),
    [WICED_PWM_2]  = PLATFORM_PWM_COMPL(0, 3, WICED_GPIO_8, 1, 1),
    [WICED_PWM_3]  = PLATFORM_PWM_COMPL(1, 1, WICED_GPIO_78, 11, 1),
    [WICED_PWM_4]  =
    {
        .block     = 0,
        .cntnum    = 0,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM0_CLOCKS0 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_5]  =
    {
        .block     = 0,
        .cntnum    = 2,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM0_CLOCKS2 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_6]  =
    {
        .block     = 0,
        .cntnum    = 4,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM0_CLOCKS4 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_7]  =
    {
        .block     = 0,
        .cntnum    = 5,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM0_CLOCKS5 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_8]  =
    {
        .block     = 0,
        .cntnum    = 6,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM0_CLOCKS6 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_9]  =
    {
        .block     = 0,
        .cntnum    = 7,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM0_CLOCKS7 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_10]  =
    {
        .block     = 1,
        .cntnum    = 0,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS0 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_11]  =
    {
        .block     = 1,
        .cntnum    = 2,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS2 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_12]  =
    {
        .block     = 1,
        .cntnum    = 3,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS3 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_13]  =
    {
        .block     = 1,
        .cntnum    = 4,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS4 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_14]  =
    {
        .block     = 1,
        .cntnum    = 5,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS5 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_15]  =
    {
        .block     = 1,
        .cntnum    = 6,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS6 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_16]  =
    {
        .block     = 1,
        .cntnum    = 7,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS7 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_17]  =
    {
        .block     = 1,
        .cntnum    = 8,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS8 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_18]  =
    {
        .block     = 1,
        .cntnum    = 9,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS9 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_19]  =
    {
        .block     = 1,
        .cntnum    = 10,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS10 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_20]  =
    {
        .block     = 1,
        .cntnum    = 11,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS11 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_21]  =
    {
        .block     = 1,
        .cntnum    = 12,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS12 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_22]  =
    {
        .block     = 1,
        .cntnum    = 13,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS13 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_23]  =
    {
        .block     = 1,
        .cntnum    = 14,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS14 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_24]  =
    {
        .block     = 1,
        .cntnum    = 15,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS15 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_25]  =
    {
        .block     = 1,
        .cntnum    = 16,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS16 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_26]  =
    {
        .block     = 1,
        .cntnum    = 17,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS17 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_27]  =
    {
        .block     = 1,
        .cntnum    = 18,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS18 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_28]  =
    {
        .block     = 1,
        .cntnum    = 19,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS19 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_29]  =
    {
        .block     = 1,
        .cntnum    = 20,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS20 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_30]  =
    {
        .block     = 1,
        .cntnum    = 21,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS21 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_31]  =
    {
        .block     = 1,
        .cntnum    = 22,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS22 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    },
    [WICED_PWM_32]  =
    {
        .block     = 1,
        .cntnum    = 23,
        .pclk      = PLATFORM_CLOCK_DIV_PTR( PCLK_TCPWM1_CLOCKS23 ),
        .out_pin   = NULL,
        .hsiom     = 0,
    }
};

/* SCB port assignments. */
const platform_scb_t platform_scb0 = { .scb_base = SCB0, .pclk = PLATFORM_CLOCK_DIV_PTR( PCLK_SCB0_CLOCK ), .irq_num = scb_0_interrupt_IRQn };
const platform_scb_t platform_scb1 = { .scb_base = SCB1, .pclk = PLATFORM_CLOCK_DIV_PTR( PCLK_SCB1_CLOCK ), .irq_num = scb_1_interrupt_IRQn };
const platform_scb_t platform_scb2 = { .scb_base = SCB2, .pclk = PLATFORM_CLOCK_DIV_PTR( PCLK_SCB2_CLOCK ), .irq_num = scb_2_interrupt_IRQn  };
const platform_scb_t platform_scb3 = { .scb_base = SCB3, .pclk = PLATFORM_CLOCK_DIV_PTR( PCLK_SCB3_CLOCK ), .irq_num = scb_3_interrupt_IRQn  };
const platform_scb_t platform_scb4 = { .scb_base = SCB4, .pclk = PLATFORM_CLOCK_DIV_PTR( PCLK_SCB4_CLOCK ), .irq_num = scb_4_interrupt_IRQn  };
const platform_scb_t platform_scb5 = { .scb_base = SCB5, .pclk = PLATFORM_CLOCK_DIV_PTR( PCLK_SCB5_CLOCK ), .irq_num = scb_5_interrupt_IRQn  };
const platform_scb_t platform_scb6 = { .scb_base = SCB6, .pclk = PLATFORM_CLOCK_DIV_PTR( PCLK_SCB6_CLOCK ), .irq_num = scb_6_interrupt_IRQn  };
const platform_scb_t platform_scb7 = { .scb_base = SCB7, .pclk = PLATFORM_CLOCK_DIV_PTR( PCLK_SCB7_CLOCK ), .irq_num = scb_7_interrupt_IRQn  };

/* SPI peripherals. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_spi_t platform_spi_peripherals[] =
{
    [WICED_SPI_1]  =
    {
        .port      = &platform_scb0,           /* SCB 0 SPI port */
        .ssel      = CY_SCB_SPI_SLAVE_SELECT0, /* SPI slave select */
        .mosi_pin  = NULL,                     /* SPI MOSI pin */
        .miso_pin  = NULL,                     /* SPI MISO pin */
        .clock_pin = NULL,                     /* SPI Clock pin */
        .cs_pin    = NULL,                     /* SPI CS pin */
    },
    [WICED_SPI_2]  =
    {
        .port      = &platform_scb1,           /* SCB 1 SPI */
        .ssel      = CY_SCB_SPI_SLAVE_SELECT0, /* SPI slave select */
        .mosi_pin  = NULL,                     /* SPI MOSI pin */
        .miso_pin  = NULL,                     /* SPI MISO pin */
        .clock_pin = NULL,                     /* SPI Clock pin */
        .cs_pin    = NULL,                     /* SPI CS pin */
    },
    [WICED_SPI_3]  =
    {
        .port      = &platform_scb2,           /* SCB 2 SPI */
        .ssel      = CY_SCB_SPI_SLAVE_SELECT0, /* SPI slave select */
        .mosi_pin  = NULL,                     /* SPI MOSI pin */
        .miso_pin  = NULL,                     /* SPI MISO pin */
        .clock_pin = NULL,                     /* SPI Clock pin */
        .cs_pin    = NULL,                     /* SPI CS pin */
    },
    [WICED_SPI_4]  =
    {
        .port      = &platform_scb3,           /* SCB 3 SPI */
        .ssel      = CY_SCB_SPI_SLAVE_SELECT0, /* SPI slave select */
        .mosi_pin  = NULL,                     /* SPI MOSI pin */
        .miso_pin  = NULL,                     /* SPI MISO pin */
        .clock_pin = NULL,                     /* SPI Clock pin */
        .cs_pin    = NULL,                     /* SPI CS pin */
    },
    [WICED_SPI_5]  =
    {
        .port      = &platform_scb4,           /* SCB 4 SPI */
        .ssel      = CY_SCB_SPI_SLAVE_SELECT0, /* SPI slave select */
        .mosi_pin  = NULL,                     /* SPI MOSI pin */
        .miso_pin  = NULL,                     /* SPI MISO pin */
        .clock_pin = NULL,                     /* SPI Clock pin */
        .cs_pin    = NULL,                     /* SPI CS pin */
    },
    [WICED_SPI_6]  =
    {
        .port      = &platform_scb5,           /* SCB 5 SPI */
        .ssel      = CY_SCB_SPI_SLAVE_SELECT0, /* SPI slave select */
        .mosi_pin  = NULL,                     /* SPI MOSI pin */
        .miso_pin  = NULL,                     /* SPI MISO pin */
        .clock_pin = NULL,                     /* SPI Clock pin */
        .cs_pin    = NULL,                     /* SPI CS pin */
    },
    [WICED_SPI_7]  =
    {
        .port      = &platform_scb6,                        /* SCB 6 SPI */
        .ssel      = CY_SCB_SPI_SLAVE_SELECT0,              /* SPI slave select */
        .mosi_pin  = &platform_gpio_pins[WICED_GPIO_85],    /* SPI MOSI pin */
        .miso_pin  = &platform_gpio_pins[WICED_GPIO_86],    /* SPI MISO pin */
        .clock_pin = &platform_gpio_pins[WICED_GPIO_87],    /* SPI Clock pin */
        .cs_pin    = &platform_gpio_pins[WICED_GPIO_88],    /* SPI CS pin */
    },
    [WICED_SPI_8]  =
    {
        .port      = &platform_scb7,           /* SCB 7 SPI */
        .ssel      = CY_SCB_SPI_SLAVE_SELECT0, /* SPI slave select */
        .mosi_pin  = NULL,                     /* SPI MOSI pin */
        .miso_pin  = NULL,                     /* SPI MISO pin */
        .clock_pin = NULL,                     /* SPI Clock pin */
        .cs_pin    = NULL,                     /* SPI CS pin */
    },
};

/* I2C peripherals. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_i2c_t platform_i2c_peripherals[] =
{
    [WICED_I2C_1] =
    {
        .port     = 0,
        .scb      = &platform_scb0,     /* SCB 0 I2C */
        .scl_pin  = NULL,               /* I2C SCL pin */
        .sda_pin  = NULL,               /* I2C SDA pin */
    },
    [WICED_I2C_2] =
    {
        .port     = 1,
        .scb      = &platform_scb1,     /* SCB 1 I2C */
        .scl_pin  = NULL,               /* I2C SCL pin */
        .sda_pin  = NULL,               /* I2C SDA pin */
    },
    [WICED_I2C_3] =
    {
        .port     = 2,
        .scb      = &platform_scb2,     /* SCB 2 I2C */
        .scl_pin  = NULL,               /* I2C SCL pin */
        .sda_pin  = NULL,               /* I2C SDA pin */
    },
    [WICED_I2C_4] =
    {
        .port     = 3,
        .scb      = &platform_scb3,                        /* SCB 3 I2C */
        .scl_pin  = &platform_gpio_pins[WICED_GPIO_37],    /* I2C SCL pin */
        .sda_pin  = &platform_gpio_pins[WICED_GPIO_38],    /* I2C SDA pin */
    },
    [WICED_I2C_5] =
    {
        .port     = 4,
        .scb      = &platform_scb4,     /* SCB 4 I2C */
        .scl_pin  = NULL,               /* I2C SCL pin */
        .sda_pin  = NULL,               /* I2C SDA pin */
    },
    [WICED_I2C_6] =
    {
        .port     = 5,
        .scb      = &platform_scb5,     /* SCB 5 I2C */
        .scl_pin  = NULL,               /* I2C SCL pin */
        .sda_pin  = NULL,               /* I2C SDA pin */
    },
    [WICED_I2C_7] =
    {
        .port     = 6,
        .scb      = &platform_scb6,     /* SCB 6 I2C */
        .scl_pin  = NULL,               /* I2C SCL pin */
        .sda_pin  = NULL,               /* I2C SDA pin */
    },
    [WICED_I2C_8] =
    {
        .port     = 7,
        .scb      = &platform_scb7,     /* SCB 7 I2C */
        .scl_pin  = NULL,               /* I2C SCL pin */
        .sda_pin  = NULL,               /* I2C SDA pin */
    },
};

const wiced_i2c_device_t auth_chip_i2c_device =
{

};

const platform_mfi_auth_chip_t platform_auth_chip =
{

};

const platform_i2s_port_info_t i2s_port_info =
{
    .external_clock     = 0,
    .tx_master_mode     = 0,
    .rx_master_mode     = 1,
    .clk_i2s_if_pin     = NULL,
    .mclk_pin           = NULL,
#ifdef WICED_USE_AUDIO
    .tx_sck_pin         = &platform_gpio_pins[WICED_GPIO_30],
    .tx_ws_pin          = &platform_gpio_pins[WICED_GPIO_31],
    .tx_sdo_pin         = &platform_gpio_pins[WICED_GPIO_32],
    .rx_sck_pin         = &platform_gpio_pins[WICED_GPIO_33],
    .rx_ws_pin          = &platform_gpio_pins[WICED_GPIO_34],
    .rx_sdi_pin         = &platform_gpio_pins[WICED_GPIO_35],
#else
    .tx_sck_pin         = NULL,
    .tx_ws_pin          = NULL,
    .tx_sdo_pin         = NULL,
    .rx_sck_pin         = NULL,
    .rx_ws_pin          = NULL,
    .rx_sdi_pin         = NULL,
#endif
};

const platform_i2s_t i2s_interfaces[WICED_I2S_MAX] =
{
    [WICED_I2S_1] =
    {
        .port_info        = &i2s_port_info,
        .stream_direction = PLATFORM_I2S_WRITE,
    },
    [WICED_I2S_2] =
    {
        .port_info        = &i2s_port_info,
        .stream_direction = PLATFORM_I2S_READ,
    },
};

const platform_pdm_pcm_t platform_pdm_pcm_peripheral =
{
    .sample_rate = 44100,
    .output_mode = CY_PDM_PCM_OUT_CHAN_LEFT,
    .clock_pin   = &platform_gpio_pins[WICED_GPIO_73],
    .data_pin    = &platform_gpio_pins[WICED_GPIO_74],
};

/* UART peripherals and runtime drivers. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_uart_t platform_uart_peripherals[] =
{
    [WICED_UART_1] =
    {
        .port    = 0,
        .scb     = &platform_scb0,
        .rx_pin  = NULL, /* UART RX pin */
        .tx_pin  = NULL, /* UART TX pin */
        .cts_pin = NULL, /* UART CTS pin */
        .rts_pin = NULL, /* UART RTS pin */
    },
    [WICED_UART_2] =
    {
        .port    = 1,
        .scb     = &platform_scb1,
        .rx_pin  = NULL, /* UART RX pin */
        .tx_pin  = NULL, /* UART TX pin */
        .cts_pin = NULL, /* UART CTS pin */
        .rts_pin = NULL, /* UART RTS pin */
    },
    [WICED_UART_3] =
    {
        .port    = 2,
        .scb     = &platform_scb2,
        .rx_pin  = &platform_gpio_pins[WICED_GPIO_21], /* UART RX pin */
        .tx_pin  = &platform_gpio_pins[WICED_GPIO_22], /* UART TX pin */
        .cts_pin = &platform_gpio_pins[WICED_GPIO_24], /* UART CTS pin */
        .rts_pin = &platform_gpio_pins[WICED_GPIO_23], /* UART RTS pin */
    },
    [WICED_UART_4] =
    {
        .port    = 3,
        .scb     = &platform_scb3,
        .rx_pin  = NULL, /* UART RX pin */
        .tx_pin  = NULL, /* UART TX pin */
        .cts_pin = NULL, /* UART CTS pin */
        .rts_pin = NULL, /* UART RTS pin */
    },
    [WICED_UART_5] =
    {
        .port    = 4,
        .scb     = &platform_scb4,
        .rx_pin  = NULL, /* UART RX pin */
        .tx_pin  = NULL, /* UART TX pin */
        .cts_pin = NULL, /* UART CTS pin */
        .rts_pin = NULL, /* UART RTS pin */
    },
    [WICED_UART_6] =
    {
        .port    = 5,
        .scb     = &platform_scb5,
#ifdef WICED_USE_AUDIO
        .rx_pin  = NULL,                               /* UART RX pin */
        .tx_pin  = NULL,                               /* UART TX pin */
#else
        .rx_pin  = &platform_gpio_pins[WICED_GPIO_29], /* UART RX pin */
        .tx_pin  = &platform_gpio_pins[WICED_GPIO_30], /* UART TX pin */
#endif
        .cts_pin = NULL,                               /* UART CTS pin */
        .rts_pin = NULL,                               /* UART RTS pin */
    },
    [WICED_UART_7] =
    {
        .port    = 6,
        .scb     = &platform_scb6,
        .rx_pin  = NULL, /* UART RX pin */
        .tx_pin  = NULL, /* UART TX pin */
        .cts_pin = NULL, /* UART CTS pin */
        .rts_pin = NULL, /* UART RTS pin */
    },
    [WICED_UART_8] =
    {
        .port    = 7,
        .scb     = &platform_scb7,
        .rx_pin  = NULL, /* UART RX pin */
        .tx_pin  = NULL, /* UART TX pin */
        .cts_pin = NULL, /* UART CTS pin */
        .rts_pin = NULL, /* UART RTS pin */
    },
};

platform_uart_driver_t platform_uart_drivers[WICED_UART_MAX];

/* SPI flash. Exposed to the applications through include/wiced_platform.h */
/* Default placeholder, to be defined per platform */
#if defined ( WICED_PLATFORM_INCLUDES_SPI_FLASH )
const wiced_spi_device_t wiced_spi_flash =
{
    .port        = WICED_SPI_1,
    .chip_select = WICED_SPI_FLASH_CS,
    .speed       = 5000000,
    .mode        = (SPI_CLOCK_RISING_EDGE | SPI_CLOCK_IDLE_HIGH | SPI_NO_DMA | SPI_MSB_FIRST),
    .bits        = 8
};
#endif

platform_spi_slave_driver_t platform_spi_slave_drivers[WICED_SPI_MAX];

/* UART standard I/O configuration */
/* Default placeholder, to be defined per platform */
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

/* Wi-Fi control pins. Used by WICED/platform/MCU/wwd_platform_common.c
 * SDIO: WWD_PIN_BOOTSTRAP[1:0] = b'00
 * gSPI: WWD_PIN_BOOTSTRAP[1:0] = b'01
 */
/* Default placeholder, to be defined per platform */
const platform_gpio_t wifi_control_pins[] =
{
    [WWD_PIN_RESET      ] = { 0 },
    [WWD_PIN_POWER      ] = { 0 },
    [WWD_PIN_32K_CLK    ] = { 0 },
    [WWD_PIN_BOOTSTRAP_0] = { 0 },
    [WWD_PIN_BOOTSTRAP_1] = { 0 },
};

/* Wi-Fi SDIO bus pins. Used by WICED/platform/MCU/<MCU>/WWD/wwd_SDIO.c */
/* Default placeholder, to be defined per platform */
const platform_gpio_t wifi_sdio_pins[] =
{
    [WWD_PIN_SDIO_OOB_IRQ] = { 0 },
    [WWD_PIN_SDIO_CLK    ] = { 0 },
    [WWD_PIN_SDIO_CMD    ] = { 0 },
    [WWD_PIN_SDIO_D0     ] = { 0 },
    [WWD_PIN_SDIO_D1     ] = { 0 },
    [WWD_PIN_SDIO_D2     ] = { 0 },
    [WWD_PIN_SDIO_D3     ] = { 0 },
};

/* Wi-Fi gSPI bus pins. Used by WICED/platform/MCU/<MCU>/WWD/wwd_SPI.c */
/* Default placeholder, to be defined per platform */
const platform_gpio_t wifi_spi_pins[] =
{
    [WWD_PIN_SPI_IRQ ] = { 0 },
    [WWD_PIN_SPI_CS  ] = { 0 },
    [WWD_PIN_SPI_CLK ] = { 0 },
    [WWD_PIN_SPI_MOSI] = { 0 },
    [WWD_PIN_SPI_MISO] = { 0 },
};

/* Bluetooth control pins. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
const platform_gpio_t* wiced_bt_control_pins[] =
{
    /* Reset pin unavailable */
    [WICED_BT_PIN_POWER      ] = &platform_gpio_pins[WICED_GPIO_25],
    [WICED_BT_PIN_HOST_WAKE  ] = &platform_gpio_pins[WICED_GPIO_26],
    [WICED_BT_PIN_DEVICE_WAKE] = &platform_gpio_pins[WICED_GPIO_27],
    [WICED_BT_PIN_RESET      ] = NULL,
};

/* Bluetooth UART pins. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
const platform_gpio_t* wiced_bt_uart_pins[] =
{
    [WICED_BT_PIN_UART_TX ] = &platform_gpio_pins[WICED_GPIO_22],
    [WICED_BT_PIN_UART_RX ] = &platform_gpio_pins[WICED_GPIO_21],
    [WICED_BT_PIN_UART_CTS] = &platform_gpio_pins[WICED_GPIO_24],
    [WICED_BT_PIN_UART_RTS] = &platform_gpio_pins[WICED_GPIO_23],
};

/* Bluetooth UART peripheral and runtime driver. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
const platform_uart_t*  wiced_bt_uart_peripheral = &platform_uart_peripherals[WICED_UART_3];
platform_uart_driver_t* wiced_bt_uart_driver     = &platform_uart_drivers    [WICED_UART_3];

/* Bluetooth UART configuration. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
const platform_uart_config_t wiced_bt_uart_config =
{
    .baud_rate    = 115200,
    .data_width   = DATA_WIDTH_8BIT,
    .parity       = NO_PARITY,
    .stop_bits    = STOP_BITS_1,
    .flow_control = FLOW_CONTROL_CTS_RTS,
};

/* BT chip specific configuration information */
const platform_bluetooth_config_t wiced_bt_config =
{
    .patchram_download_mode      = PATCHRAM_DOWNLOAD_MODE_MINIDRV_CMD,
    .patchram_download_baud_rate = 115200,
    .featured_baud_rate          = 115200
};

gpio_button_t platform_gpio_buttons[] =
{
    [PLATFORM_BUTTON_1] = { 0 },
};

/* CapSense private widget data. */
static platform_capsense_button_data_t          platform_capsense_button_0_data;
static platform_capsense_button_data_t          platform_capsense_button_1_data;
static platform_capsense_linear_slider_data_t   platform_capsense_linear_slider_0_data;

/* CapSense widget specific configuration (optional). */
static platform_capsense_linear_slider_config_t platform_capsense_linear_slider_config =
{
    .position_notification_threshold    = 5,
};

/* CapSense widget definition.
 * Definitions must agree with types imported from Creator: for example,
 * for CapSense_BUTTON0_WDGT_ID use PLATFORM_CAPSENSE_BUTTON_WIDGET_INIT initializer macro.
 */
const platform_capsense_widget_t platform_capsense_widget[] =
{
    [CapSense_BUTTON0_WDGT_ID] =
            PLATFORM_CAPSENSE_BUTTON_WIDGET_INIT( CapSense_BUTTON0_WDGT_ID, &platform_capsense_button_0_data ),
    [CapSense_BUTTON1_WDGT_ID] =
            PLATFORM_CAPSENSE_BUTTON_WIDGET_INIT( CapSense_BUTTON1_WDGT_ID, &platform_capsense_button_1_data ),
    [CapSense_LINEARSLIDER0_WDGT_ID] =
            PLATFORM_CAPSENSE_LINEAR_SLIDER_WIDGET_INIT( CapSense_LINEARSLIDER0_WDGT_ID, &platform_capsense_linear_slider_0_data, &platform_capsense_linear_slider_config ),
};

/* Configuration for the Simple CapSense platform driver. */
const platform_capsense_simple_driver_config_t platform_capsense_simple_config =
{
    .wait_for_activity_scan_period_in_millisecond   = PLATFORM_CAPSENSE_DEFAULT_TIMEOUT_MS,
    .active_scan_period_in_milliseconds             = PLATFORM_CAPSENSE_DEFAULT_ACTIVE_WIDGET_TIMEOUT_MS,
};

/* Initialize a Simple CapSense interface. */
const platform_capsense_interface_t platform_capsense_interface = PLATFORM_CAPSENSE_SIMPLE_INTERFACE_INIT( &platform_capsense_simple_config );

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
    /* Interrupt priority setup. Called by WICED/platform/MCU/PSoC6/platform_init.c */

    NVIC_SetPriority( udb_interrupts_0_IRQn, 2 );       /* WLAN SDIO */
    NVIC_SetPriority( cpuss_interrupts_dw1_1_IRQn, 3 ); /* WLAN SDIO DMA */
    NVIC_SetPriority( cpuss_interrupts_dw1_3_IRQn, 3 ); /* WLAN SDIO DMA */

    NVIC_SetPriority( csd_interrupt_IRQn,  5 );         /* CSD */
    NVIC_SetPriority( scb_0_interrupt_IRQn, 6 );        /* SCB 0 */
    NVIC_SetPriority( scb_1_interrupt_IRQn, 6 );        /* SCB 1 */
    NVIC_SetPriority( scb_2_interrupt_IRQn, 6 );        /* SCB 2 */
    NVIC_SetPriority( scb_3_interrupt_IRQn, 6 );        /* SCB 3 */
    NVIC_SetPriority( scb_4_interrupt_IRQn, 6 );        /* SCB 4 */
    NVIC_SetPriority( scb_5_interrupt_IRQn, 6 );        /* SCB 5 */
    NVIC_SetPriority( scb_6_interrupt_IRQn, 6 );        /* SCB 6 */
    NVIC_SetPriority( scb_7_interrupt_IRQn, 6 );        /* SCB 7 */
    NVIC_SetPriority( audioss_interrupt_pdm_IRQn, 6 );  /* PDM_PCM */
    NVIC_SetPriority( audioss_interrupt_i2s_IRQn, 6 );  /* I2S */
    NVIC_SetPriority( pass_interrupt_sar_IRQn, 6 );     /* ADC */
    NVIC_SetPriority( smif_interrupt_IRQn, 1 );         /* SMIF */
    NVIC_SetPriority( cpuss_interrupts_ipc_4_IRQn, 1 ); /* IPC4 */
    NVIC_SetPriority( ioss_interrupts_gpio_0_IRQn, 14 );
    NVIC_SetPriority( ioss_interrupts_gpio_1_IRQn, 14 );
    NVIC_SetPriority( ioss_interrupts_gpio_2_IRQn, 14 );
    NVIC_SetPriority( ioss_interrupts_gpio_3_IRQn, 14 );
    NVIC_SetPriority( ioss_interrupts_gpio_4_IRQn, 14 );
    NVIC_SetPriority( ioss_interrupts_gpio_5_IRQn, 14 );
    NVIC_SetPriority( ioss_interrupts_gpio_6_IRQn, 14 );
    NVIC_SetPriority( ioss_interrupts_gpio_7_IRQn, 14 );
    NVIC_SetPriority( ioss_interrupts_gpio_8_IRQn, 14 );
    NVIC_SetPriority( ioss_interrupts_gpio_9_IRQn, 14  );
    NVIC_SetPriority( ioss_interrupts_gpio_10_IRQn, 14 );
    NVIC_SetPriority( ioss_interrupts_gpio_11_IRQn, 14 );
    NVIC_SetPriority( ioss_interrupts_gpio_12_IRQn, 14 );
    NVIC_SetPriority( ioss_interrupts_gpio_13_IRQn, 14 );
    NVIC_SetPriority( ioss_interrupts_gpio_14_IRQn, 14 );
}

/* LEDs on this platform are active LOW */
platform_result_t platform_led_set_state(int led_index, int off_on )
{
    if ((led_index >= 0) && (led_index < PLATFORM_LED_COUNT))
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
        return PLATFORM_SUCCESS;
    }
    return PLATFORM_BADARG;
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

void platform_init_external_devices( void )
{
    /* Initialise LEDs and turn off by default */
    platform_led_init();

    /* Initialise buttons to input by default */
    platform_gpio_init( &platform_gpio_pins[WICED_BUTTON1], INPUT_PULL_UP );

#ifndef WICED_DISABLE_STDIO
    /* Initialise UART standard I/O */
    platform_stdio_init( &platform_uart_drivers[STDIO_UART], &platform_uart_peripherals[STDIO_UART], &stdio_config );
#endif
}

uint32_t  platform_get_button_press_time ( int button_index, int led_index, uint32_t max_time )
{
    UNUSED_PARAMETER(button_index);
    UNUSED_PARAMETER(led_index);
    UNUSED_PARAMETER(max_time);
    return 0;
}

uint32_t  platform_get_factory_reset_button_time ( uint32_t max_time )
{
    UNUSED_PARAMETER(max_time);
    return 0;
}

/******************************************************
 *           Interrupt Handlers
 ******************************************************/

WWD_RTOS_DEFINE_ISR( SCB2_IRQ )
{
    platform_uart_irq( 2 );
}
WWD_RTOS_MAP_ISR( SCB2_IRQ, scb_2_interrupt_IRQn_Handler )

WWD_RTOS_DEFINE_ISR( SCB5_IRQ )
{
    platform_uart_irq( 5 );
}
WWD_RTOS_MAP_ISR( SCB5_IRQ, scb_5_interrupt_IRQn_Handler )

#if !defined( BOOTLOADER )
WWD_RTOS_DEFINE_ISR( CSD_IRQ )
{
    platform_csd_irq();
}
WWD_RTOS_MAP_ISR( CSD_IRQ, csd_interrupt_IRQn_Handler )
#endif /* !defined( BOOTLOADER) */

void platform_smif_irq( void );
WWD_RTOS_DEFINE_ISR( mlg_platform_smif_irq )
{
    platform_smif_irq( );

}

WWD_RTOS_MAP_ISR( mlg_platform_smif_irq , smif_interrupt_IRQn_Handler )

WWD_RTOS_DEFINE_ISR( IPC4_IRQ )
{
   Cy_IPC_SystemPipeIsr();
}

WWD_RTOS_MAP_ISR( IPC4_IRQ, cpuss_interrupts_ipc_4_IRQn_Handler )

WWD_RTOS_DEFINE_ISR( FLASH_FM_IRQ )
{
   Cy_Flash_ResumeIrqHandler();
}

WWD_RTOS_MAP_ISR( FLASH_FM_IRQ, cpuss_interrupt_fm_IRQn_Handler )
