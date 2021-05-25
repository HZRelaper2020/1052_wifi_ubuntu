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
 * platform definition for CYW9MCU7X9N364
 */
#include <stdlib.h>
#include "platform_config.h"
#include "platform.h"
#include "platform_peripheral.h"
#include "platform_mcu_peripheral.h"
#include "platform_bluetooth.h"
#include "wiced_platform.h"
#include "wwd_platform_common.h"
#include "wiced_resource.h"
#include "platform_config.h"
#include "platform_resource.h"
#include "gpio_button.h"
#include "brcm_fw_types.h"
#include "wiced_hal_adc.h"
#include "wiced_bt_dev.h"
#ifdef WICED_FILESYSTEM_SUPPORT
#include "wiced_filesystem.h"
#endif
#include "wiced_hal_puart.h"
#include "wiced_hal_wdog.h"
#include "wiced.h"
/******************************************************
 *                      Macros
 ******************************************************/
/*
 * In order to use traces via JLink/SWD interface USE_JLINK_FOR_TRACES
 * flag has to be enabled from the MCU make file.
 * For more details on how RTT works and how to test the feature a
 * Readme file has been placed under RTT_Logger module.
 * Kinldy refer the same for more details
 *
 */
#ifdef ENABLE_JLINK_TRACE
#define JLINK_DEBUGGER_SUPPORT
#endif
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
 *               Function Declarations
 ******************************************************/
#ifdef WICED_FILESYSTEM_SUPPORT
platform_result_t platform_filesystem_init( void );
#endif
/******************************************************
 *               Variable Definitions
 ******************************************************/
 static platform_gpio_port_t gpio_ports[WICED_GPIO_MAX];
extern const wiced_block_device_driver_t ocf_block_device_driver;

/* Pin mux configuration */
const platform_gpio_t platform_gpio_pins[] =
{
    [WICED_GPIO_0          ]    = { &gpio_ports[WICED_GPIO_0],WICED_GPIO_0,SPI1_MISO_MUX},
    [WICED_GPIO_1          ]    = { &gpio_ports[WICED_GPIO_1],WICED_GPIO_1,0},
    [WICED_GPIO_2          ]    = { &gpio_ports[WICED_GPIO_2],WICED_GPIO_2,SPI1_CS_MUX },
    [WICED_GPIO_3          ]    = { &gpio_ports[WICED_GPIO_3],WICED_GPIO_3,0 },
    [WICED_GPIO_4          ]    = { &gpio_ports[WICED_GPIO_4],WICED_GPIO_4,SPI1_CLK_MUX },
    [WICED_GPIO_5          ]    = { &gpio_ports[WICED_GPIO_5],WICED_GPIO_5,0 },
    [WICED_GPIO_6          ]    = { &gpio_ports[WICED_GPIO_6],WICED_GPIO_6,SPI1_MOSI_MUX},
    [WICED_GPIO_7          ]    = { &gpio_ports[WICED_GPIO_7],WICED_GPIO_7,0 },
    [WICED_GPIO_8          ]    = { &gpio_ports[WICED_GPIO_8],WICED_GPIO_8,0 },
    [WICED_GPIO_9          ]    = { &gpio_ports[WICED_GPIO_9],WICED_GPIO_9,0 },
    [WICED_GPIO_10         ]    = { &gpio_ports[WICED_GPIO_10],WICED_GPIO_10,0 },
    [WICED_GPIO_11         ]    = { &gpio_ports[WICED_GPIO_11],WICED_GPIO_11,0 },
    [WICED_GPIO_12         ]    = { &gpio_ports[WICED_GPIO_12],WICED_GPIO_12,0 },
    [WICED_GPIO_13         ]    = { &gpio_ports[WICED_GPIO_13],WICED_GPIO_13,0 },
    [WICED_GPIO_14         ]    = { &gpio_ports[WICED_GPIO_14],WICED_GPIO_14,0 },
    [WICED_GPIO_15         ]    = { &gpio_ports[WICED_GPIO_15],WICED_GPIO_15,0 },
    [WICED_GPIO_16         ]    = { &gpio_ports[WICED_GPIO_16],WICED_GPIO_16,0 },
    [WICED_GPIO_17         ]    = { &gpio_ports[WICED_GPIO_17],WICED_GPIO_17,0 },
    [WICED_GPIO_18         ]    = { &gpio_ports[WICED_GPIO_18],WICED_GPIO_18,0 },
    [WICED_GPIO_19         ]    = { &gpio_ports[WICED_GPIO_19],WICED_GPIO_19,0 },
    [WICED_GPIO_20         ]    = { &gpio_ports[WICED_GPIO_20],WICED_GPIO_20,0 },
    [WICED_GPIO_21         ]    = { &gpio_ports[WICED_GPIO_21],WICED_GPIO_21,0 },
    [WICED_GPIO_22         ]    = { &gpio_ports[WICED_GPIO_22],WICED_GPIO_22,0 },
    [WICED_GPIO_23         ]    = { &gpio_ports[WICED_GPIO_23],WICED_GPIO_23,0 },
    [WICED_GPIO_24         ]    = { &gpio_ports[WICED_GPIO_24],WICED_GPIO_24,0},
    [WICED_GPIO_25         ]    = { &gpio_ports[WICED_GPIO_25],WICED_GPIO_25,0 },
    [WICED_GPIO_26         ]    = { &gpio_ports[WICED_GPIO_26],WICED_GPIO_26,0 },
    [WICED_GPIO_27         ]    = { &gpio_ports[WICED_GPIO_27],WICED_GPIO_27,0 },
    [WICED_GPIO_28         ]    = { &gpio_ports[WICED_GPIO_28],WICED_GPIO_28,0},
    [WICED_GPIO_29         ]    = { &gpio_ports[WICED_GPIO_29],WICED_GPIO_29,0 },
    [WICED_GPIO_30         ]    = { &gpio_ports[WICED_GPIO_30],WICED_GPIO_30,0 },
    [WICED_GPIO_31         ]    = { &gpio_ports[WICED_GPIO_31],WICED_GPIO_31,0 },
    [WICED_GPIO_32         ]    = { &gpio_ports[WICED_GPIO_32],WICED_GPIO_32,0 },
    [WICED_GPIO_33         ]    = { &gpio_ports[WICED_GPIO_33],WICED_GPIO_33,0 },
    [WICED_GPIO_34         ]    = { &gpio_ports[WICED_GPIO_34],WICED_GPIO_34,0 },
    [WICED_GPIO_35         ]    = { &gpio_ports[WICED_GPIO_35],WICED_GPIO_35,0 },
    [WICED_GPIO_36         ]    = { &gpio_ports[WICED_GPIO_36],WICED_GPIO_36,0 },
    [WICED_GPIO_37         ]    = { &gpio_ports[WICED_GPIO_37],WICED_GPIO_37,0 },
    [WICED_GPIO_38         ]    = { &gpio_ports[WICED_GPIO_38],WICED_GPIO_38,0 },
    [WICED_GPIO_39         ]    = { &gpio_ports[WICED_GPIO_39],WICED_GPIO_39,0 },
};

/* ADC peripherals. Used WICED/platform/MCU/wiced_platform_common.c
 * ADC channel defined here with GPIO mapping as NULL will be considered
 * as invalid, For Ex:
 * [WICED_ADC_1] = {WICED_ADC_1, NULL} makes the ADC ch1 invalid
 * refer to wiced_hal_adc.h file for channel mapping to GPIO pin.
 */
const platform_adc_t platform_adc_peripherals[] =
{
    [WICED_ADC_1] = {ADC_INPUT_P17, &platform_gpio_pins[WICED_GPIO_17]},
    [WICED_ADC_2] = {ADC_INPUT_P16, &platform_gpio_pins[WICED_GPIO_16]},
    [WICED_ADC_3] = {ADC_INPUT_P15, &platform_gpio_pins[WICED_GPIO_15]},
    [WICED_ADC_4] = {ADC_INPUT_P14, &platform_gpio_pins[WICED_GPIO_14]},
    [WICED_ADC_5] = {ADC_INPUT_P13, &platform_gpio_pins[WICED_GPIO_13]},
    [WICED_ADC_6] = {ADC_INPUT_P12, &platform_gpio_pins[WICED_GPIO_12]},
    [WICED_ADC_7] = {ADC_INPUT_P11, &platform_gpio_pins[WICED_GPIO_11]},
    [WICED_ADC_8] = {ADC_INPUT_P10, &platform_gpio_pins[WICED_GPIO_10]},
    [WICED_ADC_9] = {ADC_INPUT_P9, &platform_gpio_pins[WICED_GPIO_9]},
    [WICED_ADC_10] = {ADC_INPUT_P8, &platform_gpio_pins[WICED_GPIO_8]},
    [WICED_ADC_11] = {ADC_INPUT_P1, &platform_gpio_pins[WICED_GPIO_1]},
    [WICED_ADC_12] = {ADC_INPUT_P0, &platform_gpio_pins[WICED_GPIO_0]},
    [WICED_ADC_13] = {ADC_INPUT_P38, &platform_gpio_pins[WICED_GPIO_38]},
    [WICED_ADC_14] = {ADC_INPUT_P37, &platform_gpio_pins[WICED_GPIO_37]},
    [WICED_ADC_15] = {ADC_INPUT_P36, &platform_gpio_pins[WICED_GPIO_36]},
    [WICED_ADC_16] = {ADC_INPUT_P35, &platform_gpio_pins[WICED_GPIO_35]},
    [WICED_ADC_17] = {ADC_INPUT_P34, &platform_gpio_pins[WICED_GPIO_34]},
    [WICED_ADC_18] = {ADC_INPUT_P33, &platform_gpio_pins[WICED_GPIO_33]},
    [WICED_ADC_19] = {ADC_INPUT_P32, &platform_gpio_pins[WICED_GPIO_32]},
    [WICED_ADC_20] = {ADC_INPUT_P31, &platform_gpio_pins[WICED_GPIO_31]},
    [WICED_ADC_21] = {ADC_INPUT_P30, &platform_gpio_pins[WICED_GPIO_30]},
    [WICED_ADC_22] = {ADC_INPUT_P29, &platform_gpio_pins[WICED_GPIO_29]},
    [WICED_ADC_23] = {ADC_INPUT_P28, &platform_gpio_pins[WICED_GPIO_28]},
    [WICED_ADC_24] = {ADC_INPUT_P23, &platform_gpio_pins[WICED_GPIO_23]},
    [WICED_ADC_25] = {ADC_INPUT_P22, &platform_gpio_pins[WICED_GPIO_22]},
    [WICED_ADC_26] = {ADC_INPUT_P21, &platform_gpio_pins[WICED_GPIO_21]},
    [WICED_ADC_27] = {ADC_INPUT_P19, &platform_gpio_pins[WICED_GPIO_19]},
    [WICED_ADC_28] = {ADC_INPUT_P18, &platform_gpio_pins[WICED_GPIO_18]},
};

/* PWM peripherals. Used by WICED/platform/MCU/wiced_platform_common.c
 *
 * */
const platform_pwm_t platform_pwm_peripherals[] =
{
    [WICED_PWM_1]  = {WICED_PWM_1, WICED_ACTIVE_HIGH, &platform_gpio_pins[WICED_GPIO_26]},
    [WICED_PWM_2]  = {WICED_PWM_2, WICED_ACTIVE_HIGH, &platform_gpio_pins[WICED_GPIO_27]},
    [WICED_PWM_3]  = {WICED_PWM_3, WICED_ACTIVE_HIGH, &platform_gpio_pins[WICED_GPIO_28]},
    [WICED_PWM_4]  = {WICED_PWM_4, WICED_ACTIVE_HIGH, &platform_gpio_pins[WICED_GPIO_29]},
};


/* UART peripherals and runtime drivers. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_uart_t platform_uart_peripherals[] =
{
    [WICED_UART_0] = /* Chip PUART */
    {
        .device_id    = WICED_UART_0,
        .rx_pin  = &platform_gpio_pins[WICED_GPIO_34],
        .tx_pin  = &platform_gpio_pins[WICED_GPIO_33],
        .cts_pin = NULL,
        .rts_pin = NULL,
    }
};
platform_uart_driver_t platform_uart_drivers[WICED_UART_MAX];

/* UART standard I/O configuration */
#ifndef WICED_DISABLE_STDIO
static platform_uart_config_t stdio_config =
{
    .baud_rate    = 115200,//921600,
    .data_width   = DATA_WIDTH_8BIT,
    .parity       = NO_PARITY,
    .stop_bits    = STOP_BITS_1,
    .flow_control = FLOW_CONTROL_DISABLED,
};
#endif
/* Bluetooth control pins. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
const platform_gpio_t* wiced_bt_control_pins[] =
{
    [WICED_BT_PIN_POWER] = NULL,
    [WICED_BT_PIN_RESET] = NULL,
    [WICED_BT_PIN_HOST_WAKE] = &platform_gpio_pins[WICED_GPIO_9],
    [WICED_BT_PIN_DEVICE_WAKE] = &platform_gpio_pins[WICED_GPIO_8],
};

platform_uart_driver_t* wiced_stdio_uart_driver     = &platform_uart_drivers[STDIO_UART];

//41 SPI_IRQ, 42 HOST_WAKE 5 WL_REG__ON
const platform_spi_t platform_spi_peripherals[] =
{
    [WICED_SPI_1]  =
    {
        .port                    = SPI1,
        .pin_mosi                = &platform_gpio_pins[WICED_GPIO_6],
        .pin_miso                = &platform_gpio_pins[WICED_GPIO_0],
        .pin_clock               = &platform_gpio_pins[WICED_GPIO_4],
    },
    [WICED_SPI_2]  =
    {
        .port                    = SPI2,
        .pin_mosi                = &platform_gpio_pins[WICED_GPIO_38],
        .pin_miso                = &platform_gpio_pins[WICED_GPIO_37],
        .pin_clock               = &platform_gpio_pins[WICED_GPIO_36],
    }
};

/* wiced SPI loopback device */
const wiced_spi_device_t wiced_spi_loopback_device =
{
    .port        = WICED_SPI_1,
    .chip_select = WICED_GPIO_39,
    .speed       = 24000000, /* 24MHz */
    .mode        = 0, /* not used */
    .bits        = 0 /* not used */
};

/* Wi-Fi gSPI bus pins.*/
const platform_gpio_t* wifi_spi_pins_p[] =
{
    [WWD_PIN_SPI_IRQ] = &platform_gpio_pins[WICED_GPIO_1],
    [WWD_PIN_SPI_CS] = &platform_gpio_pins[WICED_GPIO_2],
    [WWD_PIN_SPI_CLK] = &platform_gpio_pins[WICED_GPIO_4],
    [WWD_PIN_SPI_MOSI] = &platform_gpio_pins[WICED_GPIO_6],
    [WWD_PIN_SPI_MISO] = &platform_gpio_pins[WICED_GPIO_0],
};
platform_spi_slave_driver_t platform_spi_slave_drivers[1];

/* I2C peripherals. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_i2c_t platform_i2c_peripherals[] =
{
    [WICED_I2C_1] =
    {
        .bus_id = WICED_I2C_1,
        .pin_scl = &platform_gpio_pins[WICED_GPIO_19],
        .pin_sda = &platform_gpio_pins[WICED_GPIO_20],
    },
};

gpio_button_t platform_gpio_buttons[] =
{
    [PLATFORM_BUTTON_1] =
    {
        .polarity = WICED_ACTIVE_LOW,
        .gpio = WICED_BUTTON1,
        .trigger = IRQ_TRIGGER_BOTH_EDGES,

    }
};

const wiced_gpio_t platform_gpio_leds[PLATFORM_LED_COUNT] =
{
     [WICED_LED_INDEX_1] = WICED_LED1,
     [WICED_LED_INDEX_2] = WICED_LED2,
};

platform_result_t platform_led_set_state(int led_index, int off_on )
{
    if ((led_index >= 0) && (led_index < PLATFORM_LED_COUNT))
    {
        switch (off_on)
        {
            case WICED_LED_ON:
                platform_gpio_output_low( &platform_gpio_pins[platform_gpio_leds[led_index]] );
                break;
            case WICED_LED_OFF:
                platform_gpio_output_high( &platform_gpio_pins[platform_gpio_leds[led_index]] );
                break;
        }
        return PLATFORM_SUCCESS;
    }
    return PLATFORM_BADARG;
}

/* Initialize LEDs and turn off */
void platform_init_leds( void )
{
    /* Initialise LEDs and turn off by default */
    platform_gpio_init( &platform_gpio_pins[WICED_LED1], OUTPUT_PUSH_PULL );
    platform_gpio_init( &platform_gpio_pins[WICED_LED2], OUTPUT_PUSH_PULL );

    platform_led_set_state(WICED_LED_INDEX_1, WICED_LED_OFF);
    platform_led_set_state(WICED_LED_INDEX_2, WICED_LED_OFF);
}

#ifdef WICED_PLATFORM_INCLUDES_OCF_FS
/** OCF  Filesystem */
/* Initialisation data for OCF device */
static wiced_block_device_init_data_t block_device_ocf_init_data =
{
    .base_address_offset                        = WICED_FILESYSTEM_OCF_START, /*0xA7000*/ /* Value -1 requests use of first free offset */
    .maximum_size                               = 0,   /* Value 0 requests use of maximum size of Flash */
    .volatile_and_requires_format_when_mounting = WICED_FALSE,
};
/* Variable workspace for Serial Flash device */
static ocf_block_device_specific_data_t block_device_ocf_specific_data;
/* Block device for serial flash device */
wiced_block_device_t block_device_ocf =
{
    .init_data            = &block_device_ocf_init_data,
    .driver               = &ocf_block_device_driver,
    .device_specific_data = &block_device_ocf_specific_data,
    .device_size = WICED_FILESYSTEM_OCF_SIZE, //0x59000
    .device_id = 0,
    .read_block_size = 1,   /** 1 indicates data can be accessed byte-by-byte */
    .write_block_size = 0,  /** Zero if writing is not allowed - e.g. device is read only. 1 indicates data can be accessed byte-by-byte */
    .erase_block_size = 0,  /** Zero if erasing is not required - e.g. for a RAM disk. 1 indicates data can be accessed byte-by-byte */
};
#endif // WICED_PLATFORM_INCLUDES_OCF_FS
#ifdef WICED_FILESYSTEM_SUPPORT
/* List of all filesystem devices on this platform - for interactive selection - e.g. console app */
const filesystem_list_t all_filesystem_devices[] =
{
#ifdef WICED_PLATFORM_INCLUDES_OCF_FS
    { &block_device_ocf, WICED_FILESYSTEM_HANDLE_WICEDFS, "OCF" },
#endif
    { NULL, 0, NULL },
};
#endif
/*
 #define JLINK_DEBUGGER_SUPPORT

 This feature helps to attach JLINK Debugger run time whenever user
 wants to debug. Since UART lines are muxes with SWD, we can use either
 console or SWD one at a time. Using the Button SW11 on the hardware, we
 can switch between console or JTAG mode (default console mode).
 */
#ifdef JLINK_DEBUGGER_SUPPORT
extern int gpio_function_Set_GPIO_Function(int, int);
static int jtag_onoff;
static void jlink_button_interrupt_handler( void* args )
{
    if(jtag_onoff)
    {
        jtag_onoff = 0;
        wiced_hal_puart_select_uart_pads(PLATFORM_SWDCK_PIN, PLATFORM_SWDIO_PIN, 0, 0);
        printf(" CONSOLE ON - JTAG GONE \n");
        printf(" Change SWITCH Setting on the board and check UART logs \n");
    }
    else
    {
        jtag_onoff = 1;
        printf(" JTAG ON - CONSOLE GONE \n");
        printf(" Change SWITCH Setting on the board and connect JLINK \n");
        gpio_function_Set_GPIO_Function( PLATFORM_SWDIO_PIN, 27 ); //SWDIO
        gpio_function_Set_GPIO_Function( PLATFORM_SWDCK_PIN, 26 ); //SWDCK

        wiced_hal_wdog_disable();
    }
}
#endif
void platform_init_external_devices( void )
{
    platform_init_leds();

    platform_gpio_init( &platform_gpio_pins[WICED_BUTTON1], INPUT_HIGH_IMPEDANCE );
#ifdef JLINK_DEBUGGER_SUPPORT
    wiced_gpio_input_irq_enable( WICED_BUTTON1, IRQ_TRIGGER_RISING_EDGE, jlink_button_interrupt_handler, NULL);
#endif

/*Add whatever external devices initialization code here*/
#ifndef WICED_DISABLE_STDIO
/* Initialise UART standard I/O */
    platform_stdio_init( &platform_uart_drivers[STDIO_UART], &platform_uart_peripherals[STDIO_UART], &stdio_config );
#endif
#ifdef WICED_FILESYSTEM_SUPPORT
#if defined(WICED_PLATFORM_INCLUDES_SPI_FLASH) || defined(WICED_PLATFORM_INCLUDES_OCF_FS)
    platform_filesystem_init ();
#endif
#endif
#ifdef ENABLE_BT_COEX
    wiced_bt_coex_enable(GCI_SECI_IN_PIN, GCI_SECI_OUT_PIN);
#endif
}

uint32_t  platform_get_button_press_time ( int button_index, int led_index, uint32_t max_time )
{
    int             button_gpio;
    uint32_t        button_press_timer = 0;
    int             led_state = 0;

    /* Initialize input */
    button_gpio     = platform_gpio_buttons[button_index].gpio;
    platform_gpio_init( &platform_gpio_pins[ button_gpio ], INPUT_HIGH_IMPEDANCE );
    platform_gpio_init( &platform_gpio_pins[platform_gpio_leds[led_index]] , OUTPUT_PUSH_PULL );
    platform_led_set_state(led_index, WICED_LED_OFF);
    while ( (PLATFORM_BUTTON_PRESSED_STATE == platform_gpio_input_get(&platform_gpio_pins[ button_gpio ])) )
    {
        /* wait a bit */
        host_rtos_delay_milliseconds( PLATFORM_BUTTON_PRESS_CHECK_PERIOD );

        /* Toggle LED */
        platform_led_set_state(led_index, (led_state == 0) ? WICED_LED_OFF : WICED_LED_ON);
        led_state ^= 0x01;

        /* keep track of time */
        button_press_timer += PLATFORM_BUTTON_PRESS_CHECK_PERIOD;
        if ((max_time > 0) && (button_press_timer >= max_time))
        {
            break;
        }
    }

     /* turn off the LED */
    platform_led_set_state(led_index, WICED_LED_OFF );

    return button_press_timer;
}

uint32_t  platform_get_factory_reset_button_time ( uint32_t max_time )
{
    return platform_get_button_press_time ( PLATFORM_FACTORY_RESET_BUTTON_INDEX, PLATFORM_RED_LED_INDEX, max_time );
}

