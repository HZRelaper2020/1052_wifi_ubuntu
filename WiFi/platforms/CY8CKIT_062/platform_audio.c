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
 *
 */
#include "platform.h"
#include "platform_init.h"
#include "platform_peripheral.h"
#include "platform_config.h"
#include "platform_audio.h"
#include "ak4954.h"
#include "platform_external_memory.h"

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

#ifndef PLATFORM_AUDIO_CODEC_NONE
wiced_i2c_device_t ak4954_control_port =
{
    .port          = WICED_I2C_4,
    .address       = 0x12,
    .address_width = I2C_ADDRESS_WIDTH_7BIT,
    .flags         = I2C_DEVICE_NO_DMA,
    .speed_mode    = I2C_STANDARD_SPEED_MODE, //I2C_LOW_SPEED_MODE,
};

ak4954_device_cmn_data_t ak4954 =
{
    .id                 = AK4954_DEVICE_ID_0,
    .i2c_data           = &ak4954_control_port,
    .ck                 = ak4954_pll_slave,
    .pdn                = WICED_GPIO_70,
};

ak4954_device_data_t ak4954_dac =
{
    .route              = &ak4954_dac_hp,
    .cmn                = &ak4954,
    .data_port          = WICED_I2S_1,
};

ak4954_device_data_t ak4954_adc =
{
    .route              = &ak4954_adc_mic,
    .cmn                = &ak4954,
    .data_port          = WICED_I2S_2,
};
#endif

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

#ifndef PLATFORM_AUDIO_CODEC_NONE
/* platform audio device defines */
#define AK4954_ADC_DESCRIPTION  "3 conductor 3.5mm @ J6"
#define AK4954_DAC_DESCRIPTION  "3 conductor 3.5mm @ J6"

/* defined here, specific to this platform, for platform_audio_device_info.c */
const platform_audio_device_info_t  platform_audio_input_devices[ PLATFORM_AUDIO_NUM_INPUTS ]  =
{
    AUDIO_DEVICE_ID_AK4954_ADC_LINE_INFO,
};

const platform_audio_device_info_t  platform_audio_output_devices[ PLATFORM_AUDIO_NUM_OUTPUTS ] =
{
    AUDIO_DEVICE_ID_AK4954_DAC_LINE_INFO,
};

static wiced_audio_device_interface_t platform_wiced_audio_devices[(PLATFORM_AUDIO_NUM_OUTPUTS+PLATFORM_AUDIO_NUM_INPUTS)];

/* audio_dev_class global is needed by WICED/internal/wiced_audio.c for audio device registration */
audio_device_class_t audio_dev_class =
{
    .audio_devices = platform_wiced_audio_devices,
    /* By default there are no devices registered */
    .device_count  = 0,
};
#else
#define DEFAULT_DAC_NAME         "default_dac"
#define DEFAULT_DAC_DESCRIPTION  "default_dac_description"
#define DEFAULT_DAC_DIRECTION    PLATFORM_AUDIO_DEVICE_OUTPUT
#define DEFAULT_DAC_PORT_TYPE    PLATFORM_AUDIO_LINE
#define DEFAULT_DAC_CHANNELS     2
#define DEFAULT_DAC_SIZES        (PLATFORM_AUDIO_SAMPLE_SIZE_16_BIT)
#define DEFAULT_DAC_RATES        (PLATFORM_AUDIO_SAMPLE_RATE_44_1KHZ)

#define AUDIO_DEVICE_ID_DEFAULT_DAC_INFO                    \
        { AUDIO_DEVICE_ID_OUTPUT_NONE, DEFAULT_DAC_NAME,    \
          DEFAULT_DAC_DESCRIPTION, DEFAULT_DAC_DIRECTION,   \
          DEFAULT_DAC_PORT_TYPE, DEFAULT_DAC_CHANNELS,      \
          DEFAULT_DAC_SIZES, DEFAULT_DAC_RATES }

const platform_audio_device_info_t  platform_audio_output_devices[1] =
{
    AUDIO_DEVICE_ID_DEFAULT_DAC_INFO,
};

static wiced_audio_device_interface_t platform_wiced_audio_devices[1];

/* audio_dev_class global is needed by WICED/internal/wiced_audio.c for audio device registration */
audio_device_class_t audio_dev_class =
{
    .audio_devices = platform_wiced_audio_devices,
    /* By default there are no devices registered */
    .device_count  = 0,
};
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/

#ifdef PLATFORM_AUDIO_CODEC_NONE
static wiced_result_t default_dac_init(void *driver_data, wiced_audio_data_port_t* data_port)
{
    data_port->type = PLATFORM_AUDIO_LINE;
    data_port->port = WICED_I2S_1;
    data_port->channel = WICED_PLAY_CHANNEL;

    return WICED_SUCCESS;
}

static wiced_result_t default_dac_deinit(void *driver_data)
{
    return WICED_SUCCESS;
}

static wiced_result_t default_dac_configure( void *driver_data, wiced_audio_config_t *config, uint32_t* mclk)
{
    return WICED_SUCCESS;
}

static wiced_result_t default_dac_start_play ( void* driver_data )
{
    return WICED_SUCCESS;
}

static wiced_result_t default_dac_stop_play ( void* driver_data )
{
    return WICED_SUCCESS;
}

static wiced_result_t default_dac_set_playback_volume(void *driver_data, double decibels)
{
    return WICED_SUCCESS;
}

static wiced_result_t default_dac_get_playback_volume_range(void *driver_data, double *min_volume_in_decibels, double *max_volume_in_decibels )
{
    return WICED_SUCCESS;
}

wiced_audio_device_interface_t default_dac_playback =
{
    .device_id                      = AUDIO_DEVICE_ID_OUTPUT_NONE,
    .audio_device_driver_specific   = NULL,
    .audio_device_init              = default_dac_init,
    .audio_device_deinit            = default_dac_deinit,
    .audio_device_configure         = default_dac_configure,
    .audio_device_start_streaming   = default_dac_start_play,
    .audio_device_stop_streaming    = default_dac_stop_play,
    .audio_device_set_volume        = default_dac_set_playback_volume,
    .audio_device_get_volume_range  = default_dac_get_playback_volume_range,
    .audio_device_set_treble        = NULL,
    .audio_device_set_bass          = NULL,
};
#endif

wiced_result_t platform_init_audio( void )
{
    /* Register audio device */
#ifndef PLATFORM_AUDIO_CODEC_NONE
    ak4954_device_register( &ak4954_dac, AUDIO_DEVICE_ID_AK4954_DAC_LINE );
    ak4954_device_register( &ak4954_adc, AUDIO_DEVICE_ID_AK4954_ADC_LINE );
#else
    wiced_register_audio_device( default_dac_playback.device_id, &default_dac_playback );
#endif

    return WICED_SUCCESS;
}

wiced_result_t platform_deinit_audio( void )
{
    return WICED_UNSUPPORTED;
}

uint8_t platform_audio_get_device_count(void)
{
#ifndef PLATFORM_AUDIO_CODEC_NONE
    return ((uint8_t)PLATFORM_AUDIO_NUM_OUTPUTS + (uint8_t)PLATFORM_AUDIO_NUM_INPUTS);
#else
    return 1;
#endif
}
