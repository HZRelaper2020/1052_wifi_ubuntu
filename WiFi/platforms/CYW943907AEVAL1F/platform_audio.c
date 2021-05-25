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

#include "wiced_platform.h"
#include "wiced_rtos.h" /* for wiced_mutex_t */
#include "platform_init.h"
#include "platform_peripheral.h"
#include "platform_config.h"
#include "wiced_audio.h"
#include "spdif.h"
#ifdef USE_CS47L24_AUDIO
#include "cs47l24.h"
#endif
#include "platform_external_memory.h"
#include "resources.h"

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

spdif_device_data_t i2s0_in =
{
    .port_type         = PLATFORM_AUDIO_I2S,
    .port_direction    = PLATFORM_AUDIO_DEVICE_INPUT,
    .data_port         = WICED_I2S_1,
};

spdif_device_data_t i2s0_out =
{
    .port_type         = PLATFORM_AUDIO_I2S,
    .port_direction    = PLATFORM_AUDIO_DEVICE_OUTPUT,
    .data_port         = WICED_I2S_2,
};

spdif_device_data_t i2s1_in =
{
    .port_type         = PLATFORM_AUDIO_I2S,
    .port_direction    = PLATFORM_AUDIO_DEVICE_INPUT,
    .data_port         = WICED_I2S_3,
};

spdif_device_data_t i2s1_out =
{
    .port_type         = PLATFORM_AUDIO_I2S,
    .port_direction    = PLATFORM_AUDIO_DEVICE_OUTPUT,
    .data_port         = WICED_I2S_4,
};

spdif_device_data_t spdif1_in =
{
    .port_type         = PLATFORM_AUDIO_SPDIF,
    .port_direction    = PLATFORM_AUDIO_DEVICE_INPUT,
    .data_port         = WICED_I2S_5,
};

#ifdef USE_CS47L24_AUDIO

#define CS47L24_ADC_DIGITAL_MIC_DESCRIPTION "in->J10.8,BLCK->J6.5,LRCLK->J6.6"
#define CS47L24_DAC_DESCRIPTION             "out->J6.10,BCLK->J6.29,LRCLK->J6.27"

#define CS47L24_REGISTER_DEVICES() \
    do \
    { \
        cs47l24_device_register( &cs47l24_dac, AUDIO_DEVICE_ID_CS47L24_DAC_LINE ); \
        cs47l24_device_register( &cs47l24_adc_dmic, AUDIO_DEVICE_ID_CS47L24_ADC_DIGITAL_MIC ); \
    } while (0)

wiced_spi_device_t cs47l24_control_port =
{
    .port        = WICED_SPI_1,
    .chip_select = WICED_GPIO_NONE /*WICED_GPIO_22*/,
    .speed       = 20000000,
    .mode        = (SPI_CLOCK_RISING_EDGE | SPI_CLOCK_IDLE_LOW | SPI_NO_DMA | SPI_MSB_FIRST | SPI_CS_ACTIVE_LOW),
    .bits        = 8
};

static const resource_hnd_t* dsp_wupd_fw_blobs[CS47L24_FIRMWARE_DSP_MAX] =
{
    &resources_drivers_DIR_audio_DIR_CS47L24_DIR_cs47l24_dsp2_wupd_wmfw,
    &resources_drivers_DIR_audio_DIR_CS47L24_DIR_cs47l24_dsp3_wupd_wmfw,
};

static cs47l24_dsp_resource_t cs47l24_dsp_res =
{
    .dsp_res_table = dsp_wupd_fw_blobs,
};

cs47l24_device_cmn_data_t cs47l24 =
{
    .id                 = CS47L24_DEVICE_ID_0,
    .spi_data           = &cs47l24_control_port,
    .ck                 = cs47l24_pll_slave,
    .pdn                = WICED_GPIO_11,
    .dsp                = &cs47l24_dsp_res,
};

cs47l24_device_data_t cs47l24_dac =
{
    .route              = &cs47l24_route_dac_hp,
    .cmn                = &cs47l24,
    .data_port          = WICED_I2S_4,
};

cs47l24_device_data_t cs47l24_adc_dmic =
{
    .route              = &cs47l24_route_adc_dmic,
    .cmn                = &cs47l24,
    .data_port          = WICED_I2S_1,
};

#else /* !USE_CS47L24_AUDIO */

#define CS47L24_REGISTER_DEVICES() do {} while (0)

#endif /* USE_CS47L24_AUDIO */

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

/* platform audio device defines */
#define SPDIF_ADC_DESCRIPTION               "in->J12.1"
#define I2S_0_ADC_DESCRIPTION               "in->J10.8,BLCK->J6.5,LRCLK->J6.6"
#define I2S_1_ADC_DESCRIPTION               "in->J12.1,BCLK->J6.29,LRCLK->J6.27"
#define I2S_0_DAC_DESCRIPTION               "out->J6.4,BLCK->J6.5,LRCLK->J6.6"
#define I2S_1_DAC_DESCRIPTION               "out->J6.10,BCLK->J6.29,LRCLK->J6.27"

/* defined here, specific to this platform, for platform_audio_device_info.c */
const platform_audio_device_info_t  platform_audio_input_devices[ PLATFORM_AUDIO_NUM_INPUTS ]  =
{
    AUDIO_DEVICE_ID_SPDIF_ADC_INFO,
    AUDIO_DEVICE_ID_I2S_0_ADC_INFO,
    AUDIO_DEVICE_ID_I2S_1_ADC_INFO,
#ifdef USE_CS47L24_AUDIO
    AUDIO_DEVICE_ID_CS47L24_ADC_DIGITAL_MIC_INFO,
#endif
};

const platform_audio_device_info_t  platform_audio_output_devices[ PLATFORM_AUDIO_NUM_OUTPUTS ] =
{
    AUDIO_DEVICE_ID_I2S_0_DAC_INFO,
    AUDIO_DEVICE_ID_I2S_1_DAC_INFO,
#ifdef USE_CS47L24_AUDIO
    AUDIO_DEVICE_ID_CS47L24_DAC_LINE_INFO,
#endif
};

static wiced_audio_device_interface_t platform_wiced_audio_devices[(PLATFORM_AUDIO_NUM_OUTPUTS+PLATFORM_AUDIO_NUM_INPUTS)];

/* audio_dev_class global is needed by WICED/internal/wiced_audio.c for audio device registration */
audio_device_class_t audio_dev_class =
{
    .audio_devices = platform_wiced_audio_devices,
    /* By default there are no devices registered */
    .device_count  = 0,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t platform_init_audio( void )
{
    /* Register audio device */
    spdif_device_register( &spdif1_in, AUDIO_DEVICE_ID_SPDIF_ADC );
    spdif_device_register( &i2s0_in,   AUDIO_DEVICE_ID_I2S_0_ADC );
    spdif_device_register( &i2s0_out,  AUDIO_DEVICE_ID_I2S_0_DAC );
    spdif_device_register( &i2s1_in,   AUDIO_DEVICE_ID_I2S_1_ADC );
    spdif_device_register( &i2s1_out,  AUDIO_DEVICE_ID_I2S_1_DAC );

    CS47L24_REGISTER_DEVICES();
    return WICED_SUCCESS;
}

wiced_result_t platform_deinit_audio( void )
{
    return WICED_UNSUPPORTED;
}

