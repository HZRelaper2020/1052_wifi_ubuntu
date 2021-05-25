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
#include "wiced_rtos.h" /* for wiced_mutex_t */
#include "platform_init.h"
#include "platform_peripheral.h"
#include "platform_config.h"
#include "wiced_audio.h"
#include "ak4961.h"
#include "spdif.h"
#include "platform_external_memory.h"
#include <resources.h>

/******************************************************
 *                      Macros
 ******************************************************/

#if !defined(ARRAYSIZE)
#define ARRAYSIZE(x) (sizeof(x)/sizeof(x[0]))
#endif

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

enum ak4961_device_id
{
    AK4961_DEVICE_ID_0          = 0,

    /* Not a device id! */
    AK4961_DEVICE_ID_MAX,
};

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

static ak4961_device_runtime_data_t ak4961_device_runtime_data[AK4961_DEVICE_ID_MAX];

wiced_i2c_device_t ak4961_control_port =
{
    .port               = WICED_I2C_1,
    .address            = 0x12,
    .address_width      = I2C_ADDRESS_WIDTH_7BIT,
    .speed_mode         = I2C_HIGH_SPEED_MODE,
};

const ak4961_dsp_ram_resource_t ak4961_dsp_effect_bass_treble_pram_res[] = {
    /* Sample Rate : 44100  */
    {  44100, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_bass_treble_pram_bin        },
    /* Sample Rate : 48000  */
    {  48000, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_bass_treble_pram_bin        },
    /* Sample Rate : 88200  */
    {  88200, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_bass_treble_pram_bin        },
    /* Sample Rate : 96000  */
    {  96000, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_bass_treble_pram_bin        },
    /* Sample Rate : 176400 */
    { 176400, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_bass_treble_pram_bin        },
    /* Sample Rate : 192000 */
    { 192000, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_bass_treble_pram_bin        },
};

const ak4961_dsp_ram_resource_t ak4961_dsp_effect_bass_treble_cram_res[] = {
    /* Sample Rate : 44100  */
    {  44100, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_bass_treble_cram_44_1k_bin  },
    /* Sample Rate : 48000  */
    {  48000, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_bass_treble_cram_48k_bin    },
    /* Sample Rate : 88200  */
    {  88200, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_bass_treble_cram_88_2k_bin  },
    /* Sample Rate : 96000  */
    {  96000, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_bass_treble_cram_96k_bin    },
    /* Sample Rate : 176400 */
    { 176400, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_bass_treble_cram_176_4k_bin },
    /* Sample Rate : 192000 */
    { 192000, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_bass_treble_cram_192k_bin   },
 };

const ak4961_dsp_ram_resource_t ak4961_dsp_effect_preset_pram_res[] = {
    /* Sample Rate : 44100  */
    {  44100, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_preset_pram_bin        },
    /* Sample Rate : 48000  */
    {  48000, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_preset_pram_bin        },
    /* Sample Rate : 88200  */
    {  88200, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_preset_pram_bin        },
    /* Sample Rate : 96000  */
    {  96000, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_preset_pram_bin        },
    /* Sample Rate : 176400 */
    { 176400, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_preset_pram_bin        },
    /* Sample Rate : 192000 */
    { 192000, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_preset_pram_bin        },
};

const ak4961_dsp_ram_resource_t ak4961_dsp_effect_preset_cram_res[] = {
    /* Sample Rate : 44100  */
    {  44100, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_preset_cram_44_1k_bin  },
    /* Sample Rate : 48000  */
    {  48000, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_preset_cram_48k_bin    },
    /* Sample Rate : 88200  */
    {  88200, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_preset_cram_88_2k_bin  },
    /* Sample Rate : 96000  */
    {  96000, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_preset_cram_96k_bin    },
    /* Sample Rate : 176400 */
    { 176400, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_preset_cram_176_4k_bin },
    /* Sample Rate : 192000 */
    { 192000, &resources_drivers_DIR_audio_DIR_AK4961_DIR_dsp_effect_preset_cram_192k_bin   },
 };

/* Command[0x81],Run State Data Length Register[0x009C],RAM Write Data Number[0x00],RAM Address[0x006B, 0x00D3],DSP MODE DATA[0x000010,0x000020,0x000040]  */
const uint8_t ak4961_dsp_effect_bass_boost[]   = { 0x81, 0x00, 0x9C, 0x00, 0x00, 0x6B, 0x00, 0x00, 0x20 };
const uint8_t ak4961_dsp_effect_treble_boost[] = { 0x81, 0x00, 0x9C, 0x00, 0x00, 0x6B, 0x00, 0x00, 0x40 };
const uint8_t ak4961_dsp_effect_jazz[]         = { 0x81, 0x00, 0x9C, 0x00, 0x00, 0xD3, 0x00, 0x00, 0x10 };
const uint8_t ak4961_dsp_effect_pop[]          = { 0x81, 0x00, 0x9C, 0x00, 0x00, 0xD3, 0x00, 0x00, 0x20 };
const uint8_t ak4961_dsp_effect_rock[]         = { 0x81, 0x00, 0x9C, 0x00, 0x00, 0xD3, 0x00, 0x00, 0x40 };

const ak4961_dsp_effect_data_t ak4961_dsp_effect_data[AK4961_DSP_EFFECT_MODE_MAX] =
{
    [AK4961_DSP_EFFECT_MODE_NONE] =
    {
        .dsp                    = NULL,
        .dsp_source_port        = NULL,
        .dsp_sync_domain_select = AK4961_SYNC_DOMAIN_SELECT_NA,
        .dsp_mode_ram           = NULL,
        .dsp_mode_ram_size      = 0,
        .pram_res               = NULL,
        .cram_res               = NULL,
        .pram_tab_size          = 0,
        .cram_tab_size          = 0,
    },
    [AK4961_DSP_EFFECT_MODE_BASS_BOOST] =
    {
        .dsp                    = &ak4961_dsp1,
        .dsp_source_port        = AK4961_SOURCE_PORT_DSPO1,
        .dsp_sync_domain_select = AK4961_SYNC_DOMAIN_SELECT_1,
        .dsp_mode_ram           = ak4961_dsp_effect_bass_boost,
        .dsp_mode_ram_size      = ARRAYSIZE(ak4961_dsp_effect_bass_boost),
        .pram_res               = ak4961_dsp_effect_bass_treble_pram_res,
        .cram_res               = ak4961_dsp_effect_bass_treble_cram_res,
        .pram_tab_size          = ARRAYSIZE(ak4961_dsp_effect_bass_treble_pram_res),
        .cram_tab_size          = ARRAYSIZE(ak4961_dsp_effect_bass_treble_cram_res),
    },
    [AK4961_DSP_EFFECT_MODE_TREBLE_BOOST] =
    {
        .dsp                    = &ak4961_dsp1,
        .dsp_source_port        = AK4961_SOURCE_PORT_DSPO1,
        .dsp_sync_domain_select = AK4961_SYNC_DOMAIN_SELECT_1,
        .dsp_mode_ram           = ak4961_dsp_effect_treble_boost,
        .dsp_mode_ram_size      = ARRAYSIZE(ak4961_dsp_effect_treble_boost),
        .pram_res               = ak4961_dsp_effect_bass_treble_pram_res,
        .cram_res               = ak4961_dsp_effect_bass_treble_cram_res,
        .pram_tab_size          = ARRAYSIZE(ak4961_dsp_effect_bass_treble_pram_res),
        .cram_tab_size          = ARRAYSIZE(ak4961_dsp_effect_bass_treble_cram_res),
    },
    [AK4961_DSP_EFFECT_MODE_JAZZ] =
    {
        .dsp                    = &ak4961_dsp1,
        .dsp_source_port        = AK4961_SOURCE_PORT_DSPO1,
        .dsp_sync_domain_select = AK4961_SYNC_DOMAIN_SELECT_1,
        .dsp_mode_ram           = ak4961_dsp_effect_jazz,
        .dsp_mode_ram_size      = ARRAYSIZE(ak4961_dsp_effect_jazz),
        .pram_res               = ak4961_dsp_effect_preset_pram_res,
        .cram_res               = ak4961_dsp_effect_preset_cram_res,
        .pram_tab_size          = ARRAYSIZE(ak4961_dsp_effect_preset_pram_res),
        .cram_tab_size          = ARRAYSIZE(ak4961_dsp_effect_preset_cram_res),
    },
    [AK4961_DSP_EFFECT_MODE_POP] =
    {
        .dsp                    = &ak4961_dsp1,
        .dsp_source_port        = AK4961_SOURCE_PORT_DSPO1,
        .dsp_sync_domain_select = AK4961_SYNC_DOMAIN_SELECT_1,
        .dsp_mode_ram           = ak4961_dsp_effect_pop,
        .dsp_mode_ram_size      = ARRAYSIZE(ak4961_dsp_effect_pop),
        .pram_res               = ak4961_dsp_effect_preset_pram_res,
        .cram_res               = ak4961_dsp_effect_preset_cram_res,
        .pram_tab_size          = ARRAYSIZE(ak4961_dsp_effect_preset_pram_res),
        .cram_tab_size          = ARRAYSIZE(ak4961_dsp_effect_preset_cram_res),
    },
    [AK4961_DSP_EFFECT_MODE_ROCK] =
    {
        .dsp                    = &ak4961_dsp1,
        .dsp_source_port        = AK4961_SOURCE_PORT_DSPO1,
        .dsp_sync_domain_select = AK4961_SYNC_DOMAIN_SELECT_1,
        .dsp_mode_ram           = ak4961_dsp_effect_rock,
        .dsp_mode_ram_size      = ARRAYSIZE(ak4961_dsp_effect_rock),
        .pram_res               = ak4961_dsp_effect_preset_pram_res,
        .cram_res               = ak4961_dsp_effect_preset_cram_res,
        .pram_tab_size          = ARRAYSIZE(ak4961_dsp_effect_preset_pram_res),
        .cram_tab_size          = ARRAYSIZE(ak4961_dsp_effect_preset_cram_res),
    },
};

ak4961_device_cmn_data_t ak4961 =
{
    .rtd                    = &ak4961_device_runtime_data[AK4961_DEVICE_ID_0],
    .i2c_data               = &ak4961_control_port,
    .ck                     = ak4961_pll_slave,
    .pdn                    = WICED_GPIO_AKM_PDN,
    .switcher_3v3_ps_enable = WICED_GPIO_AKM_SWITCHER_3V3_PS_ENABLE,
    .switcher_2v_enable     = WICED_GPIO_AKM_SWITCHER_2V_ENABLE,
    .ldo_1v8_enable         = WICED_GPIO_AKM_LDO_1V8_ENABLE,
    .dsp_effect_data        = &ak4961_dsp_effect_data[AK4961_DSP_EFFECT_MODE_NONE],
};

wiced_audio_device_interface_t ak4961_dac1_hp_device_interface = AK4961_DAC_AUDIO_DEVICE_INTERFACE_INITIALIZER();

const ak4961_device_route_t ak4961_dac1_hp_device_route = AK4961_DAC1_HP_INITIALIZER(&ak4961_dac1_hp_device_interface);

/* SDTI1A -> DAC1 -> HP */
ak4961_dac_route_data_t ak4961_dac1_route_data =
{
    .base.id             = AK4961_ROUTE_ID_0,
    .base.device_type    = AK4961_DEVICE_TYPE_PLAYBACK,
    .base.device_route   = &ak4961_dac1_hp_device_route,
    .source_port         = AK4961_SOURCE_PORT_SDTI1A,
    .sync_domain_select  = AK4961_SYNC_DOMAIN_SELECT_1,
    .output_left_select  = AK4961_DAC_OUTPUT_SELECT_LCH,
    .output_right_select = AK4961_DAC_OUTPUT_SELECT_RCH,
    .amp_gain_default    = AK4961_DAC_HP_AMP_GAIN_0DB_DEFAULT,
    .amp_gain_current    = AK4961_DAC_HP_AMP_GAIN_0DB_DEFAULT,
    .amp_gain_mute       = AK4961_DAC_HP_AMP_GAIN_MUTE,
    .digital_volume      = AK4961_DAC_DIGITAL_VOLUME_0DB_DEFAULT,
};

/* SDTI3 -> DAC1 -> HP */
ak4961_dac_route_data_t ak4961_dac1_bt_route_data =
{
    .base.id             = AK4961_ROUTE_ID_3,
    .base.device_type    = AK4961_DEVICE_TYPE_PLAYBACK,
    .base.device_route   = &ak4961_dac1_hp_device_route,
    .source_port         = AK4961_SOURCE_PORT_SDTI3,
    .sync_domain_select  = AK4961_SYNC_DOMAIN_SELECT_3,
    .output_left_select  = AK4961_DAC_OUTPUT_SELECT_LCH,
    .output_right_select = AK4961_DAC_OUTPUT_SELECT_RCH,
    .amp_gain_default    = AK4961_DAC_HP_AMP_GAIN_0DB_DEFAULT,
    .amp_gain_current    = AK4961_DAC_HP_AMP_GAIN_0DB_DEFAULT,
    .amp_gain_mute       = AK4961_DAC_HP_AMP_GAIN_MUTE,
    .digital_volume      = AK4961_DAC_DIGITAL_VOLUME_0DB_DEFAULT,
};

/* AIN1P <-> MPWR1A */
static const ak4961_adc_analog_input_t ak4961_adc1_analog_mic_input_left =
{
    .adc_input_select   = AK4961_ADC_INPUT_SELECT_AIN1,
    .power_supply       = AK4961_MIC_POWER_SUPPLY_MPWR1A,
    .output_voltage     = AK4961_MIC_POWER_OUTPUT_VOLTAGE_DEFAULT,
    .amp_gain           = AK4961_ADC_MIC_AMP_GAIN_18DB_DEFAULT,
};

#ifdef NOTYET
/* AIN2P <-> MPWR1B */
static const ak4961_adc_analog_input_t ak4961_adc1_analog_mic_input_right =
{
    .adc_input_select   = AK4961_ADC_INPUT_SELECT_AIN2,
    .power_supply       = AK4961_MIC_POWER_SUPPLY_MPWR1B,
    .output_voltage     = AK4961_MIC_POWER_OUTPUT_VOLTAGE_DEFAULT,
    .amp_gain           = AK4961_ADC_MIC_AMP_GAIN_18DB_DEFAULT,
};
#endif

/* AIN1P without MIC bias voltage */
static const ak4961_adc_analog_input_t ak4961_adc1_line_input_left =
{
    .adc_input_select   = AK4961_ADC_INPUT_SELECT_AIN1,
    .power_supply       = AK4961_MIC_POWER_SUPPLY_MAX,
    .output_voltage     = AK4961_MIC_POWER_OUTPUT_VOLTAGE_DEFAULT, /* not used in that case */
    .amp_gain           = AK4961_ADC_MIC_AMP_GAIN_6DB_DEFAULT,
};

/* AIN2P without MIC bias voltage */
static const ak4961_adc_analog_input_t ak4961_adc1_line_input_right =
{
    .adc_input_select   = AK4961_ADC_INPUT_SELECT_AIN2,
    .power_supply       = AK4961_MIC_POWER_SUPPLY_MAX,
    .output_voltage     = AK4961_MIC_POWER_OUTPUT_VOLTAGE_DEFAULT, /* not used in that case */
    .amp_gain           = AK4961_ADC_MIC_AMP_GAIN_6DB_DEFAULT,
};

wiced_audio_device_interface_t ak4961_adc1_mic_device_interface = AK4961_ADC_AUDIO_DEVICE_INTERFACE_INITIALIZER();

const ak4961_device_route_t ak4961_adc1_mic_device_route = AK4961_ADC1_MIC_INITIALIZER(&ak4961_adc1_mic_device_interface);

/* AIN1 -> ADC1 -> SDTO1A */
const ak4961_adc_route_data_t ak4961_mic_adc1_route_data =
{
    .base.id            = AK4961_ROUTE_ID_1,
    .base.device_type   = AK4961_DEVICE_TYPE_CAPTURE,
    .base.device_route  = &ak4961_adc1_mic_device_route,
    .sink_port          = AK4961_SINK_PORT_SDTO1A,
    .sync_domain_select = AK4961_SYNC_DOMAIN_SELECT_1,
    .type.analog        = {
        .input_left     = &ak4961_adc1_analog_mic_input_left,
        .input_right    = NULL,
    },
};

/* AIN1 -> ADC1 -> SDTO1A */
const ak4961_adc_route_data_t ak4961_line_adc1_route_data =
{
    .base.id            = AK4961_ROUTE_ID_1,
    .base.device_type   = AK4961_DEVICE_TYPE_CAPTURE,
    .base.device_route  = &ak4961_adc1_mic_device_route,
    .sink_port          = AK4961_SINK_PORT_SDTO1A,
    .sync_domain_select = AK4961_SYNC_DOMAIN_SELECT_1,
    .type.analog        = {
        .input_left     = &ak4961_adc1_line_input_left,
        .input_right    = &ak4961_adc1_line_input_right,
    },
};

/* AIN1 -> ADC1 -> SDTO3 */
const ak4961_adc_route_data_t ak4961_adc1_bt_route_data =
{
    .base.id            = AK4961_ROUTE_ID_4,
    .base.device_type   = AK4961_DEVICE_TYPE_CAPTURE,
    .base.device_route  = &ak4961_adc1_mic_device_route,
    .sink_port          = AK4961_SINK_PORT_SDTO3,
    .sync_domain_select = AK4961_SYNC_DOMAIN_SELECT_3,
    .type.analog        = {
        .input_left     = &ak4961_adc1_analog_mic_input_left,
        .input_right    = NULL,
    },
};

wiced_audio_device_interface_t ak4961_adc1_dmic1_device_interface = AK4961_ADC_AUDIO_DEVICE_INTERFACE_INITIALIZER();

const ak4961_device_route_t ak4961_adc1_dmic1_device_route = AK4961_ADC1_DMIC1_INITIALIZER(&ak4961_adc1_dmic1_device_interface);

/* DMIC1 -> ADC1 -> SDTO1A */
const ak4961_adc_route_data_t ak4961_adc1_dmic1_route_data =
{
    .base.id            = AK4961_ROUTE_ID_2,
    .base.device_type   = AK4961_DEVICE_TYPE_CAPTURE,
    .base.device_route  = &ak4961_adc1_dmic1_device_route,
    .sink_port          = AK4961_SINK_PORT_SDTO1A,
    .sync_domain_select = AK4961_SYNC_DOMAIN_SELECT_1,
    .type.digital       = {
        .lch_enabled    = 1,
        .rch_enabled    = 1,
        .polarity       = AK4961_DMIC_POLARITY_DEFAULT,
    },
};

/* DMIC1 -> ADC1 -> SDTO3 */
const ak4961_adc_route_data_t ak4961_adc1_dmic1_bt_route_data =
{
    .base.id            = AK4961_ROUTE_ID_5,
    .base.device_type   = AK4961_DEVICE_TYPE_CAPTURE,
    .base.device_route  = &ak4961_adc1_dmic1_device_route,
    .sink_port          = AK4961_SINK_PORT_SDTO3,
    .sync_domain_select = AK4961_SYNC_DOMAIN_SELECT_3,
    .type.digital       = {
        .lch_enabled    = 1,
        .rch_enabled    = 1,
        .polarity       = AK4961_DMIC_POLARITY_DEFAULT,
    },
};

ak4961_device_data_t ak4961_dac =
{
     .route             = &ak4961_dac1_route_data,
     .cmn               = &ak4961,
     .data_port         = WICED_I2S_1,
};

ak4961_device_data_t ak4961_bt_dac =
{
     .route             = &ak4961_dac1_bt_route_data,
     .cmn               = &ak4961,
     .data_port         = WICED_I2S_MAX,
};

ak4961_device_data_t ak4961_adc_line =
{
     .route             = &ak4961_line_adc1_route_data,
     .cmn               = &ak4961,
     .data_port         = WICED_I2S_2,
};

ak4961_device_data_t ak4961_adc_mic =
{
     .route             = &ak4961_mic_adc1_route_data,
     .cmn               = &ak4961,
     .data_port         = WICED_I2S_2,
};

ak4961_device_data_t ak4961_bt_adc =
{
     .route             = &ak4961_adc1_bt_route_data,
     .cmn               = &ak4961,
     .data_port         = WICED_I2S_MAX,
};

ak4961_device_data_t ak4961_adc_dmic1 =
{
    .route             = &ak4961_adc1_dmic1_route_data,
    .cmn               = &ak4961,
    .data_port         = WICED_I2S_3,
};

ak4961_device_data_t ak4961_bt_adc_dmic1 =
{
    .route             = &ak4961_adc1_dmic1_bt_route_data,
    .cmn               = &ak4961,
    .data_port         = WICED_I2S_MAX,
};

spdif_device_data_t spdif_in =
{
    .port_type         = PLATFORM_AUDIO_SPDIF,
    .port_direction    = PLATFORM_AUDIO_DEVICE_INPUT,
    .data_port         = WICED_I2S_4,
};

spdif_device_data_t i2s0_in =
{
    .port_type         = PLATFORM_AUDIO_I2S,
    .port_direction    = PLATFORM_AUDIO_DEVICE_INPUT,
    .data_port         = WICED_I2S_6,
};

spdif_device_data_t i2s1_out =
{
    .port_type         = PLATFORM_AUDIO_I2S,
    .port_direction    = PLATFORM_AUDIO_DEVICE_OUTPUT,
    .data_port         = WICED_I2S_5,
};

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

/* platform audio device defines */
#define AK4961_ADC_LINE_DESCRIPTION           "4 conductor 3.5mm @ J3"
#define AK4961_ADC_MIC_DESCRIPTION            "4 conductor 3.5mm @ J3"
#define AK4961_DAC_DESCRIPTION                "4 conductor 3.5mm @ J3"
#define AK4961_ADC_DIGITAL_MIC_DESCRIPTION    "digital mic @ MIC 1"
#define SPDIF_ADC_DESCRIPTION                 "optical connector @ J12"
#define AK4961_BT_ADC_DESCRIPTION             "4 conductor 3.5mm @ J3 (for BT audio)"
#define AK4961_BT_DAC_DESCRIPTION             "4 conductor 3.5mm @ J3 (for BT audio)"
#define AK4961_BT_ADC_DIGITAL_MIC_DESCRIPTION "digital mic @ MIC 1 (for BT audio)"
#define I2S_0_ADC_DESCRIPTION                 "in->J15.7,BCLK->J15.5,LRCLK->J15.2"
#define I2S_1_ADC_DESCRIPTION                 "in->TP5,BCLK->J14.9,LRCLK->J14.7"
#define I2S_0_DAC_DESCRIPTION                 "not available"
#define I2S_1_DAC_DESCRIPTION                 "out->J14.10,BCLK->J14.9,LRCLK->J14.7"


/* defined here, specific to this platform, for platform_audio_device_info.c */
const platform_audio_device_info_t  platform_audio_input_devices[ PLATFORM_AUDIO_NUM_INPUTS ]  =
{
    AUDIO_DEVICE_ID_AK4961_ADC_MIC_INFO,
    AUDIO_DEVICE_ID_AK4961_ADC_DIGITAL_MIC_INFO,
    AUDIO_DEVICE_ID_SPDIF_ADC_INFO,
    AUDIO_DEVICE_ID_AK4961_BT_ADC_LINE_INFO,
    AUDIO_DEVICE_ID_AK4961_BT_ADC_DIGITAL_MIC_INFO,
    AUDIO_DEVICE_ID_I2S_0_ADC_INFO,
    AUDIO_DEVICE_ID_AK4961_ADC_LINE_INFO,
};

const platform_audio_device_info_t  platform_audio_output_devices[ PLATFORM_AUDIO_NUM_OUTPUTS ] =
{
    AUDIO_DEVICE_ID_AK4961_DAC_LINE_INFO,
    AUDIO_DEVICE_ID_AK4961_BT_DAC_LINE_INFO,
    AUDIO_DEVICE_ID_I2S_1_DAC_INFO,
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
    ak4961_device_register( &ak4961_dac, AUDIO_DEVICE_ID_AK4961_DAC_LINE );

    ak4961_device_register( &ak4961_adc_mic, AUDIO_DEVICE_ID_AK4961_ADC_MIC );

    ak4961_device_register( &ak4961_adc_dmic1, AUDIO_DEVICE_ID_AK4961_ADC_DIGITAL_MIC );

    spdif_device_register( &spdif_in, AUDIO_DEVICE_ID_SPDIF_ADC );

    ak4961_device_register( &ak4961_bt_dac, AUDIO_DEVICE_ID_AK4961_BT_DAC_LINE );

    ak4961_device_register( &ak4961_bt_adc, AUDIO_DEVICE_ID_AK4961_BT_ADC_LINE );

    ak4961_device_register( &ak4961_bt_adc_dmic1, AUDIO_DEVICE_ID_AK4961_BT_ADC_DIGITAL_MIC );

    spdif_device_register( &i2s0_in,  AUDIO_DEVICE_ID_I2S_0_ADC );

    spdif_device_register( &i2s1_out, AUDIO_DEVICE_ID_I2S_1_DAC );

    ak4961_device_register( &ak4961_adc_line, AUDIO_DEVICE_ID_AK4961_ADC_LINE );

    return WICED_SUCCESS;
}

wiced_result_t platform_deinit_audio( void )
{
    return WICED_UNSUPPORTED;
}

wiced_result_t ak4961_platform_configure( ak4961_device_data_t* device_data, uint32_t mclk, uint32_t fs, uint8_t width )
{
    UNUSED_PARAMETER( device_data );
    UNUSED_PARAMETER( mclk );
    UNUSED_PARAMETER( device_data );
    UNUSED_PARAMETER( width );

    return WICED_SUCCESS;
}

wiced_result_t ak4961_free_dsp_ram_resource( const resource_hnd_t *resource, const uint8_t* buffer )
{
    resource_free_readonly_buffer( resource, buffer );

    return WICED_SUCCESS;
}

wiced_result_t ak4961_get_dsp_ram_resource( const resource_hnd_t *resource, const uint8_t** buffer, uint32_t* size )
{
    resource_result_t res_result;

    res_result = resource_get_readonly_buffer( resource, 0, 0x7fffffff, size, (const void**)buffer );

    if (res_result != RESOURCE_SUCCESS)
    {
        /* Failed reading the resource! */
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}
