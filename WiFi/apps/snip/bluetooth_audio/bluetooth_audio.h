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
 * Bluetooth A2DP Application Programming Interface
 *
 */
#pragma once

#include "wiced_bt_a2dp_sink.h"
#include "wiced_audio.h"
#include "wiced_codec_if.h"

#ifdef USE_AUDIO_PLL
#include "audio_pll_tuner.h"
#endif /* USE_AUDIO_PLL */

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************
**  macros
*****************************************************************************/

#define MILLISECONDS_TO_BYTES(duration_msecs, sample_rate, block_align) \
    (double)((double)(duration_msecs * sample_rate * block_align) / (double)MILLISECONDS_PER_SECOND)

#define BYTES_TO_MILLISECONDS(number_of_bytes, sample_rate, block_align) \
    ((double)(((MICROSECONDS_PER_SECOND / ((double)(sample_rate * block_align * 1.0f))) * number_of_bytes) / (double)MILLISECONDS_PER_SECOND))

#ifdef USE_AUDIO_PLL

#define AUDIO_PLL_CONSOLE_COMMANDS \
    { (char*) "target",    audio_pll_console_command, 1, NULL, NULL, (char *)"", (char *)"Set target buffer level as percentage of audio buffer duration (1 to 100)"         }, \
    { (char*) "threshold", audio_pll_console_command, 1, NULL, NULL, (char *)"", (char *)"Set start of playback threshold as percentage of audio buffer duration (1 to 100)" }, \
    { (char*) "log",       audio_pll_console_command, 1, NULL, NULL, (char *)"", (char *)"Set log level" }, \

#else /* !USE_AUDIO_PLL */

#define AUDIO_PLL_CONSOLE_COMMANDS

#endif /* USE_AUDIO_PLL */

/*****************************************************************************
**  constants
*****************************************************************************/

#define BT_AUDIO_DEFAULT_VOLUME                   63 /* mid of 0-127 */
#define BT_AUDIO_VOLUME_STEP                      8
#define BT_AUDIO_VOLUME_MAX                       127
#define BT_AUDIO_VOLUME_MIN                       0

#define BT_AUDIO_BUFFER_DEFAULT_MAX_DURATION_MSEC (300)
#define BT_AUDIO_BUFFER_DEFAULT_TARGET_PERCENT    (25)
#define BT_AUDIO_BUFFER_DEFAULT_THRESHOLD_PERCENT (25)

#define MILLISECONDS_PER_SECOND                   (1000)
#define MICROSECONDS_PER_SECOND                   (1000000)

/*****************************************************************************
 ** enumerations
 *****************************************************************************/

typedef enum
{
    BT_AUDIO_EVENT_START_PLAYER  = 0x00000001,
    BT_AUDIO_EVENT_STOP_PLAYER   = 0x00000002,
    BT_AUDIO_EVENT_ALL           = 0xFFFFFFFF
}player_event_flags_t;


typedef enum
{
    BT_AUDIO_DEVICE_STATE_UNINITIALIZED,
    BT_AUDIO_DEVICE_STATE_IDLE,
    BT_AUDIO_DEVICE_STATE_CONFIGURED,
    BT_AUDIO_DEVICE_STATE_STARTED,
    BT_AUDIO_DEVICE_STATE_STOPPED
}bt_audio_device_state_t;

typedef enum
{
    BT_AUDIO_CODEC_EVENT_DECODE        = 0x00000001,
    BT_AUDIO_CODEC_EVENT_CONFIGURE   = 0x00000002,
    BT_AUDIO_CODEC_EVENT_ALL               = 0xFFFFFFFF
}codec_event_flags_t;

/*****************************************************************************
**  type definitions
*****************************************************************************/
typedef struct
{
    uint16_t  length;
    uint16_t  offset;
    uint8_t   data[]; //data of length size.
}bt_audio_codec_data_t;

typedef struct
{
    uint32_t sample_rate;
    uint8_t  bits_per_sample;
    uint8_t  channels;
    uint8_t  volume;
    uint32_t bit_rate;
    uint32_t frame_size;
#ifdef USE_HYBRID_MODE
    uint8_t  audio_route;
#endif /* USE_HYBRID_MODE */
} bt_audio_config_t;

typedef struct
{
    wiced_codec_interface_t*    wiced_sbc_if;
    wiced_queue_t         queue;
    wiced_event_flags_t  events;
} bt_audio_decoder_context_t;

typedef struct
{
    wiced_event_flags_t            events;
    wiced_queue_t                  queue;

    bt_audio_device_state_t        state;
    wiced_audio_session_ref        bluetooth_audio_session_handle;
    bt_audio_config_t              bluetooth_audio_config;
    uint32_t                       single_audio_period_duration;
    uint32_t                       queue_max_entry_count;
    uint32_t                       queue_threshold_count;
    uint32_t                       overrun_count;                  /* Audio Over-run count                     */
    uint32_t                       underrun_count;                 /* Audio engine under-run count             */
    uint32_t                       buffer_unavail_count;           /* Audio engine buffer unavailability count */
    uint32_t                       silence_insertion_count;        /* Silence insertion count                  */
    wiced_semaphore_t              wait_for_cmd_completion;
#ifdef USE_AUDIO_PLL
    audio_pll_tuner_ref            pll_tuner;
    uint32_t                       pll_tuner_period_counter;
    audio_pll_tuner_init_params_t  pll_tuner_init_params;
    audio_pll_tuner_start_params_t pll_tuner_start_params;
    uint32_t                       pcm_packet_length;
    uint32_t                       audio_buffer_max_duration_msecs;
    uint8_t                        audio_buffer_target_percent;
    uint8_t                        audio_buffer_threshold_percent;
#endif /* USE_AUDIO_PLL */
} bt_audio_context_t;

/*****************************************************************************
**  external function declarations
*****************************************************************************/

#ifdef USE_AUDIO_PLL
int audio_pll_console_command( int argc, char* argv[] );
#endif /* USE_AUDIO_PLL */

wiced_result_t  bt_audio_init_player( void );
wiced_result_t bt_audio_configure_player( bt_audio_config_t* p_audio_config );
wiced_result_t bt_audio_decoder_context_init( void );
wiced_result_t bt_audio_reset_decoder_config( void );
#ifdef USE_HYBRID_MODE
wiced_result_t bt_audio_decoder_set_route ( uint8_t route );
#endif

void bt_audio_decoder_task( uint32_t arg );
wiced_result_t bt_audio_configure_decoder(wiced_bt_a2dp_codec_info_t* codec_config);
wiced_result_t bt_audio_write_to_decoder_queue(bt_audio_codec_data_t* audio);
void bt_audio_write_to_player_buffer( bt_audio_codec_data_t* pcm );
wiced_result_t bt_audio_stop_player( void );
wiced_result_t bt_audio_update_player_volume( uint8_t level );
void bt_audio_player_task(uint32_t args);

#ifdef __cplusplus
} /* extern "C" */
#endif
