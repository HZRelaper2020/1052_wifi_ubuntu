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

/*****************************************************************************
**  constants
*****************************************************************************/

#define BT_AUDIO_DEFAULT_VOLUME  63 /* mid of 0-127 */
#define BT_AUDIO_VOLUME_STEP        8
#define BT_AUDIO_VOLUME_MAX         127
#define BT_AUDIO_VOLUME_MIN          0

#define BT_AUDIO_DEFAULT_PERIOD_SIZE        ( 1024 )
#define NUM_USECONDS_IN_SECONDS                     (1000*1000)
#define NUM_MSECONDS_IN_SECONDS                     (1000)

#define BT_AUDIO_BUFFER_SIZE                ( 4*BT_AUDIO_DEFAULT_PERIOD_SIZE )
#define BT_AUDIO_QUEUE_MAX_SIZE  (28)

#define BT_AUDIO_QUEUE_THRESHOLD (10)


/*****************************************************************************
 ** enumerations
 *****************************************************************************/

#define  BT_AUDIO_EVENT_START_LOOP                  0x00000001
#define  BT_AUDIO_EVENT_STOP_LOOP                   0x00000002
#define BT_AUDIO_EVENT_READ_RECORDER_DATA   0x00000004
#define  BT_AUDIO_EVENT_ALL                               0xFFFFFFFF


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

typedef enum
{
    BT_AUDIO_DEVICE_PLAYER = 0x1,
    BT_AUDIO_DEVICE_RECORDER = 0x2
}bt_audio_device_type_t;

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
} bt_audio_config_t;

typedef struct
{
    wiced_codec_interface_t*    wiced_sbc_if;
    wiced_queue_t         queue;
    wiced_event_flags_t  events;
} bt_audio_decoder_context_t;

typedef struct
{
    wiced_event_flags_t  events;
    wiced_queue_t        queue;

    bt_audio_device_state_t state;
    //uint8_t                   now_playing;
    wiced_audio_session_ref   bluetooth_audio_session_handle;
    bt_audio_config_t         bluetooth_audio_config;
    uint32_t                  audio_buffer_weight;
    uint32_t                  audio_buffer_weight_high_water_mark;
    uint32_t                  overrun_count;      /* Number of times player did not have data to play */
    uint32_t                  cback_count;
    wiced_semaphore_t         wait_for_cmd_completion;
} bt_audio_context_t;

/*****************************************************************************
**  external function declarations
*****************************************************************************/
wiced_result_t bt_audio_decoder_context_init( void );
wiced_result_t bt_audio_reset_decoder_config( void );
void bt_audio_player_task(uint32_t args);
void bt_audio_decoder_task( uint32_t arg );
void bta_wiced_audio_recorder_task(uint32_t context);

wiced_result_t bt_audio_configure_decoder(wiced_bt_a2dp_codec_info_t* codec_config);
wiced_result_t bt_audio_write_to_decoder_queue(bt_audio_codec_data_t* audio);
void bt_audio_write_to_player_buffer( bt_audio_codec_data_t* pcm );

wiced_bool_t is_bt_audio_device_initialized( bt_audio_device_type_t device_type );
wiced_bool_t is_bt_audio_device_prepared( bt_audio_device_type_t device_type );
wiced_result_t  bt_audio_init_device( bt_audio_device_type_t device_type );
void bt_audio_start_loop(bt_audio_device_type_t device_type);
wiced_result_t bt_audio_end_loop( uint32_t device_type );
wiced_result_t bt_audio_start( bt_audio_device_type_t device_type );
void bt_audio_stop( bt_audio_device_type_t device_type );
wiced_result_t  bt_audio_deinit_device( bt_audio_device_type_t device_type );
uint16_t bt_audio_get_recorder_buffer_count( void );

wiced_result_t bt_audio_update_player_volume( uint8_t level );
void bt_audio_hfp_send_sco_data(bt_audio_codec_data_t* pcm);


#ifdef __cplusplus
extern "C"
{
#endif


#ifdef __cplusplus
}
#endif
