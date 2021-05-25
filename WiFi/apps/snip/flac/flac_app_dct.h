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
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "wiced_flac_interface.h"
#include "platform_audio.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define FLAC_BUFFERING_MS_MIN        0
#define FLAC_BUFFERING_MS_DEFAULT   50
#define FLAC_BUFFERING_MS_MAX     1000

#define FLAC_THRESHOLD_MS_MIN        0
#define FLAC_THRESHOLD_MS_DEFAULT   40
#define FLAC_THRESHOLD_MS_MAX     1000

#define FLAC_VOLUME_MIN              0
#define FLAC_VOLUME_DEFAULT         50
#define FLAC_VOLUME_MAX            100

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

typedef struct flac_app_dct_s {
    AUDIO_CHANNEL_MAP_T channel;
    int buffering_ms;
    int threshold_ms;
    int clock_enable;   /* enable AS clock */
    int volume;
    platform_audio_device_id_t audio_device_rx;
    platform_audio_device_id_t audio_device_tx;
} flac_app_dct_t;


#ifdef __cplusplus
} /* extern "C" */
#endif
