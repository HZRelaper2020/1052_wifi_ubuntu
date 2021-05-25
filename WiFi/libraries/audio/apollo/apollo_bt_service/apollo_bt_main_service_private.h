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
 * This file provides definitions and function prototypes for Apollo BT main service
 * device
 *
 */
#pragma once

#include "wiced_bt_stack.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define BT_DEVICE_ADDRESS { 0x11, 0x22, 0x33, 0xAA, 0xBB, 0xCC }

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
 *                 Global Variables
 ******************************************************/

extern       wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[ ];
extern const uint8_t                 sdp_database[];
extern const uint16_t                wiced_bt_sdp_db_size;

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_bt_management_evt_data_t *apollo_bt_service_get_management_evt_data(void);


/**
 * Reconnect to last-connected Bluetooth A2DP audio source.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t apollo_bt_a2dp_sink_connect( void );

wiced_result_t apollo_bt_service_reconnection_timer_start(void);
wiced_result_t apollo_bt_service_reconnection_timer_stop(void);

#ifdef __cplusplus
} /* extern "C" */
#endif
