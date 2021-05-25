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

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/*
 * AP settings in this file are stored in the DCT. These
 * settings may be overwritten at manufacture when the
 * DCT is written with the final production configuration
 */

/* Mesh Network Description.
 * Currently only Open Security (WICED_SECURITY_OPEN) is accepted.
 */
#define CONFIG_AP_SSID       "MyMesh"
#define CONFIG_AP_PASSPHRASE "MESH USES ONLY OPEN"
#define CONFIG_AP_SECURITY   WICED_SECURITY_OPEN
#define CONFIG_AP_CHANNEL    48
#define CONFIG_AP_VALID      WICED_FALSE

/*
 * Gateway Configuration.
 * Second STA interface (if available) will connect to this Gateway AP when Gateway mode is enabled.
 * Standard Wiced security modes can be used on this connection.
 * Channel & Band are ignored (since STA will adopt channel of AP).
 */
#define CLIENT_AP_2_SSID       "My-Router"
#define CLIENT_AP_2_PASSPHRASE "abcd1234"
#define CLIENT_AP_2_SECURITY   WICED_SECURITY_WPA2_AES_PSK
#define CLIENT_AP_2_BSS_TYPE   WICED_BSS_TYPE_INFRASTRUCTURE
#define CLIENT_AP_2_CHANNEL    1
#define CLIENT_AP_2_BAND       WICED_802_11_BAND_2_4GHZ

/* Override default country code */
#define WICED_COUNTRY_CODE    WICED_COUNTRY_UNITED_STATES
#define WICED_COUNTRY_AGGREGATE_CODE    WICED_COUNTRY_AGGREGATE_XV_0

/* Not used for Mesh */
#define SOFT_AP_SSID         "NotUsed"
#define SOFT_AP_PASSPHRASE   "YOUR_AP_PASSPHRASE"
#define SOFT_AP_SECURITY     WICED_SECURITY_OPEN
#define SOFT_AP_CHANNEL      44

/* Not used for Mesh */
#define CLIENT_AP_SSID       "NotUsed"
#define CLIENT_AP_PASSPHRASE "YOUR_AP_PASSPHRASE"
#define CLIENT_AP_BSS_TYPE   WICED_BSS_TYPE_INFRASTRUCTURE
#define CLIENT_AP_SECURITY   WICED_SECURITY_WPA2_MIXED_PSK
#define CLIENT_AP_CHANNEL    1
#define CLIENT_AP_BAND       WICED_802_11_BAND_2_4GHZ
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

/******************************************************
 *               Function Declarations
 ******************************************************/

#ifdef __cplusplus
} /*extern "C" */
#endif
