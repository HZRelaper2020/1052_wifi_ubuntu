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
 * Defines internal configuration of the CY8CKIT_062 board
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *  MCU Constants and Options
 ******************************************************/

/* CPU clock : 100 MHz */
/* CLK_HF[0]:CLK_FAST is setup in Cy_SystemInit() in peripheral library */
#define CPU_CLOCK_HZ         ( 100000000 )

/******************************************************
 *  Wi-Fi Options
 ******************************************************/

/* GPIO pins are used to bootstrap WiFi to SDIO or gSPI mode */
//#define WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP_1    /* WiFi chip is configured for SDIO mode in CY8CKIT_062 board */

/*  WiFi GPIO pin used for out-of-band interrupt */
//#define WICED_WIFI_OOB_IRQ_GPIO_PIN  ( 0 )    /* WiFi chip is not configured for GPIO OOB IRQ in CY8CKIT_062 board */

/* WiFi power pin is present */
//#define WICED_USE_WIFI_POWER_PIN    /* WiFi chip is powered by external power source in CY8CKIT_062 board */

/* WiFi reset pin is present */
#define WICED_USE_WIFI_RESET_PIN

/* WiFi power pin is active high */
//#define WICED_USE_WIFI_POWER_PIN_ACTIVE_HIGH    /* WiFi chip is powered by external power source in CY8CKIT_062 board */

/*  WLAN Powersave Clock Source
 *  The WLAN sleep clock can be driven from one of two sources:
 *  1. MCO (MCU Clock Output) - default
 *  2. WLAN 32K internal oscillator (30% inaccuracy)
 *     - Comment the following directive : WICED_USE_WIFI_32K_CLOCK_MCO
 */

//#define WICED_USE_WIFI_32K_CLOCK_MCO  /* CY8CKIT_062 board supplies the 32K clock */

/*  OTA */
//#define PLATFORM_HAS_OTA /* OTA support is not enabled for CY8CKIT_062 board */
/*  WICED Resources uses a filesystem */
#ifndef WWD_DIRECT_RESOURCES
#define USES_RESOURCE_FILESYSTEM
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
