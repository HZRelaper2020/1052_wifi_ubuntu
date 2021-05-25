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

/*******************************************************************************
* \file cy_smif_memconfig.h
* \version 1.0
*
* \brief
* Provides declarations of the SMIF-driver memory configuration.
*
* Note: This is an auto generated file. Do not modify it.
*
********************************************************************************
* \copyright
* Copyright 2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/
#ifndef CY_SMIF_MEMCONFIG_H
#define CY_SMIF_MEMCONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "smif/cy_smif_memslot.h"

#define CY_SMIF_DEVICE_NUM 1

extern cy_stc_smif_mem_cmd_t S25FL512S_0_readCmd;
extern cy_stc_smif_mem_cmd_t S25FL512S_0_writeEnCmd;
extern cy_stc_smif_mem_cmd_t S25FL512S_0_writeDisCmd;
extern cy_stc_smif_mem_cmd_t S25FL512S_0_eraseCmd;
extern cy_stc_smif_mem_cmd_t S25FL512S_0_chipEraseCmd;
extern cy_stc_smif_mem_cmd_t S25FL512S_0_programCmd;
extern cy_stc_smif_mem_cmd_t S25FL512S_0_readStsRegQeCmd;
extern cy_stc_smif_mem_cmd_t S25FL512S_0_readStsRegWipCmd;
extern cy_stc_smif_mem_cmd_t S25FL512S_0_writeStsRegQeCmd;

extern cy_stc_smif_mem_device_cfg_t S25FL512S_0_DeviceCfg;

extern const cy_stc_smif_mem_config_t S25FL512S_0;

extern const cy_stc_smif_mem_config_t* smifMemConfigs[CY_SMIF_DEVICE_NUM];

extern const cy_stc_smif_block_config_t smifBlockConfig;

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /*CY_SMIF_MEMCONFIG_H*/
