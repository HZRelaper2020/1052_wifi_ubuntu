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
* \file cy_smif_memconfig.c
* \version 1.0
*
* \brief
* Provides definitions of the SMIF-driver memory configuration.
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

#include "cy_smif_memconfig.h"

cy_stc_smif_mem_cmd_t S25FL512S_0_readCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0xEBU,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_QUAD,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0x01U,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_QUAD,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 4U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_QUAD
};

cy_stc_smif_mem_cmd_t S25FL512S_0_writeEnCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x06U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 0U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_SINGLE
};

cy_stc_smif_mem_cmd_t S25FL512S_0_writeDisCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x04U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 0U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_SINGLE
};

cy_stc_smif_mem_cmd_t S25FL512S_0_eraseCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0xD8U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 0U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_SINGLE
};

cy_stc_smif_mem_cmd_t S25FL512S_0_chipEraseCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x60U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 0U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_SINGLE
};

cy_stc_smif_mem_cmd_t S25FL512S_0_programCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x38U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_QUAD,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 0U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_QUAD
};

cy_stc_smif_mem_cmd_t S25FL512S_0_readStsRegQeCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x35U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 0U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_SINGLE
};

cy_stc_smif_mem_cmd_t S25FL512S_0_readStsRegWipCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x05U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 0U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_SINGLE
};

cy_stc_smif_mem_cmd_t S25FL512S_0_writeStsRegQeCmd =
{
    /* The 8-bit command. 1 x I/O read command. */
    .command = 0x01U,
    /* The width of the command transfer. */
    .cmdWidth = CY_SMIF_WIDTH_SINGLE,
    /* The width of the address transfer. */
    .addrWidth = CY_SMIF_WIDTH_SINGLE,
    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode = 0xFFFFFFFFU,
    /* The width of the mode command transfer. */
    .modeWidth = CY_SMIF_WIDTH_SINGLE,
    /* The number of dummy cycles. A zero value suggests no dummy cycles. */
    .dummyCycles = 0U,
    /* The width of the data transfer. */
    .dataWidth = CY_SMIF_WIDTH_SINGLE
};

cy_stc_smif_mem_device_cfg_t deviceCfg_S25FL512S_0 =
{
    /* Specifies the number of address bytes used by the memory slave device. */
    .numOfAddrBytes = 0x03U,
    /* The size of the memory. */
    .memSize = 0x04000000U,
    /* Specifies the Read command. */
    .readCmd = &S25FL512S_0_readCmd,
    /* Specifies the Write Enable command. */
    .writeEnCmd = &S25FL512S_0_writeEnCmd,
    /* Specifies the Write Disable command. */
    .writeDisCmd = &S25FL512S_0_writeDisCmd,
    /* Specifies the Erase command. */
    .eraseCmd = &S25FL512S_0_eraseCmd,
    /* Specifies the sector size of each erase. */
    .eraseSize = 0x00040000U,
    /* Specifies the Chip Erase command. */
    .chipEraseCmd = &S25FL512S_0_chipEraseCmd,
    /* Specifies the Program command. */
    .programCmd = &S25FL512S_0_programCmd,
    /* Specifies the page size for programming. */
    .programSize = 0x00000200U,
    /* Specifies the command to read the QE-containing status register. */
    .readStsRegQeCmd = &S25FL512S_0_readStsRegQeCmd,
    /* Specifies the command to read the WIP-containing status register. */
    .readStsRegWipCmd = &S25FL512S_0_readStsRegWipCmd,
    /* Specifies the command to write into the QE-containing status register. */
    .writeStsRegQeCmd = &S25FL512S_0_writeStsRegQeCmd,
    /* The mask for the status register. */
    .stsRegBusyMask = 0x01U,
    /* The mask for the status register. */
    .stsRegQuadEnableMask = 0x02U,
    /* The max time for the erase type-1 cycle-time in ms. */
    .eraseTime = 520U,
    /* The max time for the chip-erase cycle-time in ms. */
    .chipEraseTime = 134000U,
    /* The max time for the page-program cycle-time in us. */
    .programTime = 340U
};

const cy_stc_smif_mem_config_t S25FL512S_SlaveSlot_0 =
{
    /* Determines the slot number where the memory device is placed. */
    .slaveSelect = CY_SMIF_SLAVE_SELECT_0,
    /* Flags. */
    .flags = CY_SMIF_FLAG_MEMORY_MAPPED | CY_SMIF_FLAG_WR_EN,
    /* The data-line selection options for a slave device. */
    .dataSelect = CY_SMIF_DATA_SEL0,
    /* The base address the memory slave is mapped to in the PSoC memory map.
    Valid when the memory-mapped mode is enabled. */
    .baseAddress = 0x18000000U,
    /* The size allocated in the PSoC memory map, for the memory slave device.
    The size is allocated from the base address. Valid when the memory mapped mode is enabled. */
    .memMappedSize = 0x10000U,
    /* If this memory device is one of the devices in the dual quad SPI configuration.
    Valid when the memory mapped mode is enabled. */
    .dualQuadSlots = 0,
    /* The configuration of the device. */
    .deviceCfg = &deviceCfg_S25FL512S_0
};

const cy_stc_smif_mem_config_t* smifMemConfigs[] = {
   &S25FL512S_SlaveSlot_0
};

const cy_stc_smif_block_config_t smifBlockConfig =
{
    /* The number of SMIF memories defined. */
    .memCount = CY_SMIF_DEVICE_NUM,
    /* The pointer to the array of memory config structures of size memCount. */
    .memConfig = (cy_stc_smif_mem_config_t**)smifMemConfigs,
    /* The version of the SMIF driver. */
    .majorVersion = CY_SMIF_DRV_VERSION_MAJOR,
    /* The version of the SMIF driver. */
    .minorVersion = CY_SMIF_DRV_VERSION_MINOR
};


