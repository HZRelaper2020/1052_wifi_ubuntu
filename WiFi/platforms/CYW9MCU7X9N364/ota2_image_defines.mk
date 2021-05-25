#
# Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 # Cypress Semiconductor Corporation. All Rights Reserved.
 # This software, including source code, documentation and related
 # materials ("Software"), is owned by Cypress Semiconductor Corporation
 # or one of its subsidiaries ("Cypress") and is protected by and subject to
 # worldwide patent protection (United States and foreign),
 # United States copyright laws and international treaty provisions.
 # Therefore, you may use this Software only as provided in the license
 # agreement accompanying the software package from which you
 # obtained this Software ("EULA").
 # If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 # non-transferable license to copy, modify, and compile the Software
 # source code solely for use in connection with Cypress's
 # integrated circuit products. Any reproduction, modification, translation,
 # compilation, or representation of this Software except as specified
 # above is prohibited without the express written permission of Cypress.
 #
 # Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 # EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 # reserves the right to make changes to the Software without notice. Cypress
 # does not assume any liability arising out of the application or use of the
 # Software or any product or circuit described in the Software. Cypress does
 # not authorize its products for use in any products where a malfunction or
 # failure of the Cypress product may reasonably be expected to result in
 # significant property damage, injury or death ("High Risk Product"). By
 # including Cypress's product in a High Risk Product, the manufacturer
 # of such system or application assumes all risk of such use and in doing
 # so agrees to indemnify Cypress against all liability.
#
#
# Constant sector size FLASH
#

SECTOR_SIZE 						 := 4096

#
#   Sizes are on sector boundaries for this platform
#
#   +---------------------------------------+
#   | Boot loader Area1						     | 68k  Size of build/waf.ota2_bootloader-<platform>/binary/waf.ota2_bootloader-<platform>.bin
#   +---------------------------------------+
#   | Boot loader Area2						     | 68k  Size of build/waf.ota2_bootloader-<platform>/binary/waf.ota2_bootloader-<platform>.bin
#   +---------------------------------------+
#   | Factory Reset APP				           | 24k  Size of build/waf.ota2_factory_reset-<platforn>/binary/waf.ota2_factory_reset-<platform>.bin
#   +---------------------------------------+
#   | OTA2 Failsafe App						     | 24k  Size of build/waf.ota2_failsafe-<platform>/binary/waf.ota2_failsafe-<platform>.bin
#   +---------------------------------------+
#   | DCT Save area (when updating)			     | 16k  (same size as normal DCT)
#   +--                                   ---------------+
#   | Application Lookup Table (LUT)		            |  4k
#   +--                                   ---------------+
#   | DCT Copy 1                      		            | 16k   DCT 1 & 2 must be contiguous
#   +--                                   ---------------+
#   | DCT Copy 2                      		            | 16k
#   +--                                   ---------------+
#   | OTA2 Extractor Application      		            | 24k  Size of build/snip.ota2_extract-<platform>/binary/snip.ota2_extract-<platform>.bin
#   +--                                   ---------------+
#   | File system                      		                  | 388k
#   +--                                   ---------------+
#   | Current Application      				     | ??? --\
#   +--                                   --+        >-- total 156K
#   | Expansion area 		  				     | ??? --/
#   +---------------------------------------+
#   | OTA2 Staging area (downloaded image)  | 204K  Adjust this size to max. expectation of update build/<your_application>-<platform>/OTA2_image_file.bin
#	|									     |      (Will probably be larger than the Factory Reset Image in Update builds)
#   +---------------------------------------+
#
#  LAST KNOWN GOOD Not supported

OTA2_IMAGE_FLASH_BASE_ADDRESS		 := 0x00500000

# OTA2 FLASH Area                        Location     Size
OTA2_IMAGE_FLASH_BASE                := 0x00000000
OTA2_IMAGE_DO_NOT_WRITE_AREA_START	 := 0x00004000  #########  DO NOT WRITE TO THE FLASH BELOW THIS AFTER MANUFACTURE  ########
OTA2_IMAGE_BOOTLOADER_START          := 0x00004000	#  68k  0x00011000
OTA2_IMAGE_BOOTLOADER2_START          := 0x00015000		#  68k  0x00011000

# DO NOT CHANGE THESE LOCATIONS AFTER A PRODUCT HAS SHIPPED
# DO NOT CHANGE THESE FOR AN UPDATE TO ANOTHER SDK
# (The Bootloader needs these to always be the same - bootloader does not get updated and would not know of changes between SDKs)
# New for this SDK, use new layout
# OTA2 Failsafe Application Storage
OTA2_IMAGE_FR_APP_AREA_BASE	 := 0x00026000  # 24k  0x00006000
OTA2_IMAGE_FAILSAFE_APP_AREA_BASE	 := 0x0002C000  # 24k  0x00006000
OTA2_IMAGE_FACTORY_RESET_AREA_BASE   := 0x005FFFFF	# not valid
OTA2_IMAGE_DO_NOT_WRITE_AREA_END	 := 0x00032000  #########  DO NOT WRITE TO THE FLASH ABOVE THIS AFTER MANUFACTURE  ########

OTA2_IMAGE_APP_DCT_SAVE_AREA_BASE    := 0x00032000  #  16k  0x00004000
OTA2_IMAGE_CURR_LUT_AREA_BASE        := 0x00036000  #   4k  0x00001000
# Note: Any change in this value should be updated in DCT_OCF_START_ADDR of 20739B1.mk.
OTA2_IMAGE_CURR_DCT_1_AREA_BASE      := 0x00037000	#  16k  0x00004000
OTA2_IMAGE_CURR_DCT_2_AREA_BASE      := 0x0003B000	#  16k  0x00004000
# DO NOT CHANGE THE ABOVE LOCATIONS AFTER A PRODUCT HAS SHIPPED
# DO NOT CHANGE THE ABOVE LOCATIONS FOR AN UPDATE TO ANOTHER SDK

# OTA2 Extraction Application Storage (start on 4k boundary)
OTA2_IMAGE_CURR_OTA_APP_AREA_BASE    := 0x0003F000	# 24k 0x0006000

# File system (start on 4k boundary)
OTA2_IMAGE_CURR_FS_AREA_BASE         := 0x00045000	# 4k  0x0001000 (fs header start)
# Note: Any change in this value should be updated in FILESYSTEM_OCF_START_ADDR of 20739B1.mk
#         and WICED_FILESYSTEM_OCF_START of BCM920739.mk file.
OTA2_IMAGE_FS_DATA_AREA_BASE         := 0x00046000	# 384k  0x00060000 (fs data start)
# Note: Any change in this value should be updated in FILSYSTEM_OCF_MAX_SIZE of 20739B1.mk
#         and WICED_FILESYSTEM_OCF_SIZE of BCM920739.mk file.
OTA2_IMAGE_FS_DATA_AREA_SIZE          := 0x00060000 # (fs data size 384K)
OTA2_IMAGE_FS_AREA_SIZE         := 0x00061000 # total FS area size (header + data)

#App0 - Application (start on 4k boundary)
OTA2_IMAGE_CURR_APP0_AREA_BASE       := 0x000A6000  # 156k	0x00028000. for XIP 56k
ifeq (1, $(XIP_SUPPORT))
OTA2_IMAGE_APP0_XIP_AREA_BASE          := 0x000B4000 # 100k
endif #ifeq (1, $(XIP_SUPPORT))

# unused area is after APP0 and before The Staging Area

# Staging Area for OTA2 updates
# DO NOT CHANGE THIS LOCATION AFTER A PRODUCT HAS SHIPPED
# DO NOT CHANGE THIS LOCATION FOR AN UPDATE TO ANOTHER SDK
# (The Bootloader needs this to always be the same - bootloader does not get updated and would not know of changes between SDKs)
OTA2_IMAGE_STAGING_AREA_BASE         := 0x000CD000	#   204k  0x00033000 (156+28+16+4)
OTA2_IMAGE_STAGING_AREA_SIZE         := 0x00033000


