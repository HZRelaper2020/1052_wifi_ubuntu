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

NAME := Platform_BCM943362WCD8

WLAN_CHIP            := 43362
WLAN_CHIP_REVISION   := A2
WLAN_CHIP_FAMILY     := 43362
HOST_MCU_FAMILY      := SAM4S

PLATFORM_SUPPORTS_BUTTONS := 1

INTERNAL_MEMORY_RESOURCES = $(ALL_RESOURCES)

ifndef BUS
BUS := SDIO
endif

EXTRA_TARGET_MAKEFILES +=  $(MAKEFILES_PATH)/standard_platform_targets.mk

VALID_BUSES := SDIO

# For apps without wifi firmware, NO_WIFI_FIRMWARE := YES and
# resources will be in DIRECT_RESOURCE for SDIO bus
ifneq ($(NO_WIFI_FIRMWARE),)
ifeq ($(BUS),SDIO)
RESOURCES_LOCATION := RESOURCES_IN_DIRECT_RESOURCES
endif
endif

# WIFI_FIRMWARE and WIFI_FIRMWARE_CLM_BLOB are now included into resources
# RESOURCES_LOCATION default to RESOURCES_IN_WICEDFS. But can be optionally config to RESOURCES_IN_DIRECT_RESOURCES
# WARNING: Config RESOURCES_LOCATION to RESOURCES_IN_DIRECT_RESOURCES will build firmware and blob to into main application
# and may cause internal flash to overflow
RESOURCES_LOCATION ?= RESOURCES_IN_WICEDFS

ifeq ($(RESOURCES_LOCATION), RESOURCES_IN_DIRECT_RESOURCES)
INTERNAL_MEMORY_RESOURCES = $(ALL_RESOURCES)
GLOBAL_DEFINES += WWD_DIRECT_RESOURCES
endif

# Global includes
GLOBAL_INCLUDES  := .
GLOBAL_INCLUDES  += $(SOURCE_ROOT)libraries/inputs/gpio_button

GLOBAL_DEFINES  += __SAM4S16B__ \
                   ARM_MATH_CM4=true \
                   CONFIG_SYSCLK_SOURCE=SYSCLK_SRC_PLLACK \
                   CONFIG_SYSCLK_PRES=SYSCLK_PRES_1 \
                   CONFIG_PLL0_SOURCE=PLL_SRC_MAINCK_XTAL \
                   CONFIG_PLL0_MUL=41 \
                   CONFIG_PLL0_DIV=4 \
                   BOARD=USER_BOARD \
                   BOARD_FREQ_SLCK_XTAL=32768 \
                   BOARD_FREQ_SLCK_BYPASS=32768 \
                   BOARD_FREQ_MAINCK_XTAL=12000000 \
                   BOARD_FREQ_MAINCK_BYPASS=12000000 \
                   BOARD_OSC_STARTUP_US=2000 \
                   BOARD_MCK=123000000 \
                   CPU_CLOCK_HZ=123000000 \
                   PERIPHERAL_CLOCK_HZ=123000000 \
                   SDIO_4_BIT \
                   TRACE_LEVEL=0 \
                   WAIT_MODE_SUPPORT \
                   WAIT_MODE_ENTER_DELAY_CYCLES=3 \
                   WAIT_MODE_EXIT_DELAY_CYCLES=34

# Components
$(NAME)_COMPONENTS += drivers/spi_flash \
                      inputs/gpio_button

GLOBAL_DEFINES += $$(if $$(NO_CRLF_STDIO_REPLACEMENT),,CRLF_STDIO_REPLACEMENT)


# Source files
$(NAME)_SOURCES := platform.c

# WICED APPS
# APP0 and FILESYSTEM_IMAGE are reserved main app and resources file system
# FR_APP :=
# DCT_IMAGE :=
# OTA_APP :=
# FILESYSTEM_IMAGE :=
# WIFI_FIRMWARE :=
# APP0 :=
# APP1 :=
# APP2 :=

# WICED APPS LOOKUP TABLE
APPS_LUT_HEADER_LOC := 0x0000
APPS_START_SECTOR := 1

ifneq ($(APP),bootloader)
ifneq ($(MAIN_COMPONENT_PROCESSING),1)
$(info +-----------------------------------------------------------------------------------------------------+ )
$(info | IMPORTANT NOTES                                                                                     | )
$(info +-----------------------------------------------------------------------------------------------------+ )
$(info | Wi-Fi MAC Address                                                                                   | )
$(info |    The target Wi-Fi MAC address is defined in <WICED-SDK>/generated_mac_address.txt                 | )
$(info |    Ensure each target device has a unique address.                                                  | )
$(info +-----------------------------------------------------------------------------------------------------+ )
$(info | MCU & Wi-Fi Power Save                                                                              | )
$(info |    It is *critical* that applications using WICED Powersave API functions connect an accurate 32kHz | )
$(info |    reference clock to the sleep clock input pin of the WLAN chip. Please read the WICED Powersave   | )
$(info |    Application Note located in the documentation directory if you plan to use powersave features.   | )
$(info +-----------------------------------------------------------------------------------------------------+ )
endif
endif
