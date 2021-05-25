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

NAME := Platform_CYW9MCU7X9N364

WLAN_CHIP            := 43364
WLAN_CHIP_REVISION   := A1
WLAN_CHIP_FAMILY     := 4343x
HOST_MCU_FAMILY      := BCM920739
HOST_MCU_VARIANT     := BCM920739B0
HOST_MCU_PART_NUMBER := BCM920739B0
BT_CHIP              := 20739
BT_CHIP_REVISION     := B1
BT_CHIP_FAMILY       := 2073x
APPS_CHIP_REVISION   := $(BT_CHIP_REVISION)
#BT_CHIP_XTAL_FREQUENCY := 40MHz
#BT_MODE              ?= HCI

PLATFORM_SUPPORTS_BUTTONS := 1
PLATFORM_SUPPORTS_FILESYSTEM := 1
PLATFORM_SUPPORTS_OC_FLASH := 1

ifeq ($(PLATFORM_SUPPORTS_FILESYSTEM),1)
# platform supports resource generic file system
PLATFORM_SUPPORTS_RESSOURCE_GENFS := 1
endif
#RTOS is linked from firmware patch
WICED_OSTYPE := ROM
# Netx linked from firmware patch
WICED_NETTYPE := ROM
# Crypto is linked with ROM security code
WICED_SECURITY := ROM

GLOBAL_DEFINES          +=  WICED_SECURITY_ROM

GLOBAL_DEFINES          +=  WICED_NETTYPE_ROM
GLOBAL_DEFINES          +=  BT_CHIP_REVISION_$(BT_CHIP_REVISION)

GLOBAL_DEFINES          +=  ENABLE_BT_COEX

PLATFORM_CONFIGS := BCM20739$(BT_CHIP_REVISION).cgs

#PLATFORM_ENABLE_CLIENT_CONTROL := 1
# Powersave Macros
#GLOBAL_DEFINES += WICED_DISABLE_MCU_POWERSAVE

ifeq (1, $(PLATFORM_ENABLE_CLIENT_CONTROL))
GLOBAL_DEFINES += WICED_DISABLE_MCU_POWERSAVE
endif

#OTA2_SUPPORT := 1
ifndef BUS
BUS := SPI
endif

VALID_BUSES := SPI

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
ifneq (1, $(OTA2_SUPPORT))
RESOURCES_LOCATION ?= RESOURCES_IN_WICEDFS
endif
ifeq (1, $(OTA2_SUPPORT))
GLOBAL_DEFINES	+= PLATFORM_OTA2_SOFTAP_NOT_SUPPORTED
PLATFORM_SUPPORTS_OTA2_UPDATE_FS	:= 1
GLOBAL_DEFINES+= WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
endif

ifeq ($(RESOURCES_LOCATION), RESOURCES_IN_DIRECT_RESOURCES)
INTERNAL_MEMORY_RESOURCES = $(ALL_RESOURCES)
GLOBAL_DEFINES += WWD_DIRECT_RESOURCES
endif

# Global includes
GLOBAL_INCLUDES  := .
GLOBAL_INCLUDES  += $(SOURCE_ROOT)libraries/inputs/gpio_button

GLOBAL_DEFINES += WPRINT_BT_APP_INFO=WPRINT_APP_INFO
# Global defines
GLOBAL_DEFINES	+= OCF_WIFI_FIRMWARE=YES

#GLOBAL_DEFINES += HSE_VALUE=26000000
GLOBAL_DEFINES += $$(if $$(NO_CRLF_STDIO_REPLACEMENT),,CRLF_STDIO_REPLACEMENT)

# Components
#$(NAME)_COMPONENTS += drivers/spi_flash \
#                      inputs/gpio_button

$(NAME)_COMPONENTS += inputs/gpio_button
#$(NAME)_COMPONENTS += io_manager
#$(NAME)_COMPONENTS += hal/led_gpio
#$(NAME)_COMPONENTS += hal/button_gpio
#$(NAME)_COMPONENTS += sensor_fw/sensor_manager
#$(NAME)_COMPONENTS += sensor_fw/sensor_fal
#$(NAME)_COMPONENTS += sensor_fw/sensor_hal

# Source files
$(NAME)_SOURCES := platform.c

# WICED APPS
# APP0 and FILESYSTEM_IMAGE are reserved main app and resources file system
# FR_APP := resources/sflash/snip_ota_fr-BCM943362WCD6.stripped.elf
# DCT_IMAGE :=
# OTA_APP :=
# FILESYSTEM_IMAGE :=
# WIFI_FIRMWARE :=
# APP0 :=
# APP1 :=
# APP2 :=
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

