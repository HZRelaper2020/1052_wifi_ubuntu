set(armgcc 1)
set(WifiSDK WiFi)

if (${armgcc})
set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "arm-none-eabi-g++")

set(CMAKE_EXE_LINKER_FLAGS "--specs=nosys.specs") 
endif()

cmake_minimum_required(VERSION 3.16)

project(wifi)

if (${armgcc})
add_compile_options(
        -mlittle-endian
        -mfloat-abi=softfp
        -mfpu=fpv5-d16
        -mcpu=cortex-m7
	-mthumb
	#	-O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3
)
endif()

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

add_definitions(-DCPU_MIMXRT1052CVL5B
-DPRINTF_FLOAT_ENABLE=1
-DSCANF_FLOAT_ENABLE=1
-DPRINTF_ADVANCED_ENABLE=1
-DSCANF_ADVANCED_ENABLE=1
-DFSL_RTOS_FREE_RTOS
-DUSE_RTOS=1
-DWICED_DISABLE_BOOTLOADER
-DWWD_BUS_SDIO_USE_WLAN_SDIO_PULLUPS
-DNO_WICED_API

-DDEBUG

-DSERIAL_PORT_TYPE_UART=1
-DSDIO_ENABLED
-DWPRINT_ENABLE_ERROR
-DDHCP_DEBUG)


set(FreeRTOS_Version 10.3.0)
set(use_library 2)

if (${use_library} EQUAL 1)
include_directories(
libraries/CMSIS/Include
libraries/drivers
libraries/utilities
libraries/utilities/str
libraries/utilities/log
libraries/utilities/io
libraries/xip
libraries
user
user/bsp
user/bsp/sdio/inc
user/bsp/sdio/port
)

elseif (${use_library} EQUAL 2)
include_directories(
libraries2/lib/CMSIS
libraries2/lib/device
libraries2/lib/drivers
libraries2/lib/utilities
libraries2/lib/sdmmc/host
libraries2/lib/sdmmc/inc
libraries2/lib/sdmmc/osa
libraries2/lib/sdmmc/src
libraries2/lib/component/lists
libraries2/lib/component/osa
libraries2/lib/component/uart
libraries2/lib/component/serial_manager
)
endif()

set(network lwip)

if (${network} STREQUAL lwip)
include_directories(
${WifiSDK}/WICED/network/LwIP/ver2.1.2
${WifiSDK}/WICED/network/LwIP/ver2.1.2
${WifiSDK}/WICED/network/LwIP/ver2.1.2/lwip/include
	)

list(APPEND sources
${WifiSDK}/WICED/network/LwIP/WWD/wwd_buffer.c
)

endif()	
		
include_directories(
${WifiSDK}/WICED/RTOS/FreeRTOS/ver${FreeRTOS_Version}/Source
${WifiSDK}/WICED/RTOS/FreeRTOS/ver${FreeRTOS_Version}/Source/include
#${WifiSDK}/WICED/RTOS/FreeRTOS/ver${FreeRTOS_Version}/Source/portable/GCC/ARM_CM7/r0p1
${WifiSDK}/WICED/RTOS/FreeRTOS/ver${FreeRTOS_Version}/Source/portable/GCC/ARM_CM4F
${WifiSDK}/include
${WifiSDK}/libraries/utilities/linked_list
${WifiSDK}/libraries/utilities/TLV
${WifiSDK}/resources/firmware/43362
${WifiSDK}/WICED/internal/unit
${WifiSDK}/WICED/internal
${WifiSDK}/WICED/network/LwIP/WWD
#${WifiSDK}/WICED/network/NoNS/WWD
#${WifiSDK}/WICED/network/LwIP/WWD/FreeRTOS
#${WifiSDK}/WICED/network/LwIP/WWD/FreeRTOS/arch
${WifiSDK}/WICED/platform/MCU/iMXRT10xx
${WifiSDK}/WICED/platform/include
${WifiSDK}/WICED/platform/MCU/iMXRT10xx
${WifiSDK}/WICED/platform/MCU/iMXRT10xx/peripherals
${WifiSDK}/WICED/platform/MCU/iMXRT10xx/WWD
${WifiSDK}/WICED/WWD/include
${WifiSDK}/WICED/WWD/include/network
${WifiSDK}/WICED/WWD/include/platform
${WifiSDK}/WICED/WWD/include/RTOS
${WifiSDK}/WICED/WWD
${WifiSDK}/WICED/WWD/internal/bus_protocols
${WifiSDK}/WICED/WWD/internal/bus_protocols/SDIO
${WifiSDK}/WICED/WWD/internal/chips/43362
${WifiSDK}/WICED/RTOS/FreeRTOS/WWD
${WifiSDK}/WICED/WWD/internal
${WifiSDK}/WICED/platform/GCC
${WifiSDK}/WICED/security/BESL/crypto_internal
${WifiSDK}/WICED/security/BESL/mbedtls_open/include
${WifiSDK}/WICED/security/BESL/include
${WifiSDK}/WICED/RTOS/FreeRTOS/WICED
${WifiSDK}/WICED/platform/MCU
${WifiSDK}/libraries/utilities/ring_buffer
${WifiSDK}/WICED/RTOS/FreeRTOS/WWD/ARM_CM7
${WifiSDK}/WICED/network/LwIP/WICED
${WifiSDK}/WICED/security/BESL/host/WICED
${WifiSDK}/WICED/network/LwIP/WWD/FreeRTOS/arch
${WifiSDK}/WICED/security/PostgreSQL/include
	)

list(APPEND sources
${WifiSDK}/WICED/WWD/internal/bus_protocols/SDIO/wwd_bus_protocol.c
${WifiSDK}/WICED/WWD/internal/bus_protocols/wwd_bus_common.c
${WifiSDK}/WICED/WWD/internal/chips/43362/wwd_ap.c
${WifiSDK}/WICED/WWD/internal/chips/43362/wwd_chip_specific_functions.c
${WifiSDK}/WICED/WWD/internal/wwd_ap_common.c
${WifiSDK}/WICED/WWD/internal/wwd_debug.c
${WifiSDK}/WICED/WWD/internal/wwd_eapol.c
${WifiSDK}/WICED/WWD/internal/wwd_internal.c
${WifiSDK}/WICED/WWD/internal/wwd_logging.c
${WifiSDK}/WICED/WWD/internal/wwd_management.c
${WifiSDK}/WICED/WWD/internal/wwd_rtos_interface.c
${WifiSDK}/WICED/WWD/internal/wwd_sdpcm.c
${WifiSDK}/WICED/WWD/internal/wwd_thread.c
${WifiSDK}/WICED/WWD/internal/wwd_thread_internal.c
${WifiSDK}/WICED/WWD/internal/wwd_wifi.c
${WifiSDK}/WICED/WWD/internal/wwd_wifi_chip_common.c
${WifiSDK}/WICED/WWD/internal/wwd_wifi_sleep.c
${WifiSDK}/WICED/RTOS/FreeRTOS/WWD/wwd_rtos.c
${WifiSDK}/WICED/platform/MCU/iMXRT10xx/WWD/wlan_bus_sdio.c
${WifiSDK}/WICED/platform/MCU/iMXRT10xx/WWD/wwd_platform.c
${WifiSDK}/WICED/platform/MCU/platform_resource.c
${WifiSDK}/WICED/platform/MCU/wwd_resources.c
${WifiSDK}/WICED/platform/MCU/wwd_platform_separate_mcu.c
${WifiSDK}/WICED/platform/ARM_CM7/host_cm7.c
${WifiSDK}/resources/firmware/43362/43362A2_bin.c
${WifiSDK}/libraries/utilities/TLV/tlv.c
${WifiSDK}/libraries/utilities/ring_buffer/ring_buffer.c
${WifiSDK}/WICED/internal/wiced_lib.c
${WifiSDK}/WICED/internal/wiced_crypto.c

# lwip
${WifiSDK}/WICED/network/LwIP/WWD/wwd_network.c
#${WifiSDK}/WICED/network/NoNS/WWD/wwd_buffer.c


# my code
my/scan.c
my/dhcp.c
my/tcpecho.c
)

add_library(wiced_wifi STATIC  ${sources} ${myc})

if (0)
list(APPEND myc
	my/imp_need.c
	my/main.c
)

add_executable(app my/main.c ${sources} ${myc})
endif()
