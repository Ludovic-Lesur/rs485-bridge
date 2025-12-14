#
# CMakeLists.txt
#
#  Created on: 09 dec. 2025
#      Author: Ludo
#

# Minimum CMake version.
cmake_minimum_required(VERSION 3.23)

# Project creation.
project(rs485-bridge)
add_executable(${PROJECT_NAME})

# Use object build mode in all sub-directories.
set(BUILD_MODE "OBJECT")

# Macro to convert options to variables.
macro(add_compilation_flag FLAG_NAME FLAG_DESCRIPTION FLAG_DEFAULT_VALUE)
    option(FLAG_NAME FLAG_DESCRIPTION FLAG_DEFAULT_VALUE)
    set(${FLAG_NAME} ${FLAG_DEFAULT_VALUE} CACHE STRING ${FLAG_DESCRIPTION})
    list(APPEND COMPILATION_FLAGS_LIST ${FLAG_NAME})
endmacro()

# Hardware compilation flags.
set(RS485_BRIDGE_BOARD "NONE" CACHE STRING "Board selection")
set(RS485_BRIDGE_HW_VERSION "NONE" CACHE STRING "Hardware version")

# Software compilation flags.
add_compilation_flag(RS485_BRIDGE_MODE_LOW_BAUD_RATE "Enable low baud-rate mode" ON)
add_compilation_flag(RS485_BRIDGE_ENABLE_UNA_AT "Enable UNA-AT interface" ON)
add_compilation_flag(RS485_BRIDGE_ENABLE_UNA_R4S8CR "Enable UNA-R4S8CR interface" ON)

# Hardware specific settings.
if(RS485_BRIDGE_BOARD STREQUAL "DIM")
    # DIM HW1.0.
    if(RS485_BRIDGE_HW_VERSION STREQUAL "HW1_0")   
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m0plus -O1")
        set(RS485_BRIDGE_MCU "stm32l0xx")
        set(PROJECT_LINKER_SCRIPT "stm32l031x6xx.ld")
    # DIM HW1.1.
    elseif(RS485_BRIDGE_HW_VERSION STREQUAL "HW1_1")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m0plus -O1")
        set(RS485_BRIDGE_MCU "stm32l0xx")
        set(PROJECT_LINKER_SCRIPT "stm32l031x6xx.ld")
    # Unknown version.
    else()
        message(FATAL_ERROR "Invalid hardware version.")
    endif()
elseif(RS485_BRIDGE_BOARD STREQUAL "RS485_BRIDGE")
    # RS485-BRIDGE HW1.0.
    if(RS485_BRIDGE_HW_VERSION STREQUAL "HW1_0")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4 -O0")
        set(RS485_BRIDGE_MCU "stm32g4xx")
        set(PROJECT_LINKER_SCRIPT "stm32g441xbxx.ld")
    # Unknown version.
    else()
        message(FATAL_ERROR "Invalid hardware version.")
    endif()
else()
    message(FATAL_ERROR "Invalid board.")
endif()
set(PROJECT_LINKER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/drivers/device/${RS485_BRIDGE_MCU}-device/linker")

# Add hardware compilation flags.
add_compile_definitions(
    __RS485_BRIDGE_FLAGS_H__
    ${RS485_BRIDGE_BOARD}
    ${RS485_BRIDGE_HW_VERSION}
)

# Add software compilation flags.
foreach(FLAG ${COMPILATION_FLAGS_LIST})
    # Macros only defined.
    if(${FLAG} STREQUAL ON)
        add_compile_definitions(${FLAG})
    endif()
    # Macros with value.
    if((NOT ${FLAG} STREQUAL OFF) AND (NOT ${FLAG} STREQUAL ON))
        set(${FLAG} "(${${FLAG}})")
        add_compile_definitions(${FLAG}=${${FLAG}}) 
    endif()
endforeach()

# Add project sources files.
target_sources(${PROJECT_NAME}
    PRIVATE
        drivers/peripherals/src/mcu_mapping.c
        drivers/mac/src/lmac_hw.c
        drivers/components/src/dip_switch.c
        drivers/components/src/led.c
        drivers/components/src/r4s8cr_hw.c
        drivers/utils/src/terminal_hw.c
        middleware/analog/src/analog.c
        middleware/cli/src/cli.c
        middleware/analog/src/analog.c
        middleware/node/src/node.c
        middleware/node/src/una_at_hw.c
        middleware/power/src/power.c
        application/src/main.c
)

# Project include folders.
include_directories(${PROJECT_NAME}
    PRIVATE
        drivers/device/inc
        drivers/registers/inc
        drivers/registers/${RS485_BRIDGE_MCU}-registers/inc
        drivers/peripherals/inc
        drivers/peripherals/${RS485_BRIDGE_MCU}-drivers/inc
        drivers/components/inc
        drivers/components/r4s8cr-driver/inc
        drivers/utils/inc
        drivers/utils/embedded-utils/inc
        drivers/mac/inc
        drivers/mac/lmac-driver/inc
        middleware/analog/inc
        middleware/cli/inc
        middleware/power/inc
        middleware/node/inc
        middleware/node/una-lib/inc
        middleware/node/una-at/inc
        middleware/node/una-r4s8cr/inc
        application/inc
)

# Add submodules sources files.
add_subdirectory(drivers/device/${RS485_BRIDGE_MCU}-device)
add_subdirectory(drivers/peripherals/${RS485_BRIDGE_MCU}-drivers)
add_subdirectory(drivers/components/r4s8cr-driver)
add_subdirectory(drivers/utils/embedded-utils)
add_subdirectory(drivers/mac/lmac-driver)
add_subdirectory(middleware/node/una-lib)
add_subdirectory(middleware/node/una-at)
add_subdirectory(middleware/node/una-r4s8cr)

# Link libraries.
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${RS485_BRIDGE_MCU}-device
        ${RS485_BRIDGE_MCU}-drivers
        embedded-utils
        lmac-driver
        r4s8cr-driver
        una-lib
        una-at
        una-r4s8cr
        c_nano
        nosys
        gcc
)

# Linker and artifact.
include(script/cmake-arm-none-eabi/linker.cmake)
include(script/cmake-arm-none-eabi/artifact.cmake)
