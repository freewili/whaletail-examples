cmake_minimum_required( VERSION 3.12...3.13 )

# Defaults to the Pico board (a value of 'pico'). Uncomment the following line if using an Pico W.
# set(PICO_BOARD pico_w)
set(PICO_BOARD pico)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(whaletail_10baset1l C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


# These are the definitions to select the MCP25xxFD on the CANPico and
# the binding to the RP2040 and CANPico pinouts.
add_definitions(-DORCA_WHALE_TAIL)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME} 
    main.cpp
    SparkFun_ADIN1110_Arduino_Library/src/adi_mac.c
    SparkFun_ADIN1110_Arduino_Library/src/adi_phy.c
    SparkFun_ADIN1110_Arduino_Library/src/adi_spi_generic.c
    SparkFun_ADIN1110_Arduino_Library/src/adi_spi_oa.c
    SparkFun_ADIN1110_Arduino_Library/src/adin1110.c
    SparkFun_ADIN1110_Arduino_Library/src/fcs.c
    SparkFun_ADIN1110_Arduino_Library/src/hal.c
    SparkFun_ADIN1110_Arduino_Library/src/sfe_spe_advanced.cpp
    SparkFun_ADIN1110_Arduino_Library/src/boardsupport_ard.cpp
    SparkFun_ADIN1110_Arduino_Library/src/SparkFun_SinglePairEthernet.cpp
)

target_include_directories( whaletail_10baset1l
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" )

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    hardware_spi
    hardware_gpio
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)