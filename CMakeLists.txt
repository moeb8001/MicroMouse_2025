# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(MicroMouse_Robot C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add source files
set(SOURCES
    MicroMouse_Robot.c
    src/tof/tof.c
    src/motor/motor.c
    src/encoder/encoder.c
    src/controller/controller.c
    src/imu/imu.c
)

# Add executable
add_executable(MicroMouse_Robot ${SOURCES})

pico_set_program_name(MicroMouse_Robot "MicroMouse_Robot")
pico_set_program_version(MicroMouse_Robot "0.1")

# Generate PIO header
pico_generate_pio_header(MicroMouse_Robot ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(MicroMouse_Robot 0)
pico_enable_stdio_usb(MicroMouse_Robot 1)

# Add the standard library to the build
target_link_libraries(MicroMouse_Robot
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(MicroMouse_Robot PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(MicroMouse_Robot 
        hardware_spi
        hardware_i2c
        hardware_dma
        hardware_pio
        hardware_interp
        hardware_timer
        hardware_watchdog
        hardware_clocks
        hardware_pwm
        )

pico_add_extra_outputs(MicroMouse_Robot)

