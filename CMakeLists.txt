cmake_minimum_required(VERSION 3.13...3.27)

# initialize pico-sdk from submodule
# note: this must happen before project()
set(PICO_CXX_ENABLE_EXCEPTIONS ON CACHE BOOL "enable CPP exceptions" FORCE) # enabled for PicoLED
include(pico-sdk/pico_sdk_init.cmake)

project(rp_fcc)

set(CMAKE_MODULE_PATH 
    ${CMAKE_CURRENT_SOURCE_DIR}/pico-bidir-dshot
)
add_subdirectory(pico-bidir-dshot)
# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(rp_fcc
    main.cpp
    minihdlc/minihdlc.c
)

include(PicoLED/PicoLed.cmake)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(rp_fcc pico_stdlib Pico_Bidir_DShot PicoLed)
pico_enable_stdio_usb(rp_fcc 1)
pico_enable_stdio_uart(rp_fcc 1)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(rp_fcc)

