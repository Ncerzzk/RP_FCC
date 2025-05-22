cmake_minimum_required(VERSION 3.13...3.27)

# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

project(rp_fcc)

set(CMAKE_MODULE_PATH 
    ${CMAKE_CURRENT_SOURCE_DIR}/pico-bidir-dshot
)
add_subdirectory(pico-bidir-dshot)
# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(rp_fcc
    main.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(rp_fcc pico_stdlib DShot_for_RP2040_RP2350_Pi_Pico)
pico_enable_stdio_usb(rp_fcc 1)
pico_enable_stdio_uart(rp_fcc 1)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(rp_fcc)

