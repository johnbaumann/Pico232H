# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(pico232h C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
pico_sdk_init()

add_executable(pico232h
    cpu-fifo.c
    main.c
    usb-descriptors.c
)

# Set CPU frequency to 276MHz at boot
# python vcocalc.py 276 --cmake
target_compile_definitions(pico232h PRIVATE
        PLL_SYS_REFDIV=1
        PLL_SYS_VCO_FREQ_HZ=1380000000
        PLL_SYS_POSTDIV1=5
        PLL_SYS_POSTDIV2=1
        SYS_CLK_HZ=276000000
        SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST=1
        SYS_CLK_VREG_VOLTAGE_MIN=VREG_VOLTAGE_1_30
)

target_include_directories(pico232h PUBLIC
    ./
    ${PICO_TINYUSB_PATH}/src)

pico_set_binary_type(pico232h copy_to_ram)

pico_generate_pio_header(pico232h ${CMAKE_CURRENT_LIST_DIR}/cpu-fifo.pio)

target_link_libraries(pico232h
    hardware_dma
    hardware_flash    
    hardware_pio
    pico_multicore
    pico_stdlib
    pico_unique_id
    pico_usb_reset_interface
    tinyusb_device)

pico_add_extra_outputs(pico232h)
