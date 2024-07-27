cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(pico232h C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(pico232h
    cpu-fifo.c
    main.c
    usb-descriptors.c
)

target_include_directories(pico232h PUBLIC
    ./
    ${PICO_TINYUSB_PATH}/src)

pico_set_binary_type(pico232h copy_to_ram)

pico_generate_pio_header(pico232h ${CMAKE_CURRENT_LIST_DIR}/cpu-fifo.pio)

target_link_libraries(pico232h
    hardware_flash    
    hardware_pio
    pico_multicore
    pico_stdlib
    pico_unique_id
    pico_usb_reset_interface
    tinyusb_device)

pico_add_extra_outputs(pico232h)
