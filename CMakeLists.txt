cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(pico232h C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(pico232h
    cpu-fifo.c
    main.c
)

pico_set_binary_type(pico232h no_flash)

pico_enable_stdio_uart(pico232h 1)
pico_enable_stdio_usb(pico232h 0)

# by default the header is generated into the build dir
pico_generate_pio_header(pico232h ${CMAKE_CURRENT_LIST_DIR}/cpu-fifo.pio)
# however, alternatively you can choose to generate it somewhere else (in this case in the source tree for check in)
#pico_generate_pio_header(pio_blink ${CMAKE_CURRENT_LIST_DIR}/blink.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})

# target_sources(pio_blink PRIVATE blink.c)

target_link_libraries(pico232h pico_stdlib hardware_pio pico_multicore)
pico_add_extra_outputs(pico232h)
