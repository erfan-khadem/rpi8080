cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)
include(pico_extras_import_optional.cmake)

project(i8080em C CXX ASM)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(
    i8080em
	main.c
	i8080.c
)


# pull in common dependencies and additional pwm hardware support
target_link_libraries(
	i8080em
	pico_stdlib
	pico_multicore
	hardware_pwm
	hardware_clocks
	hardware_resets
	pico_bootsel_via_double_reset
)

pico_enable_stdio_usb(i8080em 1)
pico_enable_stdio_uart(i8080em 0)

target_compile_definitions(
	i8080em PRIVATE
	PICO_BOOTSEL_VIA_DOUBLE_RESET_ACTIVITY_LED=25
)

# create map/bin/hex file etc.
pico_add_extra_outputs(i8080em)
