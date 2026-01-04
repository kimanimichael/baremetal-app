cmake_minimum_required(VERSION 3.19.0 FATAL_ERROR)

set(CMAKE_ASM_COMPILER "arm-none-eabi-gcc")
set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_LINKER "arm-none-eabi-ld")
set(CMAKE_OBJCOPY "arm-none-eabi-objcopy")

set(CMAKE_C_COMPILER_WORKS 1)

set (BOOT_TARGET baremetal_bootloader)

add_executable(${BOOT_TARGET})

set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/bootloader.ld")
set_target_properties(${BOOT_TARGET} PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

target_compile_options(baremetal_bootloader PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -march=armv7e-m
        -Wall
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard    
)

target_link_options(baremetal_bootloader PRIVATE
        "-nostartfiles"
        "-T${LINKER_SCRIPT}"
        "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/baremetal_bootloader.map"
        ${CPU_FLAGS}
)

set(CMAKE_C_FLAGS "${CPU_FLAGS} -O0 -g -Wall")

project(${BOOT_TARGET} C)

target_include_directories(${BOOT_TARGET}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/include/core
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/include/cmsis
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/include/utils
)

target_sources(${BOOT_TARGET}
        PRIVATE
        src/bootloader.c
        src/comms.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/src/core/ring_buffer.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/src/core/uart.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/src/utils/crc8.c

)

add_custom_command(TARGET ${BOOT_TARGET} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_BINARY_DIR}/${BOOT_TARGET} --pad-to=0x8008000 --gap-fill=0Xff ${CMAKE_BINARY_DIR}/${BOOT_TARGET}.bin
)

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()

