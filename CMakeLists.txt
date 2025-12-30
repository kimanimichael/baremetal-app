cmake_minimum_required(VERSION 3.19.0 FATAL_ERROR)

set(CMAKE_ASM_COMPILER "arm-none-eabi-gcc")
set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_LINKER "arm-none-eabi-gcc")
set(CMAKE_OBJCOPY "arm-none-eabi-objcopy")

set(CMAKE_C_COMPILER_WORKS 1)

set (APP_TARGET baremetal_app)

add_executable(${APP_TARGET})

set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/app/app.ld")
set_target_properties(${APP_TARGET} PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

target_compile_options(baremetal_app PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -march=armv7e-m
        -Wall
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        --specs=nano.specs    
)

target_link_options(baremetal_app PRIVATE
        "-nostartfiles"
        "-T${LINKER_SCRIPT}"
        "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/baremetal_app.map"
        ${CPU_FLAGS}
        -u_printf_float
        -u_scanf_float
        --specs=nano.specs
        --specs=nosys.specs
)

set(CMAKE_C_FLAGS "${CPU_FLAGS} -O0 -g -Wall")

project(${APP_TARGET} C)

enable_language(ASM)

target_include_directories(${APP_TARGET}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/common/include
        ${CMAKE_CURRENT_SOURCE_DIR}/common/include/core
        ${CMAKE_CURRENT_SOURCE_DIR}/common/include/cmsis
        app/include
)

target_sources(${APP_TARGET}
        PRIVATE
        common/src/core/system_stm32f4xx.c
        common/src/core/uart.c
        app/src/bsp.c
        app/src/main.c
        app/src/startup.c
        app/src/syscalls.c
        app/src/bootloader.S
)

add_custom_command(TARGET ${APP_TARGET} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_BINARY_DIR}/${APP_TARGET} ${CMAKE_BINARY_DIR}/${APP_TARGET}.bin
)

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()
