project(NEWLIB)

set(SOURCES
    _sbrk.c
    _syscalls.c
    _init.c
)


add_library(NEWLIB_STM32F0 OBJECT ${SOURCES})
    message("  newlib stm32f0")
	target_compile_options(NEWLIB_STM32F0 PRIVATE -mcpu=cortex-m0 -mthumb)
    target_compile_options(NEWLIB_STM32F0 PRIVATE -Wno-unused-parameter -Wno-deprecated)

add_library(NEWLIB_STM32F4 OBJECT ${SOURCES})
    message("  newlib stm32f4")
	target_compile_options(NEWLIB_STM32F4 PRIVATE -mcpu=cortex-m3 -mthumb)
    target_compile_options(NEWLIB_STM32F4 PRIVATE -Wno-unused-parameter -Wno-deprecated)

add_library(NEWLIB_STM32G4 OBJECT ${SOURCES})
    message("  newlib stm32g4")
	target_compile_options(NEWLIB_STM32G4  PRIVATE -mcpu=cortex-m3 -mthumb)
    target_compile_options(NEWLIB_STM32G4   PRIVATE -Wno-unused-parameter -Wno-deprecated)
