project(STM32_HAL)

include(../../cmake/slalib.cmake)

set(SOURCES

    cmsis/inc/cmsis_armcc.h
    cmsis/inc/arm_math.h
    cmsis/inc/cmsis_armcc_V6.h
    cmsis/inc/arm_common_tables.h
    cmsis/inc/core_cm3.h
    cmsis/inc/core_cm0plus.h
    cmsis/inc/core_cm7.h
    cmsis/inc/cmsis_gcc.h
    cmsis/inc/core_cmInstr.h
    cmsis/inc/core_cmFunc.h
    cmsis/inc/core_cm0.h
    cmsis/inc/arm_const_structs.h
    cmsis/inc/core_sc300.h
    cmsis/inc/device/stm32f446xx.h
    cmsis/inc/device/stm32f4xx.h
    cmsis/inc/device/system_stm32f4xx.h
    cmsis/inc/cmsis_device.h
    cmsis/inc/core_cm4.h
    cmsis/inc/core_sc000.h
    cmsis/inc/core_cmSimd.h
    
    cmsis/src/system_stm32f4xx.c

    hal/src/stm32f4xx_hal.c
    hal/src/stm32f4xx_hal_can.c
    hal/src/stm32f4xx_hal_cortex.c
    hal/src/stm32f4xx_hal_dma.c
    hal/src/stm32f4xx_hal_flash.c
    hal/src/stm32f4xx_hal_flash_ex.c
    hal/src/stm32f4xx_hal_gpio.c
    hal/src/stm32f4xx_hal_i2c.c
    hal/src/stm32f4xx_hal_i2c_ex.c
    hal/src/stm32f4xx_hal_pcd.c
    hal/src/stm32f4xx_hal_pcd_ex.c
    hal/src/stm32f4xx_hal_pwr.c
    hal/src/stm32f4xx_hal_pwr_ex.c
    hal/src/stm32f4xx_hal_rcc.c
    hal/src/stm32f4xx_hal_rcc_ex.c
    hal/src/stm32f4xx_hal_tim.c
    hal/src/stm32f4xx_hal_tim_ex.c
    hal/src/stm32f4xx_hal_uart.c
    hal/src/stm32f4xx_hal_usart.c

    hal/inc/stm32f4xx_hal_conf.h
    hal/inc/stm32f4xx_hal_tim.h
    hal/inc/stm32f4xx_hal_pcd.h
    hal/inc/stm32f4xx_hal_gpio.h
    hal/inc/stm32f4xx_hal_i2c_ex.h
    hal/inc/stm32f4xx_hal_flash_ex.h
    hal/inc/stm32f4xx_hal_dma_ex.h
    hal/inc/stm32f4xx_hal_i2c.h
    hal/inc/stm32f4xx_hal_pwr.h
    hal/inc/stm32f4xx_hal_pwr_ex.h
    hal/inc/stm32f4xx_hal_dma.h
    hal/inc/stm32f4xx_hal_gpio_ex.h
    hal/inc/Legacy/stm32_hal_legacy.h
    hal/inc/stm32f4xx_hal_rcc_ex.h
    hal/inc/stm32f4xx_hal_def.h
    hal/inc/stm32f4xx_hal.h
    hal/inc/stm32f4xx_hal_rcc.h
    hal/inc/stm32f4xx_hal_cortex.h
    hal/inc/stm32f4xx_hal_flash.h
    hal/inc/stm32f4xx_hal_tim_ex.h
    hal/inc/stm32f4xx_hal_pcd_ex.h
    hal/inc/stm32f4xx_hal_can.h

)

set(INCLUDE_DIRS
    cmsis/inc
    cmsis/inc/device
    hal/inc 
    config/
    ../../targets/
    ../../inc/
)

slalib_add_hal_library("STM32F4" "cortex-m4" "${SOURCES}" "${INCLUDE_DIRS}")
