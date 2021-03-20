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
    cmsis/inc/device/stm32f031x6.h
    cmsis/inc/device/stm32f048xx.h
    cmsis/inc/device/stm32f042x6.h
    cmsis/inc/device/stm32f091xc.h
    cmsis/inc/device/stm32f0xx.h
    cmsis/inc/device/stm32f030x6.h
    cmsis/inc/device/stm32f030x8.h
    cmsis/inc/device/stm32f078xx.h
    cmsis/inc/device/system_stm32f0xx.h
    cmsis/inc/device/stm32f038xx.h
    cmsis/inc/device/stm32f058xx.h
    cmsis/inc/device/stm32f070xb.h
    cmsis/inc/device/stm32f098xx.h
    cmsis/inc/device/stm32f070x6.h
    cmsis/inc/device/stm32f071xb.h
    cmsis/inc/device/stm32f030xc.h
    cmsis/inc/device/stm32f051x8.h
    cmsis/inc/device/stm32f072xb.h
    cmsis/inc/cmsis_device.h
    cmsis/inc/core_cm4.h
    cmsis/inc/core_sc000.h
    cmsis/inc/core_cmSimd.h

    hal/src/stm32f0xx_hal.c
    hal/src/stm32f0xx_hal_can.c
    hal/src/stm32f0xx_hal_cortex.c
    hal/src/stm32f0xx_hal_dma.c
    hal/src/stm32f0xx_hal_flash.c
    hal/src/stm32f0xx_hal_flash_ex.c
    hal/src/stm32f0xx_hal_gpio.c
    hal/src/stm32f0xx_hal_i2c.c
    hal/src/stm32f0xx_hal_i2c_ex.c
    hal/src/stm32f0xx_hal_pcd.c
    hal/src/stm32f0xx_hal_pcd_ex.c
    hal/src/stm32f0xx_hal_pwr.c
    hal/src/stm32f0xx_hal_pwr_ex.c
    hal/src/stm32f0xx_hal_rcc.c
    hal/src/stm32f0xx_hal_rcc_ex.c
    hal/src/stm32f0xx_hal_tim.c
    hal/src/stm32f0xx_hal_tim_ex.c

    cmsis/src/system_stm32f0xx.c

    hal/inc/stm32f0xx_hal_conf.h
    hal/inc/stm32f0xx_hal_tim.h
    hal/inc/stm32f0xx_hal_pcd.h
    hal/inc/stm32f0xx_hal_gpio.h
    hal/inc/stm32f0xx_hal_i2c_ex.h
    hal/inc/stm32f0xx_hal_flash_ex.h
    hal/inc/stm32f0xx_hal_dma_ex.h
    hal/inc/stm32f0xx_hal_i2c.h
    hal/inc/stm32f0xx_hal_pwr.h
    hal/inc/stm32f0xx_hal_pwr_ex.h
    hal/inc/stm32f0xx_hal_dma.h
    hal/inc/stm32f0xx_hal_gpio_ex.h
    hal/inc/Legacy/stm32_hal_legacy.h
    hal/inc/stm32f0xx_hal_rcc_ex.h
    hal/inc/stm32f0xx_hal_def.h
    hal/inc/stm32f0xx_hal.h
    hal/inc/stm32f0xx_hal_rcc.h
    hal/inc/stm32f0xx_hal_cortex.h
    hal/inc/stm32f0xx_hal_flash.h
    hal/inc/stm32f0xx_hal_tim_ex.h
    hal/inc/stm32f0xx_hal_pcd_ex.h
    hal/inc/stm32f0xx_hal_can.h
    
    usb_device/Core/Inc/usbd_ioreq.h
    usb_device/Core/Inc/usbd_def.h
    usb_device/Core/Inc/usbd_ctlreq.h
    usb_device/Core/Inc/usbd_core.h
    usb_device/Core/Src/usbd_ioreq.c
    usb_device/Core/Src/usbd_ctlreq.c
    usb_device/Core/Src/usbd_core.c
    usb_device/config/usbd_conf.h
    
)

set(INCLUDE_DIRS
    cmsis/inc
    cmsis/inc/device
    hal/inc 
    config/
    ../../targets/
    ../../inc/
    usb_device/Core/Inc
    usb_device/config
)

slalib_add_hal_library("STM32F0" "cortex-m0" "${SOURCES}" "${INCLUDE_DIRS}")
