cmake_minimum_required(VERSION 3.10)


# Include toolchain configuration file
include(cmake/gcc-arm-none-eabi.cmake)

project(slatFirmwareBuildTest C ASM)


# Common compiler flags for all targets
add_compile_options(
    -std=gnu11
	#-Wall -Wextra -Werror -Wno-deprecated
	-Wall -Wextra -Wno-deprecated
    -fmessage-length=0
    -fsigned-char
    -ffunction-sections -fdata-sections
    -ffreestanding
    -fno-move-loop-invariants
    -Os -g3
)


# Common linker options for all targets
add_link_options(
    -Wall -Wextra -g3
    -nostartfiles -Xlinker --gc-sections --specs=nano.specs
)


# Directories for libraries
message("Add libraries:")
add_subdirectory(drivers/stm32f0)
#add_subdirectory(drivers/stm32f4)
add_subdirectory(drivers/stm32g4)
add_subdirectory(drivers/newlib)


# Project source files
set(
    SOURCE_FILES
        include/config.h

        include/gs_usb.h
        include/usbd_desc.h src/usbd_desc.c
        include/usbd_gs_can.h src/usbd_gs_can.c
        src/usbd_conf.c

        include/can.h src/can.c
        include/dfu.h src/dfu.c
        include/flash.h src/flash.c
        include/gpio.h src/gpio.c
        include/led.h src/led.c
        include/queue.h src/queue.c
        include/timer.h src/timer.c
        include/util.h src/util.c

        #src/startup.c
        src/main.c
        src/interrupts.c

)

set(
    INCLUDE_DIRECTORIES   
        include/
)

# Record HG rev
#add_custom_target(
#    HGREV_TARGET
#    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#    COMMAND python ../drivers/sla_lib/sla_hg_buildscript.py
#)


####### some helpers to generate targets
function(make_bin_file target)
    add_custom_command(
        TARGET ${target} POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_OBJCOPY} -O binary ${target} ${target}.bin
    )
endfunction()

# The compiler makes an elf... but TrueStudio wants the elf extension
function(make_elf_file target)
    add_custom_command(
        TARGET ${target} POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} -E copy ${target} ${target}.elf
    )
endfunction()

function(show_object_size target)
    string(REPLACE "objcopy" "size" CMAKE_OBJSIZE "${CMAKE_OBJCOPY}")
    add_custom_command(
        TARGET ${target} POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_OBJSIZE} ${target}
    )
endfunction()

function(add_flash_target target)
	add_custom_target(
		flash-${target} dfu-util -d 1d50:606f -a 0 -R -s 0x08000000 -D ${target}.bin
	)
endfunction()


# List to hold all targets that are added
message("Create target configurations:")
set(TARGETLIST )

# Add a new STM32 target
function(add_stm32_target TARGET_NAME TARGET_PROCESSOR)

    # Add target name to target list
    set(TARGETLIST ${TARGETLIST} ${TARGET_NAME}_fw PARENT_SCOPE)
    
	option(BUILD_${TARGET_NAME} "Build firmware for \"${TARGET_NAME}\" (default=yes)" ON)
    if (BUILD_${TARGET_NAME})

        # Trim target family from target processor. Yields "STM32F0" for example.
        string(SUBSTRING ${TARGET_PROCESSOR} 0 7 TARGET_FAMILY)
        string(TOUPPER ${TARGET_FAMILY} TARGET_FAMILY_UPPERCASE)
        
        # Trim down further to just target family. Yields "F0" for example.
        #string(SUBSTRING ${TARGET_FAMILY_UPPERCASE} 5 99 TARGET_FAMILY_SHORT)
        
        # Grab the specific part number (everything after STM32F0)
        string(SUBSTRING ${TARGET_PROCESSOR} 7  99 TARGET_PARTNUMBER_SUFFIX)
        # Make this last part uppercase except for X's so that our define matches what the ST libs expect
        string(TOUPPER ${TARGET_PARTNUMBER_SUFFIX} TARGET_PARTNUMBER_SUFFIX)
        string(REPLACE "X" "x" TARGET_PARTNUMBER_SUFFIX ${TARGET_PARTNUMBER_SUFFIX})

        
        # Form the specific processor definition (uppercase processor/family + lowercase suffix, e.g., STM32F042x6)
        string(CONCAT TARGET_PROCESSOR_DEFINITION ${TARGET_FAMILY_UPPERCASE} ${TARGET_PARTNUMBER_SUFFIX})
        # Form the lowercase version of the above text, for startup.S file inclusion
        string(TOLOWER ${TARGET_PROCESSOR_DEFINITION} TARGET_PROCESSOR_DEFINITION_LOWERCASE)

        message("  Created target \"${TARGET_NAME}\" with processor ${TARGET_PROCESSOR_DEFINITION}")
      
        # Add project sources
        add_executable(${TARGET_NAME}_fw ${SOURCE_FILES})
        target_include_directories(${TARGET_NAME}_fw PRIVATE ${INCLUDE_DIRECTORIES})


        # Set board name definition based on target name
        target_compile_definitions(${TARGET_NAME}_fw PRIVATE BOARD_${TARGET_NAME})
        message("Target board name: BOARD_${TARGET_NAME}")
        
        # Set the processor family definition (e.g. STM32F0)
        target_compile_definitions(${TARGET_NAME}_fw PRIVATE ${TARGET_FAMILY_UPPERCASE})
        
        # Set the specific part definition (e.g., STM32F02x6)
        target_compile_definitions(${TARGET_NAME}_fw PRIVATE ${TARGET_PROCESSOR_DEFINITION})

        
        
        if(${TARGET_FAMILY_UPPERCASE} STREQUAL "STM32F0")
            set(MCPU "cortex-m0")
            set(MATHDEF "ARM_MATH_CM0")
        elseif(${TARGET_FAMILY_UPPERCASE} STREQUAL "STM32F1")
            set(MCPU "cortex-m3")        
            set(MATHDEF "ARM_MATH_CM3")
        elseif(${TARGET_FAMILY_UPPERCASE} STREQUAL "STM32F3")
            set(MCPU "cortex-m4")
            set(MATHDEF "ARM_MATH_CM4")
        elseif(${TARGET_FAMILY_UPPERCASE} STREQUAL "STM32F4")
            set(MCPU "cortex-m4")
            set(MATHDEF "ARM_MATH_CM4")
        elseif(${TARGET_FAMILY_UPPERCASE} STREQUAL "STM32F7")
            set(MCPU "cortex-m7")
            set(MATHDEF "ARM_MATH_CM7")
            

        elseif(${TARGET_FAMILY_UPPERCASE} STREQUAL "STM32G0")
            set(MCPU "cortex-m0plus")
            set(MATHDEF "ARM_MATH_CM0PLUS")
        elseif(${TARGET_FAMILY_UPPERCASE} STREQUAL "STM32G4")
            set(MCPU "cortex-m4")
            set(MATHDEF "ARM_MATH_CM4")

            
        elseif(${TARGET_FAMILY_UPPERCASE} STREQUAL "STM32L0")
            set(MCPU "cortex-m0plus")
            set(MATHDEF "ARM_MATH_CM0PLUS")
        elseif(${TARGET_FAMILY_UPPERCASE} STREQUAL "STM32L1")
            set(MCPU "cortex-m0plus")
            set(MATHDEF "ARM_MATH_CM0PLUS")
        elseif(${TARGET_FAMILY_UPPERCASE} STREQUAL "STM32L4")
            set(MCPU "cortex-m4")
            set(MATHDEF "ARM_MATH_CM4")
        elseif(${TARGET_FAMILY_UPPERCASE} STREQUAL "STM32L4plus")
            set(MCPU "cortex-m4")
            set(MATHDEF "ARM_MATH_CM4")
        else()
            message(FATAL_ERROR "MCPU is invalid, could not find ${TARGET_FAMILY_UPPERCASE}! Check your processor name string.")
        endif()
        
        
       string(TOLOWER ${TARGET_FAMILY_UPPERCASE} TARGET_FAMILY_LOWERCASE)

        
        # Add processor type
        #message("\tDetected core " ${MCPU})
        target_compile_options(${TARGET_NAME}_fw PRIVATE -mcpu=${MCPU} -mthumb)
        target_link_options(${TARGET_NAME}_fw PRIVATE -mcpu=${MCPU} -mthumb -O -T ${CMAKE_SOURCE_DIR}/targets/${TARGET_NAME}_${TARGET_PROCESSOR_DEFINITION_LOWERCASE}.ld)        


        # Pull in the libraries library for this specific part number
        message("Pulling in HAL library HAL_${TARGET_PROCESSOR_DEFINITION_LOWERCASE}_${TARGET_NAME}" )
        target_link_libraries(${TARGET_NAME}_fw PRIVATE HAL_${TARGET_PROCESSOR_DEFINITION_LOWERCASE}_${TARGET_NAME} NEWLIB_${TARGET_FAMILY_UPPERCASE})
        message("    Linking library HAL_${TARGET_PROCESSOR_DEFINITION_LOWERCASE}")
        message("    Linking library newlib_${TARGET_FAMILY_UPPERCASE}")
        
        
        # This sets which HAL conf file we include
        target_compile_definitions(${TARGET_NAME}_fw PRIVATE -DTARGET_HAL_CONFIGURATION="${TARGET_NAME}_${TARGET_PROCESSOR_DEFINITION_LOWERCASE}_hal_conf.h")

        # This sets which HAL library
        target_compile_definitions(${TARGET_NAME}_fw PRIVATE -DTARGET_HAL_LIB_INCLUDE="${TARGET_FAMILY_LOWERCASE}xx_hal.h")
        message("    Adding HAL include ${TARGET_FAMILY_LOWERCASE}xx_hal.h")

        # Startup file is added in each HAL library
        
        # Set which math lib to compile for
        target_compile_definitions(${TARGET_NAME}_fw PRIVATE ${MATHDEF})

        # Our firmware needs the hgrev.h to compile
 #       add_dependencies(${TARGET_NAME}_fw HGREV_TARGET)
        
        # The sla_lib needs hgrev.hg to compile
#        add_dependencies(SLA_LIB_${TARGET_FAMILY_UPPERCASE} HGREV_TARGET)
        
        

        
        make_bin_file(${TARGET_NAME}_fw)
        make_elf_file(${TARGET_NAME}_fw)
        show_object_size(${TARGET_NAME}_fw)
        add_flash_target(${TARGET_NAME}_fw)
        
    endif()
    
    
endfunction()


# Add all the boards/processors we would like to compile for
add_stm32_target("canable" "stm32f042x6")
add_stm32_target("canable2" "stm32g431xx")

message("\r\n")
message("You may now:\n\t-compile all targets ('make')\n\t-compile a single target (e.g. 'make mytarget_fw'")
message("\t-flash a device (e.g. 'make flash-mytarget_fw'")
message("\r\nAvailable targets:")
foreach (TARGETNAME IN LISTS TARGETLIST)
    message("\t${TARGETNAME}")
endforeach()
message("\r\n")

