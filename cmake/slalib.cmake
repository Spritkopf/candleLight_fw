#
# cmake stm32 multi-target library
#
#  Ethan Zonca 2021
#


# Return useful strings based off of fully qualified path to a startup.s file
function(slalib_stm32_info_from_startup_filestr IN_FILSTR)

  # Find location of startup_ in string to trim
  string(FIND ${IN_FILSTR} "startup_" STARTUP_INDEX REVERSE)
  string(SUBSTRING ${IN_FILSTR} ${STARTUP_INDEX} 99 STARTUP_FILENAME)
    
  # Trim string down to just the filename, no path
  string(SUBSTRING ${STARTUP_FILENAME} 8 99 DERIVED_TARGET_NAME)
  
  # Trim .s file extension suffix
  string(LENGTH ${DERIVED_TARGET_NAME} FILENAMELEN)
  math(EXPR FILENAMELEN ${FILENAMELEN}-2)
  string(SUBSTRING ${DERIVED_TARGET_NAME} 0 ${FILENAMELEN} DERIVED_TARGET_NAME)
  
  # Trim target family from target processor. Yields "STM32F0" for example.
  string(SUBSTRING ${DERIVED_TARGET_NAME} 0 7 TARGET_FAMILY)
  string(TOUPPER ${TARGET_FAMILY} TARGET_FAMILY_UPPERCASE)

  # Grab the specific part number (everything after STM32F0)
  string(SUBSTRING ${DERIVED_TARGET_NAME} 7  99 TARGET_PARTNUMBER_SUFFIX)
  
  # Make this all uppercase as any non-'x' letters in the suffix should be in caps
  # For some reason, the partnumber definition is mixed-case but the cmsis include is all lowercase... Thanks, ST.
  string(TOUPPER ${TARGET_PARTNUMBER_SUFFIX} TARGET_PARTNUMBER_SUFFIX)
  # Replace all uppercase X's with lowercase x's.
  string(REPLACE "X" "x" TARGET_PARTNUMBER_SUFFIX ${TARGET_PARTNUMBER_SUFFIX})
  
  # Form the specific processor definition (uppercase processor/family + lowercase suffix, e.g., STM32F042x6)
  string(CONCAT TARGET_PROCESSOR_DEFINITION ${TARGET_FAMILY_UPPERCASE} ${TARGET_PARTNUMBER_SUFFIX})
  # Form the lowercase version of the above text, for startup.S file inclusion
  string(TOLOWER ${TARGET_PROCESSOR_DEFINITION} TARGET_PROCESSOR_DEFINITION_LOWERCASE)

  
  ########### Return ################
  # Target Processor Definition:  e.g. STM32F042x6
  set(SLALIB_PROCESSOR_DEFINITION ${TARGET_PROCESSOR_DEFINITION} PARENT_SCOPE)
  # Target Processor Definition: all lowercase (e.g., stm32g471xx)
  set(SLALIB_PROCESSOR_DEFINITION_LOWERCASE ${DERIVED_TARGET_NAME} PARENT_SCOPE)
  # Startup.S filename (e.g., startup_stm32g471xx.s)
  set(SLALIB_STARTUP_FILENAME ${STARTUP_FILENAME} PARENT_SCOPE)
  # Target family (e.g., STM32F0)
  set(SLALIB_STARTUP_TARGET_FAMILY ${TARGET_FAMILY_UPPERCASE} PARENT_SCOPE)

endfunction()



# Return useful strings based off of fully qualified path to HAL configuration file (PROJECT-SPECIFIC)
function(slalib_stm32_info_from_halconfig IN_FILSTR)

  # Find location of startup_ in string to trim
  string(FIND ${IN_FILSTR} "_stm32" CORE_INDEX REVERSE)
  
  # Trim off leading string
  string(SUBSTRING ${IN_FILSTR} ${CORE_INDEX} 99 PROCESSOR_SERIES)

  # Trim down to fully-qualified processor name (e.g., f446xx)
  string(SUBSTRING ${PROCESSOR_SERIES} 6 6 PROCESSOR_SERIES)

  string(CONCAT PROCESSOR_SERIES "stm32" ${PROCESSOR_SERIES})
  #string(TOUPPER ${PROCESSOR_SERIES} PROCESSOR_SERIES)



    string(FIND ${IN_FILSTR} "/" LASTSLASH_INDEX REVERSE)
    # Trim to slash
    string(SUBSTRING ${IN_FILSTR} ${LASTSLASH_INDEX} 99 PROJECT_NAME)
    # Trim slash itself to the index of the processor part of the string
    string(FIND ${PROJECT_NAME} "_stm32" NM_IDX REVERSE)
    math(EXPR NM_IDX "${NM_IDX} - 1")
    string(SUBSTRING ${PROJECT_NAME} 1 ${NM_IDX} PROJECT_NAME)

    
    #message("!!!Project name is ${PROJECT_NAME} ${LASTSLASH_INDEX}, series is ${PROCESSOR_SERIES}")
    
    
  ########### Return ################
  # Target Processor Series:  e.g. STM32F0
  set(SLALIB_HALCONF_PROCESSOR_SERIES ${PROCESSOR_SERIES} PARENT_SCOPE)
  # Target Project Name:  e.g. canable_flex
  set(SLALIB_HALCONF_PROJECT_NAME ${PROJECT_NAME} PARENT_SCOPE)

    

endfunction()



# Add a new HAL library: create target configurations for each fully-qualified stm32 processor that has a startup file in this HAL library
function(slalib_add_hal_library PROCESSOR_FAMILY CORE_TYPE SOURCES_IN INCLUDES_IN)

    #file(GLOB startupfiles "cmsis/src/startup_*")
    file(GLOB halconfigs "../../targets/*_hal_conf.h")

    foreach(halconfig ${halconfigs})

        #foreach(startupfile ${startupfiles})
        
            # Calculate various useful strings from the filename (returned as set values)
            slalib_stm32_info_from_halconfig(${halconfig})
            slalib_stm32_info_from_startup_filestr("startup_${SLALIB_HALCONF_PROCESSOR_SERIES}.s")

            
            
            ## TODO: This will only create HAL configs for the passed in core.
            # Future, this function could just do everything and iterate through all configs.
            
            message("Comparing ${SLALIB_STARTUP_TARGET_FAMILY} ${PROCESSOR_FAMILY}")
            
            if(SLALIB_STARTUP_TARGET_FAMILY STREQUAL PROCESSOR_FAMILY)
                message("Building target ${SLALIB_HALCONF_PROCESSOR_SERIES} for project ${SLALIB_HALCONF_PROJECT_NAME}")

                string(CONCAT MODULE_NAME "HAL_" ${SLALIB_HALCONF_PROCESSOR_SERIES} "_" ${SLALIB_HALCONF_PROJECT_NAME})
                message("MODULE NAME IS ${MODULE_NAME}")
                
                # Add library and set compilation options
                add_library(${MODULE_NAME} OBJECT ${SOURCES_IN} )
                message("  HAL library: ${MODULE_NAME}")
                target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDES_IN})
                target_compile_options(${MODULE_NAME} PRIVATE -Wno-unused-parameter -Wno-deprecated)
                target_compile_options(${MODULE_NAME} PRIVATE -mcpu=${CORE_TYPE} -mthumb)
                target_compile_definitions(${MODULE_NAME} PRIVATE ${SLALIB_PROCESSOR_DEFINITION})
                
                # This sets which HAL conf file we include
                target_compile_definitions(${MODULE_NAME} PRIVATE -DTARGET_HAL_CONFIGURATION="${SLALIB_HALCONF_PROJECT_NAME}_${SLALIB_HALCONF_PROCESSOR_SERIES}_hal_conf.h")
                target_sources(${MODULE_NAME} PRIVATE "../../targets/${SLALIB_HALCONF_PROJECT_NAME}_${SLALIB_HALCONF_PROCESSOR_SERIES}_hal_conf.h")
                
                #target_sources(${MODULE_NAME} PRIVATE "cmsis/src/startup_${SLALIB_HALCONF_PROCESSOR_SERIES}.s")

                
                message("Adding source ../../targets/${SLALIB_HALCONF_PROJECT_NAME}_${SLALIB_HALCONF_PROCESSOR_SERIES}_hal_conf.h")
                
                #target_sources(${MODULE_NAME} PRIVATE cmsis/src/${SLALIB_STARTUP_FILENAME})
                target_sources(${MODULE_NAME} PRIVATE ../../targets/${SLALIB_HALCONF_PROJECT_NAME}_${SLALIB_STARTUP_FILENAME})
          
          #message("Added target defintion ${SLALIB_PROCESSOR_DEFINITION}")

           endif()
        
      
      
          
      #endforeach()
    endforeach()


endfunction()
