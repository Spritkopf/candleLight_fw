
if (WIN32)

    set (CMAKE_GENERATOR "Unix Makefiles" CACHE INTERNAL "" FORCE)

    find_path(
        TOOLCHAIN_BIN_DIR 
        arm-none-eabi-gcc.exe
        HINTS 
            "/c/dev_tools"
        ENV TOOLCHAIN_BIN_DIR
    )

    set(CMAKE_SYSTEM_NAME Generic)
    set(CMAKE_SYSTEM_PROCESSOR arm)

    set(CMAKE_C_COMPILER "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-gcc.exe" CACHE INTERNAL "")
    set(CMAKE_ASM_COMPILER "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-gcc.exe" CACHE INTERNAL "")
    set(CMAKE_CXX_COMPILER "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-g++.exe" CACHE INTERNAL "")

endif(WIN32)

if(UNIX)
    
    find_path(
        TOOLCHAIN_BIN_DIR 
        arm-none-eabi-gcc
        HINTS 
            /opt/gcc-arm-none-eabi-8-2019-q3-update/bin
        ENV TOOLCHAIN_BIN_DIR
    )

    set(CMAKE_SYSTEM_NAME Generic)
    set(CMAKE_SYSTEM_PROCESSOR arm)

    set(CMAKE_C_COMPILER "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-gcc" CACHE INTERNAL "")
    set(CMAKE_CXX_COMPILER "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-g++" CACHE INTERNAL "")

endif(UNIX)
    
enable_language(ASM)

SET(CMAKE_ASM_FLAGS "${CFLAGS} -x assembler-with-cpp")

    
set(CMAKE_EXE_LINKER_FLAGS "" CACHE INTERNAL "")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

SET (CMAKE_C_COMPILER_WORKS 1)
SET (CMAKE_CXX_COMPILER_WORKS 1)