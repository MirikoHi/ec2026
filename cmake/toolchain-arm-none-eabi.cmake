# Toolchain file for ARM Cortex-M0+ (MSPM0G3507) with arm-none-eabi-gcc
# Usage: cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake -G Ninja

set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

# Toolchain prefix
# ----------------------------------------------------------------------
# Locate arm-none-eabi-gcc
# ----------------------------------------------------------------------
if(DEFINED ENV{ARM_GCC_ROOT})
    set(TOOLCHAIN_ROOT "$ENV{ARM_GCC_ROOT}")
else()
    # Try to find the compiler in PATH
    find_program(CMAKE_C_COMPILER_FOUND arm-none-eabi-gcc)
    if(CMAKE_C_COMPILER_FOUND)
        get_filename_component(TOOLCHAIN_BIN_DIR ${CMAKE_C_COMPILER_FOUND} DIRECTORY)
        get_filename_component(TOOLCHAIN_ROOT ${TOOLCHAIN_BIN_DIR} DIRECTORY)
    else()
        message(FATAL_ERROR
                "arm-none-eabi-gcc not found. Please install the toolchain and add it to PATH, "
                "or set the ARM_GCC_ROOT environment variable to its installation directory.")
    endif()
endif()

set(TOOLCHAIN_PREFIX "${TOOLCHAIN_ROOT}/bin/arm-none-eabi-")

# ----------------------------------------------------------------------
# Compiler / tools
# ----------------------------------------------------------------------
set(CMAKE_TRY_COMPILE_TARGET_TYPE   STATIC_LIBRARY)

set(CMAKE_C_COMPILER                "${TOOLCHAIN_PREFIX}gcc.exe")
set(CMAKE_ASM_COMPILER              "${TOOLCHAIN_PREFIX}gcc.exe")
set(CMAKE_CXX_COMPILER              "${TOOLCHAIN_PREFIX}g++.exe")

set(CMAKE_OBJCOPY                   "${TOOLCHAIN_PREFIX}objcopy")
set(CMAKE_OBJDUMP                   "${TOOLCHAIN_PREFIX}objdump")
set(CMAKE_SIZE                      "${TOOLCHAIN_PREFIX}size")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Common flags for Cortex-M0+
set(ARCH_FLAGS "-mcpu=cortex-m0plus -march=armv6-m -mthumb -mfloat-abi=soft")

set(CMAKE_C_FLAGS_INIT               "${ARCH_FLAGS} -std=c99")
set(CMAKE_ASM_FLAGS_INIT             "${ARCH_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_INIT      "${ARCH_FLAGS} -Wl,--gc-sections -static --specs=nano.specs -nostartfiles")
