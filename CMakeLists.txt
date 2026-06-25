cmake_minimum_required(VERSION 3.20)
project(electric-competition-robot C ASM)

# ============================================================
# MCU and compiler flags
# (CPU architecture flags are set in the toolchain file)
# ============================================================
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -g -gstrict-dwarf -ffunction-sections -fdata-sections -Wall")

add_compile_definitions(__MSPM0G3507__)

# ============================================================
# SysConfig generated files (pre-build step)
# ============================================================
set(SYSCFG_OUTPUT
        ${CMAKE_CURRENT_SOURCE_DIR}/ti_msp_dl_config.c
        ${CMAKE_CURRENT_SOURCE_DIR}/ti_msp_dl_config.h
)

# 查找 SysConfig CLI（环境变量或 PATH）
if(DEFINED ENV{SYSCONFIG_ROOT})
    find_program(SYSCONFIG_CLI
            NAMES sysconfig_cli.bat sysconfig_cli
            PATHS "$ENV{SYSCONFIG_ROOT}"
            NO_DEFAULT_PATH
    )
else()
    find_program(SYSCONFIG_CLI
            NAMES sysconfig_cli.bat sysconfig_cli
    )
endif()

if(SYSCONFIG_CLI)
    add_custom_command(
            OUTPUT ${SYSCFG_OUTPUT}
            COMMAND ${SYSCONFIG_CLI}
            -o ${CMAKE_CURRENT_SOURCE_DIR}
            -s ${CMAKE_CURRENT_SOURCE_DIR}/.metadata/product.json
            --compiler gcc
            ${CMAKE_CURRENT_SOURCE_DIR}/robot.syscfg
            DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/robot.syscfg
            COMMENT "Running SysConfig to regenerate ti_msp_dl_config.c/h"
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    set_source_files_properties(${SYSCFG_OUTPUT} PROPERTIES GENERATED TRUE)
else()
    message(STATUS "SysConfig CLI not found — using pre-generated ti_msp_dl_config.c/h")
endif()

# ============================================================
# 递归包含头文件目录的函数
# ============================================================
# 递归包含头文件目录，并打印每个目录下的头文件列表（用于调试）
function(include_sub_directories_recursively root_dir)
    _include_recursive("${root_dir}" "")
endfunction()

function(_include_recursive root_dir indent)
    if (IS_DIRECTORY ${root_dir})
        # 1. 打印当前目录路径
        message(STATUS "${indent}Include dir: ${root_dir}")
        target_include_directories(${PROJECT_NAME} PRIVATE ${root_dir})

        # 2. 列出该目录下的所有头文件并打印
        file(GLOB HEADER_FILES "${root_dir}/*.h" "${root_dir}/*.hpp" "${root_dir}/*.inc")
        foreach(hdr ${HEADER_FILES})
            get_filename_component(hdr_name ${hdr} NAME)
            message(STATUS "${indent}  [H] ${hdr_name}")
        endforeach()
    endif()

    # 递归处理子目录
    file(GLOB ALL_SUB RELATIVE ${root_dir} ${root_dir}/*)
    foreach(sub ${ALL_SUB})
        if (IS_DIRECTORY ${root_dir}/${sub})
            _include_recursive("${root_dir}/${sub}" "${indent}  ")
        endif()
    endforeach()
endfunction()

# ============================================================
# FreeRTOS kernel source files（保持显式列表，因为结构固定）
# ============================================================
set(FREERTOS_CORE_DIR     Vendor/FreeRTOS/Source)
set(FREERTOS_PORT_DIR     Vendor/FreeRTOS/Source/portable/GCC/ARM_CM0)
set(FREERTOS_MEM_DIR      Vendor/FreeRTOS/Source/portable/MemMang)
set(FREERTOS_DPL_DIR      Vendor/FreeRTOS/dpl)
set(FREERTOS_BUILD_DIR    Vendor/FreeRTOS/builds/LP_MSPM0G3507/release)
set(FREERTOS_POSIX_DIR    Vendor/mspm0_sdk/ti/posix/freertos)

set(FREERTOS_SOURCES
        # Core
        ${FREERTOS_CORE_DIR}/croutine.c
        ${FREERTOS_CORE_DIR}/event_groups.c
        ${FREERTOS_CORE_DIR}/list.c
        ${FREERTOS_CORE_DIR}/queue.c
        ${FREERTOS_CORE_DIR}/stream_buffer.c
        ${FREERTOS_CORE_DIR}/tasks.c
        ${FREERTOS_CORE_DIR}/timers.c
        # Memory management
        ${FREERTOS_MEM_DIR}/heap_4.c
        # GCC Cortex-M0+ port
        ${FREERTOS_PORT_DIR}/port.c
        ${FREERTOS_PORT_DIR}/portasm.c
        # DPL
        ${FREERTOS_DPL_DIR}/AppHooks_freertos.c
        ${FREERTOS_DPL_DIR}/ClockP_freertos.c
        ${FREERTOS_DPL_DIR}/DebugP_freertos.c
        ${FREERTOS_DPL_DIR}/MutexP_freertos.c
        ${FREERTOS_DPL_DIR}/SemaphoreP_freertos.c
        ${FREERTOS_DPL_DIR}/SystemP_freertos.c
        ${FREERTOS_DPL_DIR}/StaticAllocs_freertos.c
        ${FREERTOS_DPL_DIR}/TaskP_freertos.c
        ${FREERTOS_DPL_DIR}/HwiPMSPM0_freertos.c
        # POSIX
        ${FREERTOS_POSIX_DIR}/clock.c
        ${FREERTOS_POSIX_DIR}/memory.c
        ${FREERTOS_POSIX_DIR}/mqueue.c
        ${FREERTOS_POSIX_DIR}/pthread_barrier.c
        ${FREERTOS_POSIX_DIR}/pthread_cond.c
        ${FREERTOS_POSIX_DIR}/pthread.c
        ${FREERTOS_POSIX_DIR}/pthread_mutex.c
        ${FREERTOS_POSIX_DIR}/pthread_rwlock.c
        ${FREERTOS_POSIX_DIR}/sched.c
        ${FREERTOS_POSIX_DIR}/semaphore.c
        ${FREERTOS_POSIX_DIR}/sleep.c
        ${FREERTOS_POSIX_DIR}/timer.c
        ${FREERTOS_POSIX_DIR}/aeabi_portable.c
)

# ============================================================
# 应用程序源文件（使用 GLOB_RECURSE 自动收集）
# ============================================================
# 收集 Core, BSP, APP, Vendor/SEGGER 等目录下的所有 .c 和 .s 文件
# 注意排除 FreeRTOS 目录，因为已经单独列出
file(GLOB_RECURSE APP_SOURCES_RECURSE
        "${CMAKE_CURRENT_SOURCE_DIR}/Core/*.c"
        #"${CMAKE_CURRENT_SOURCE_DIR}/Core/*.s"
        "${CMAKE_CURRENT_SOURCE_DIR}/BSP/*.c"
        #"${CMAKE_CURRENT_SOURCE_DIR}/BSP/*.s"
        "${CMAKE_CURRENT_SOURCE_DIR}/APP/*.c"
        #"${CMAKE_CURRENT_SOURCE_DIR}/APP/*.s"
        "${CMAKE_CURRENT_SOURCE_DIR}/Vendor/SEGGER/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Vendor/SYSVIEWFreeRTOSV11/*.c"
)

# 添加启动文件（特殊）
set(STARTUP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/Vendor/target/gcc/startup_mspm0g350x_gcc.c")
list(APPEND APP_SOURCES_RECURSE ${STARTUP_FILE})

# 添加 SysConfig 生成的文件（可能尚未生成，但已声明）
list(APPEND APP_SOURCES_RECURSE ${SYSCFG_OUTPUT})

# ============================================================
# 创建可执行目标
# ============================================================
add_executable(${PROJECT_NAME}
        ${FREERTOS_SOURCES}
        ${APP_SOURCES_RECURSE}
)

# ============================================================
# 添加头文件包含目录（递归包含所有需要的目录）
# ============================================================
# 首先添加一些基础目录（必须优先包含）
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/Core
        ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/SEGGER/RTT
        ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/SEGGER/Config
        ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/SEGGER/log
        ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/mspm0_sdk
        ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/mspm0_sdk/third_party/CMSIS/Core/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/mspm0_sdk/ti/posix/gcc
        ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/FreeRTOS/Source/include
        ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/FreeRTOS/Source/portable/GCC/ARM_CM0
        ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/FreeRTOS/builds/LP_MSPM0G3507/release
)

# 然后递归包含 BSP, APP, Core 的子目录（但要注意不要重复包含 FreeRTOS 内部头文件，但无害）
include_sub_directories_recursively(${CMAKE_CURRENT_SOURCE_DIR}/BSP)
include_sub_directories_recursively(${CMAKE_CURRENT_SOURCE_DIR}/APP)
include_sub_directories_recursively(${CMAKE_CURRENT_SOURCE_DIR}/Core)
include_sub_directories_recursively(${CMAKE_CURRENT_SOURCE_DIR}/Vendor/SYSVIEWFreeRTOSV11)
# 也可以包含 Vendor/mspm0_sdk 下的某些目录，但可能会很多，建议只包含必要的，或者按需添加。
# 如果您的代码引用了 mspm0_sdk 内部头文件，可以递归包含它，但会拉入很多。
# 这里我们保守一些，只包含上面已列出的。

# ============================================================
# 预构建的 DriverLib
# ============================================================
set(DRIVERLIB_PATH
        ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/mspm0_sdk/ti/driverlib/lib/gcc/m0p/mspm0g1x0x_g3x0x/driverlib.a
)

target_link_libraries(${PROJECT_NAME} PRIVATE
        ${DRIVERLIB_PATH}
        -lm
        -lc
        -lnosys
        -lgcc
)

# ============================================================
# 链接器脚本和链接选项
# ============================================================
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/target/gcc/mspm0g3507.lds)

target_link_options(${PROJECT_NAME} PRIVATE
        -Wl,-T,${LINKER_SCRIPT}
        -Wl,-Map,${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
        -Wl,--gc-sections,--print-memory-usage
)

# ============================================================
# 后处理：生成 .hex 和 .elf，打印大小
# ============================================================
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O elf32-littlearm
        $<TARGET_FILE:${PROJECT_NAME}>
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.elf
        COMMAND ${CMAKE_OBJCOPY} -O ihex
        $<TARGET_FILE:${PROJECT_NAME}>
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
        COMMENT "Generating hex file and printing size"
)