cmake_minimum_required(VERSION 3.20)
project(electric-competition-robot C ASM)

# ============================================================
# MCU and compiler flags
# (CPU architecture flags are set in the toolchain file)
# ============================================================
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -g -gstrict-dwarf -ffunction-sections -fdata-sections -Wall")

add_compile_definitions(__MSPM0G3507__)

# ============================================================
# Include directories
# ============================================================
set(PROJECT_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}                          # .
    ${CMAKE_CURRENT_SOURCE_DIR}/Core                     # Core (main, board, tasks)
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Motor                # BSP headers
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Sensor
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Display
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Comm
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Control
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/System
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/UI
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Periph
    ${CMAKE_CURRENT_SOURCE_DIR}/APP                      # APP headers
    ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/SEGGER/RTT          # SEGGER RTT
    ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/SEGGER/Config       # SEGGER RTT config
    ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/SEGGER/log          # SEGGER log
    ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/mspm0_sdk           # TI SDK root
    ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/mspm0_sdk/third_party/CMSIS/Core/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/mspm0_sdk/ti/posix/gcc  # POSIX compat headers
    ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/FreeRTOS/Source/portable/GCC/ARM_CM0
    ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/FreeRTOS/builds/LP_MSPM0G3507/release
)

# ============================================================
# SysConfig generated files (pre-build step)
# ============================================================
set(SYSCFG_OUTPUT
    ${CMAKE_CURRENT_SOURCE_DIR}/ti_msp_dl_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ti_msp_dl_config.h
)

# Optionally run SysConfig when robot.syscfg changes.
# This requires TI SysConfig CLI to be installed and in PATH.
# If SysConfig is not available, the previously-generated files are used as-is.
find_program(SYSCONFIG_CLI sysconfig_cli.bat
    PATHS "D:/TI/SYSCONFIG"
    NO_DEFAULT_PATH
)
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
# FreeRTOS kernel source files
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
    # DPL (Driver Porting Layer)
    ${FREERTOS_DPL_DIR}/AppHooks_freertos.c
    ${FREERTOS_DPL_DIR}/ClockP_freertos.c
    ${FREERTOS_DPL_DIR}/DebugP_freertos.c
    ${FREERTOS_DPL_DIR}/MutexP_freertos.c
    ${FREERTOS_DPL_DIR}/SemaphoreP_freertos.c
    ${FREERTOS_DPL_DIR}/SystemP_freertos.c
    ${FREERTOS_DPL_DIR}/StaticAllocs_freertos.c
    ${FREERTOS_DPL_DIR}/TaskP_freertos.c
    ${FREERTOS_DPL_DIR}/HwiPMSPM0_freertos.c
    # POSIX compatibility layer
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
# Application source files
# ============================================================
set(APP_SOURCES
    # Startup (GCC version — C, not assembly)
    Vendor/target/gcc/startup_mspm0g350x_gcc.c
    # Main
    Core/main.c
    Core/app_tasks.c
    Core/board.c
    # SysConfig output
    ${SYSCFG_OUTPUT}
    # BSP
    BSP/Display/OLED.c
    BSP/Display/OLED_Data.c
    BSP/Periph/KEY.c
    BSP/Sensor/trace.c
    BSP/System/misc.c
    BSP/Control/PID.c
    BSP/System/dwt.c
    BSP/System/daemon.c
    BSP/UI/menu.c
    BSP/System/fifo.c
    BSP/Motor/ZDT_Motor.c
    BSP/Comm/K230.c
    BSP/Sensor/icm42688.c
    BSP/Sensor/No_Mcu_Ganv_Grayscale_Sensor.c
    BSP/Motor/encoder_timer.c
    BSP/Motor/encoder.c
    BSP/Motor/dcmotor.c
    BSP/Sensor/IMU.c
    # APP
    APP/Robot_Cmd.c
    APP/Chassis.c
    APP/Robot.c
    APP/Gimbal.c
    # SEGGER
    Vendor/SEGGER/RTT/SEGGER_RTT.c
    Vendor/SEGGER/RTT/SEGGER_RTT_printf.c
    Vendor/SEGGER/log/bsp_log.c
)

# ============================================================
# Executable target
# ============================================================
add_executable(${PROJECT_NAME}
    ${FREERTOS_SOURCES}
    ${APP_SOURCES}
)

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_INCLUDE_DIRS})

# ============================================================
# Prebuilt DriverLib
# ============================================================
set(DRIVERLIB_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/mspm0_sdk/ti/driverlib/lib/gcc/m0p/mspm0g1x0x_g3x0x/driverlib.a
)
set(SYSLIB_DIR
    ${TOOLCHAIN_ROOT}/arm-none-eabi/lib/thumb/v6-m/nofp
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${DRIVERLIB_PATH}
    -lm
    -lc
    -lnosys
    -lgcc
)

# ============================================================
# Linker configuration
# ============================================================
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/target/gcc/mspm0g3507.lds)

target_link_options(${PROJECT_NAME} PRIVATE
    -Wl,-T,${LINKER_SCRIPT}
    -Wl,-Map,${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
    -Wl,--gc-sections,--print-memory-usage
)

# ============================================================
# Post-build: generate .hex and print size
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
