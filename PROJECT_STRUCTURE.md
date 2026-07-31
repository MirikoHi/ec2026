# 电赛训练项目 - 项目结构文档

## 1. 项目概述

本项目是一个基于 **TI MSPM0G3507** 微控制器的竞赛机器人固件，运行 **FreeRTOS** 实时操作系统。机器人为**差分驱动底盘 + 二轴云台**结构，搭载 AI 视觉模块（K230），具备巡线、IMU 姿态控制、视觉目标跟踪与自动射击等功能。

### 硬件平台

| 项目 | 规格 |
|------|------|
| MCU | TI MSPM0G3507 (ARM Cortex-M0+, LQFP-64) |
| Flash | 128 KB |
| SRAM | 32 KB |
| 主频 | 80 MHz (40 MHz 外部晶振 + PLL) |
| 调试接口 | SWD (PA20/PA19), J-Link 4 MHz |
| IDE | CLion (CMake + Ninja) |
| 工具链 | arm-none-eabi-gcc 14.3 |
| 配置工具 | TI SysConfig |

### 软件栈

```
┌─────────────────────────────────────────────┐
│                 APP 层                       │
│  Robot | Chassis | Gimbal | Robot_Cmd       │
├─────────────────────────────────────────────┤
│                 BSP 驱动层                   │
│  dcmotor | encoder | ICM42688 | IMU | OLED  │
│  ZDT_Motor | K230 | trace | PAW3395 | PID   │
│  daemon | dwt | menu | fifo | bsp_log       │
├─────────────────────────────────────────────┤
│              FreeRTOS 内核                    │
│  任务调度 | 队列 | 信号量 | 定时器 | DPL     │
├─────────────────────────────────────────────┤
│           TI MSPM0 SDK (driverlib)           │
│  GPIO | UART | SPI | I2C | Timer | ADC      │
├─────────────────────────────────────────────┤
│              硬件 (MSPM0G3507)               │
└─────────────────────────────────────────────┘
```

### 机器结构参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `CHASSIS_LENGTH_TO_CENTER` | 0.166 m | 光流传感器到车体中心的距离 |
| `GIMBAL_LENGTH_TO_CENTER` | 0.66 m | 云台旋转中心到目标的距离 |
| `PulseofCircle` | 1733 | 每圈编码器脉冲数 (13×10×40/3) |
| 轮径 | 0.065 m | 车轮直径 |
| 减速比 | 30:1 | 电机减速比 |

---

## 2. 目录结构

```
electric-competition-training/
├── CMakeLists.txt              # 顶层 CMake 构建脚本
├── README.md                   # TI 官方 Blinky 示例说明（已过时）
├── robot.syscfg            # TI SysConfig 项目配置（引脚/时钟/外设）
├── ti_msp_dl_config.c/.h       # SysConfig 生成的驱动库配置
├── Core/                        # 程序核心（入口 & 板级初始化）
│   ├── main.c                  # 程序入口 (main 函数)
│   ├── Core/app_tasks.c             # FreeRTOS 任务创建（主应用）
│   └── board.c / board.h       # 板级初始化与延时函数
│
├── APP/                        # 应用层
│   ├── Robot.c / Robot.h       # 系统初始化总入口
│   ├── Chassis.c / Chassis.h   # 底盘控制（巡线/转向/位置模式）
│   ├── Gimbal.c / Gimbal.h     # 云台控制（视觉跟踪/射击）
│   └── Robot_Cmd.c / Robot_Cmd.h # 命令队列 + Gimbal PID 初始化
│
├── BSP/                        # 板级支持包（驱动层）
│   ├── Motor/                  # 电机驱动
│   │   ├── dcmotor.c/h         # 直流电机驱动（级联 PID）
│   │   ├── encoder.c/h         # 编码器正交解码
│   │   ├── encoder_timer.c/h   # 编码器定时更新
│   │   ├── ZDT_Motor.c/h       # ZDT 步进电机驱动
│   │   └── bsp_motor.c/h       # 备用电机驱动（旧版）
│   ├── Sensor/                 # 传感器
│   │   ├── icm42688.c/h        # ICM42688 6轴 IMU (SPI)
│   │   ├── IMU.c/h             # Madgwick AHRS 姿态解算
│   │   ├── JY901S.c/h          # JY901S 姿态传感器
│   │   ├── PAW3395.c/h         # PAW3395 光流传感器
│   │   ├── No_Mcu_Ganv_Grayscale_Sensor.c/.h # 灰度巡线传感器
│   │   └── trace.c/h           # 巡线 PID 控制
│   ├── Display/                # 显示
│   │   ├── OLED.c/h            # OLED 图形显示驱动
│   │   ├── OLED_Data.c/h       # OLED 帧缓冲与 I2C 操作
│   │   ├── oledfont.h          # 字库数据
│   │   └── bsp_oled.c/h        # 备用 OLED 驱动（旧版）
│   ├── Comm/                   # 通信
│   │   ├── K230.c/h            # K230 AI 视觉模块 (UART, steel_ball_movement)
│   │   ├── NRF24L01.c/h        # NRF24L01 2.4G 无线模块
│   │   └── Emm_V5.c/h          # EMM V5 模块
├── lichee_rec/             # LiChee RV Nano 识别模块
│   │   ├── lichee_rec.c/h      # LiChee 通信协议解析 (0xA5/CRC8)
├── Algorithm/              # 算法库
│   │   ├── crc8.c/h            # CRC-8 校验 (含 CRC-8/MAXIM)
│   ├── Control/                # 控制算法
│   │   ├── PID.c/h             # 通用 PID 控制器
│   │   └── bsp_pid.c/h         # 速度 PI 控制器（旧版）
│   ├── System/                 # 系统工具
│   │   ├── daemon.c/h          # 心跳/超时监控守护进程
│   │   ├── dwt.c/h             # DWT 高精度周期计时器
│   │   ├── fifo.c/h            # 环形缓冲区 (FIFO)
│   │   └── misc.c/h            # 杂项工具函数
│   ├── UI/                     # 用户界面
│   │   └── menu.c/h            # OLED 三级菜单系统
│   └── Periph/                 # 外设驱动
│       ├── KEY.c/h             # 按键驱动
│       ├── bsp_key.c/h         # 备用按键驱动
│       ├── LED.c/h             # LED 控制
│       ├── Servo.c/h           # 舵机驱动
│       ├── ADC_Voltage.c/h     # ADC 电压测量
│       ├── bsp_beep.c/h        # 蜂鸣器
│       ├── bsp_gyro.c/h        # 陀螺仪
│       ├── bsp_track.c/h       # 备用巡线
│       ├── ganwei.c/h          # 感威传感器
│       └── tjc.c/h             # TJC 串口屏
│
│
├── Vendor/                      # 第三方库和 SDK
│   ├── target/                  # MSPM0G3507 启动文件 + 链接脚本 (4种编译器)
│   │   ├── gcc/                 # GCC: startup + .lds
│   │   ├── keil/                # Keil MDK
│   │   ├── iar/                 # IAR
│   │   └── ticlang/             # TI Clang
│   ├── FreeRTOS/                # FreeRTOS 内核
│   │   ├── Source/              # 内核源码 + include + portable
│   │   ├── dpl/                 # 驱动移植层 (DPL)
│   │   └── builds/.../          # FreeRTOSConfig.h
│   ├── mspm0_sdk/               # TI MSPM0 SDK
│   │   ├── ti/driverlib/        # 驱动库 (预编译 .a)
│   │   ├── ti/posix/            # POSIX 兼容层
│   │   └── third_party/CMSIS/   # ARM CMSIS
│   └── SEGGER/                  # SEGGER RTT 日志
│       ├── RTT/                 # RTT 核心实现
│       ├── Config/              # RTT 配置
│       └── log/                 # 日志宏封装 (bsp_log)
│
├── cmake/
│   └── toolchain-arm-none-eabi.cmake # CMake 工具链文件
│
├── Clion_project.jdebug        # SEGGER Ozone 调试配置 (CLion)
└── .idea/                      # CLion IDE 配置
```

---

## 3. 启动流程

### 3.1 从复位到 main()

```
复位 (Reset_Handler)
  │
  ├─ 加载栈指针 (__StackTop = 0x20208000)
  ├─ 复制 .data  段 (Flash → SRAM)
  ├─ 复制 .ramfunc 段 (Flash → SRAM)
  ├─ 清零 .bss 段
  ├─ 调用 __libc_init_array() (C++ 静态构造)
  └─ 调用 main()
```

代码位于 `Vendor/target/gcc/startup_mspm0g350x_gcc.c`。

### 3.2 main() → 调度器启动

```
main()                                    [Core/main.c]
  │
  ├─ prvSetupHardware()
  │   └─ SYSCFG_DL_init()                [SysConfig 生成]
  │       ├─ 时钟树配置 (80 MHz)
  │       ├─ GPIO 初始化
  │       ├─ 外设时钟使能
  │       └─ 中断配置
  │
  └─ Robot_Init()                        [APP/Robot.c]
      ├─ __disable_irq()                 # 全局关中断
      ├─ DWT_Init(80)                    # 高精度定时器初始化 (80MHz)
      ├─ MenuInit()                      # 三级菜单初始化
      ├─ Chassis_Init()                  # 底盘初始化（电机+IMU）
      ├─ RobotCmd_Init()                 # 命令队列创建 + PID 初始化
      ├─ main_blinky()                   # 创建所有 FreeRTOS 任务 [Core/app_tasks.c]
      ├─ K230_Init()                     # AI 视觉模块初始化
      └─ vTaskStartScheduler()           # 启动 FreeRTOS 调度器 (永不返回)
```

### 3.3 内存布局

```
SRAM (0x20200000 - 0x20208000, 32KB):
┌─────────────────────┐ 0x20208000  __StackTop
│       .stack        │ ← 向下增长 (最小 128 字节)
├─────────────────────┤
│       .heap         │ ← FreeRTOS 堆 (3072 字节)
├─────────────────────┤
│      .noinit        │
├─────────────────────┤
│       .bss          │ ← 零初始化
├─────────────────────┤
│       .data         │ ← 从 Flash 复制
├─────────────────────┤
│      .ramfunc       │ ← 从 Flash 复制 (RAM 中执行)
├─────────────────────┤
│      .vtable        │ ← RAM 向量表
└─────────────────────┘ 0x20200000

Flash (0x00000000 - 0x00020000, 128KB):
┌─────────────────────┐
│    .BSLConfig       │ 0x41C00100
├─────────────────────┤
│    .BCRConfig       │ 0x41C00000
├─────────────────────┤
│      .rodata        │
├─────────────────────┤
│       .text         │
├─────────────────────┤
│     .intvecs        │ ← 中断向量表
└─────────────────────┘ 0x00000000
```

---

## 4. 构建系统

### 4.1 CMake 配置

- **文件**: `CMakeLists.txt` + `cmake/toolchain-arm-none-eabi.cmake`
- **CMake 最低版本**: 3.20
- **项目名**: `electric-competition-robot`
- **编译选项**: `-O2 -g -gstrict-dwarf -ffunction-sections -fdata-sections -Wall`
- **链接选项**: `-Wl,--gc-sections -static --specs=nano.specs -nostartfiles`
- **CPU**: `cortex-m0plus`, `armv6-m`, `thumb`, soft float
- **输出**: ELF → HEX (arm-none-eabi-objcopy), Size 报告 (arm-none-eabi-size)

### 4.2 源文件分组

| 组 | 内容 |
|----|------|
| FREERTOS_SOURCES | 内核 (tasks/queue/timers/event_groups 等) + heap_4 + GCC 移植 + DPL + POSIX 层 |
| APP_SOURCES | 启动文件 + main + board + BSP + APP + SysConfig + SEGGER RTT |
| 预编译库 | `driverlib.a` (TI MSPM0 驱动库) |
| 系统库 | `-lm -lc -lnosys -lgcc` |

### 4.3 SysConfig 集成

构建前可选步骤：如果检测到 `D:/TI/SYSCONFIG/sysconfig_cli.bat`，则从 `robot.syscfg` 重新生成 `ti_msp_dl_config.c/h`。否则使用预生成的版本。

---

## 5. 硬件配置

### 5.1 系统时钟

```
外部 40MHz HFXT 晶振
  └─ PLL 使能
      └─ SYSPLL / 2
          └─ 80 MHz 系统时钟 (MCLK)
```

### 5.2 引脚分配

#### 电机控制

| 功能 | 引脚 | 外设 |
|------|------|------|
| 左电机 PWM | PB10 | TIMG0 (PWM, 反相) |
| 右电机 PWM | PB11 | TIMG0 (PWM, 反相) |
| 左电机方向 A | PA7 | GPIO |
| 左电机方向 B | PA21 | GPIO |
| 右电机方向 A | PA18 | GPIO |
| 右电机方向 B | PB14 | GPIO |
| 左编码器 A/B | PB2/PB3 | GPIO (外部中断, 优先级3) |
| 右编码器 A/B | PB18/PB19 | GPIO (外部中断, 优先级3) |
| 继电器 | PB4 | GPIO |

#### 传感器

| 功能 | 引脚 | 外设 |
|------|------|------|
| IMU (ICM42688) SPI SCK | PB23 | SPI1 |
| IMU SPI MOSI | PB8 | SPI1 |
| IMU SPI MISO | PB21 | SPI1 |
| IMU SPI CS | PA29 | GPIO (片选) |
| 灰度传感器地址 | PA24, PB25, PB24 | GPIO |
| ADC 采样 | PA22 | ADC12, Ch8 |

#### 显示与交互

| 功能 | 引脚 | 外设 |
|------|------|------|
| OLED I2C SCL | PA1 | I2C |
| OLED I2C SDA | PA0 | I2C |
| 按键 1-4 | PA30, PA31, PB0, PB1 | GPIO (上拉输入) |
| 蜂鸣器 | PA14 | GPIO |

#### 云台（步进电机）

| 功能 | 引脚 | 外设 |
|------|------|------|
| Yaw Step | PA15 | GPIO |
| Yaw Dir | PA8 | GPIO |
| Pitch Step | PA17 | GPIO |
| Pitch Dir | PA9 | GPIO |

#### 通信

| 功能 | 引脚 | 外设 |
|------|------|------|
| K230 UART RX/TX | PB16/PB15 | UART2, 115200 |   //这个是准的  代码里面的逻辑也没有问题  即插即用
| UART1 RX/TX | PB7/PB6 | UART1, 115200 |
| UART0 RX/TX | PB13/PB12 | UART0, 115200 |

#### 调试

| 功能 | 引脚 | 外设 |
|------|------|------|
| SWD CLK | PA20 | DEBUGSS |
| SWD IO | PA19 | DEBUGSS |

### 5.3 定时器分配

| 定时器 | 用途 | 周期 |
|--------|------|------|
| TIMA0 | 系统滴答 (TIMER_TICK) | 10 ms, 中断优先级3 |
| TIMG0 | 电机 PWM | 周期 2500 |
| TIMG6 | 步进电机脉冲 | 0.3 ms |
| TIMG7 | 编码器数据更新中断 | - |
| TIMG12 | DWT 高精度计时 | ~53.7 秒周期 |

---

## 6. FreeRTOS 配置与任务

### 6.1 关键配置 (`FreeRTOSConfig.h`)

| 参数 | 值 | 说明 |
|------|-----|------|
| `configCPU_CLOCK_HZ` | 32,000,000 | 系统时钟 (Hz) |
| `configTICK_RATE_HZ` | 1000 | 滴答频率 (1ms) |
| `configUSE_PREEMPTION` | 1 | 抢占式调度 |
| `configUSE_TIME_SLICING` | 0 | 禁用时间片轮转 |
| `configUSE_TICKLESS_IDLE` | 1 | 低功耗 tickless 模式 |
| `configMAX_PRIORITIES` | 10 | 优先级 0-9 |
| `configMINIMAL_STACK_SIZE` | 128 字 (512 字节) | 最小栈 |
| `configTOTAL_HEAP_SIZE` | 3072 字节 | FreeRTOS 堆 (heap_4) |
| `configCHECK_FOR_STACK_OVERFLOW` | 2 | 栈溢出检测方法2 |
| `configUSE_TIMERS` | 1 | 软件定时器 |
| `configTIMER_TASK_PRIORITY` | 9 (最高) | 定时器任务优先级 |
| `configUSE_MUTEXES` | 1 | 互斥锁 |
| `configUSE_COUNTING_SEMAPHORES` | 1 | 计数信号量 |
| `configUSE_EVENT_GROUPS` | 1 | 事件组 |
| `configUSE_TASK_NOTIFICATIONS` | 1 | 任务通知 |
| `configPRIO_BITS` | 2 | 硬件实现 4 级优先级 |

### 6.2 任务列表

所有任务在 `Core/app_tasks.c` 的 `main_blinky()` 中创建。

| 任务名 | 入口函数 | 优先级 | 栈 (字) | 周期 | 说明 |
|--------|----------|--------|---------|------|------|
| `Key` | `KeyTask` | 1 (IDLE+1) | 128 | 1 ms | 按键轮询 + 菜单刷新 |
| `HwMotor` | `HwMotorTask` | 2 (IDLE+2) | 256 | 10 ms | 直流电机控制循环 |
| `RobotCmd` | `RobotCmdTask` | 2 (IDLE+2) | 128 | 5 ms | 命令处理 + PID 计算 |
| `Chassis` | `ChassisTask` | 2 (IDLE+2) | 256 | 5 ms | 底盘控制（巡线/IMU） |
| `Gimbal` | `GimbalTask` | 2 (IDLE+2) | 128 | 5 ms | 云台控制（视觉跟踪） |
| `Daemon` | `DaemonTask` | 0 (IDLE) | 128 | 10 ms | 守护进程/超时监控 |

**已注释的任务**（不在当前构建中）:

| 任务名 | 周期 | 说明 |
|--------|------|------|
| `Trace` | 5 ms | 巡线传感器调试输出 |
| `StepMotor` | 1 ms | 步进电机独立控制 |
| `NRF24L01` | 1 ms | 无线通信 |
| `Rx` / `TX` | 阻塞 / 1s | FreeRTOS 演示任务 (队列收发) |

### 6.3 任务间通信 - 队列

```
                      ┌──────────────┐
                      │  RobotCmd    │ (每 5ms)
                      │  Task        │
                      └──┬───┬───┬──┘
                         │   │   │
              ┌──────────┘   │   └──────────┐
              ▼              ▼              ▼
   ┌─────────────────┐ ┌──────────────┐ ┌─────────────────┐
   │ chassis_cmd_q   │ │gimbal_cmd_q  │ │trace_fetch_data │
   │ (深度4)          │ │(深度4)        │ │_queue (深度4)    │
   └────────┬────────┘ └──────┬───────┘ └────────┬────────┘
            ▼                 ▼                  ▲
   ┌─────────────────┐ ┌──────────────┐          │
   │  Chassis Task   │ │  Gimbal Task │  [外部数据源]
   │  (每 5ms)        │ │  (每 5ms)     │
   └─────────────────┘ └──────────────┘
   chassis_cmd_queue  gimbal_cmd_queue

   还有:
   ┌─────────────────────┐
   │chassis_fetch_data_q │ (深度4, 占位)
   └─────────────────────┘
```

**队列数据结构**:

- `chassis_cmd_q`: `{ Chassis_Mode_e Chassis_Mode; uint8_t circle_set; }`
- `gimbal_cmd_q`: `{ float yaw; float pitch; float aim_x; float aim_y; uint8_t relay_on_flag; uint8_t task_flag; }`
- `trace_fetch_data_q`: `{ float pid_output; }`

---

## 7. 模块详解

---

### 7.1 APP 层

---

#### 7.1.1 Robot — 系统初始化总入口

**文件**: `APP/Robot.c`, `APP/Robot.h`

**职责**: 按顺序初始化所有子系统，最后启动 FreeRTOS 调度器。

**API**:

| 函数 | 说明 |
|------|------|
| `void Robot_Init(void)` | 系统总初始化，调用各子系统 Init，最后启动调度器 |

**初始化序列**:
1. `__disable_irq()` — 关全局中断
2. `DWT_Init(80)` — 初始化 80MHz 高精度定时器
3. `MenuInit()` — 初始化 OLED 三级菜单
4. `Chassis_Init()` — 初始化左右直流电机 + IMU
5. `RobotCmd_Init()` — 创建 4 个 FreeRTOS 队列 + 3 个 PID 控制器
6. `main_blinky()` — 创建所有 FreeRTOS 任务
7. `K230_Init()` — 初始化 AI 视觉模块 UART 通信
8. `vTaskStartScheduler()` — 启动调度器 (永不返回)

---

#### 7.1.2 Chassis — 底盘控制

**文件**: `APP/Chassis.c`, `APP/Chassis.h`

**职责**: 差速驱动底盘的运动控制，支持四种工作模式，包含分段导航状态机。

**枚举**:

```c
typedef enum {
    Chassis_Line,   // 直线运动
    Chassis_Turn,   // 旋转运动
    Chassis_Stop    // 停止
} Chassis_Move_State_e;
```

**底盘模式**（定义在 `Robot_Cmd.h`）:

```c
typedef enum {
    NORMAL_MODE = 0,  // 普通模式（开环调试）
    TRACE_MODE,       // 巡线模式（灰度传感器 + 分段导航）
    IMU_MODE,         // IMU 模式（姿态读取）
    POSITION_MODE     // 位置模式（预留）
} Chassis_Mode_e;
```

**API**:

| 函数 | 说明 |
|------|------|
| `void Chassis_Init(void)` | 初始化左右直流电机 (DCMotor) + IMU |
| `void Chassis(void)` | 底盘主控制循环 (由 ChassisTask 每 5ms 调用) |
| `void Motor_Cmd_CallBack(uint8_t i)` | 电机使能/失能切换 |
| `void Chassis_get_init_angle(void)` | 获取初始角度 |

**巡线模式控制流程** (`Chassis` 中的 `TRACE_MODE`):

```
1. Chassis_State_Turn()     ← 分段导航状态机
2. Trace_task()             ← 读取灰度传感器，输出 PID 修正
3. DCMotor_SetTraceCompensation(左轮, -修正)
4. DCMotor_SetTraceCompensation(右轮, +修正)  ← 差速转向修正
5. Stop_Detect()            ← 检测到达目标位置
```

**分段导航状态机** (`Chassis_State_Turn`):
一圈路径由 4 条直线和 4 个转弯组成（矩形巡线），状态 0-9 循环，支持 `circle_set` 圈数设定：

| 状态 | 动作 | 转移条件 |
|------|------|----------|
| 0 | 圈数判断，前进 0.08m | → 状态 1 |
| 1 | 右转 (左轮 -0.01, 右轮 +0.01) | Stop_Flag |
| 2 | 前进 0.082m | Spin_succeed_flag |
| 3 | 右转 | Stop_Flag |
| 4 | 前进 0.082m | Spin_succeed_flag |
| 5 | 右转 | Stop_Flag |
| 6 | 前进 0.082m | Spin_succeed_flag |
| 7 | 右转 | Stop_Flag |
| 8 | 前进 0.01m (短收尾) | Spin_succeed_flag |
| 9 | 圈数++, 重置到 0 | Stop_Flag |

**停止检测** (`Stop_Detect`):
- **直线完成**: 两轮位置误差同时 < 0.008m 持续 40 个周期
- **旋转完成**: 计数 200 个周期 + 轮位置误差 < 0.008m

**依赖**: `dcmotor.h`, `trace.h`, `IMU.h`, `JY901S.h`, `dwt.h`, `misc.h`

---

#### 7.1.3 Gimbal — 云台控制 (ZDT 电机滑槽小球闭环)

**文件**: `APP/Gimbal.c`, `APP/Gimbal.h`

**职责**: 基于 ZDT_Emm 步进电机的滑槽小球位置闭环控制。通过 K230 视觉回传的钢球位置数据，经 PID 控制器 + 速度前馈 + 底盘加速度前馈，计算舵机/步进电机角度，控制滑槽倾角使小球保持在目标位置。

**控制策略**:
1. `K230_Read()` 获取钢球位置 `x_position` (0~640) 和帧间隔 `dt`
2. 坐标一阶低通滤波 → 速度估计 (Δx/dt) → 速度低通滤波
3. 位置误差 → PID (Kp=0.1244, Kd=0.0137, Ki=0.00001) → 基础角度
4. 速度前馈 (SLIDE_VEL_FF_GAIN=0.5) + 底盘加速度前馈 (SLIDE_ACC_GAIN=50)
5. 合成角度限幅 ±45° → ZDT_Emm_Pos_Control 输出脉冲

**任务模式** (`task_flag` 驱动):
| 任务 | 行为 |
|------|------|
| 0 | 复位模式: 目标位置 = 312 (画面中心) |
| 3 | 静态滚球 ±5cm: 目标在 180/440 间切换, 到位后触 `car_stop` |
| 6 | 指定位置: 从 LiCheeRec 获取 `relative_position` 作为目标 |

**API**:

| 函数 | 说明 |
|------|------|
| `void Gimbal_Init(void)` | 初始化 ZDT_Emm 电机 UART + 滑槽 PID 控制器 |
| `void Gimbal(void)` | 200 Hz 控制循环: 接收命令 + Slide_Control_Run |
| `void Gimbal_Attitude_Solving(void)` | 笛卡尔坐标 → 云台角度逆运动学解算 |
| `void Slider_Set_Pos_Pixel(uint16_t)` | 设置滑槽目标位置 (像素) |
| `void UART3_IRQHandler(void)` | ZDT 步进电机 UART 接收中断 |

**依赖**: `ZDT_Emm.h`, `ZDT_Motor.h`, `K230.h`, `lichee_rec.h`, `dcmotor.h`, `PID.h`, `dwt.h`

---

#### 7.1.4 Robot_Cmd — 命令队列与 PID

**文件**: `APP/Robot_Cmd.c`, `APP/Robot_Cmd.h`

**职责**: 管理 FreeRTOS 命令队列，初始化 Gimbal PID 控制器，运行命令分发循环。

**导出的队列句柄**:

```c
extern QueueHandle_t chassis_cmd_queue;        // 底盘命令 (chassis_cmd_q, 深度4)
extern QueueHandle_t chassis_fetch_data_queue; // 底盘数据获取 (占位)
extern QueueHandle_t trace_fetch_data_queue;   // 巡线数据获取 (trace_fetch_data_q, 深度4)
extern QueueHandle_t gimbal_cmd_queue;         // 云台命令 (gimbal_cmd_q, 深度4)
```

**Gimbal PID 配置** (在 `RobotCmd_Init` 中初始化):

| PID | Kp | Kd | Ki | Max Out | 模式 |
|-----|-----|-----|-----|---------|------|
| gimbal_yaw_PID | 0.003 | 0.0001 | 0.0 | 4.0 | POSITION |
| gimbal_pitch_PID | -0.003 | -0.0001 | 0.0 | 4.0 | POSITION |
| gimbal_yaw_forwardfeed_PID | 0.0 | -0.0001 | 0.0 | 4.0 | POSITION |

**API**:

| 函数 | 说明 |
|------|------|
| `void RobotCmd_Init(void)` | 创建 4 个队列 + 初始化 3 个 PID |
| `void Robot_Cmd(void)` | 主控制循环: 读取巡线数据 → PID 计算 → 发送到 Chassis/Gimbal 队列 |
| `void Control_Switch_Callback(uint8_t i)` | 控制开关回调 |
| `void Task_Callback(uint8_t i)` | 设置 gimbal task_flag (1 或 2) |
| `void Chassis_Mode_Switch_Callback(uint8_t i)` | 底盘模式切换 (0=普通/1=巡线/2=IMU/3=位置) |
| `static void Gimbal_Pid_Cal(void)` | K230 误差 → 位置 PID → 累积到云台角度 |
| `static void draw_sin(void)` | 正弦轨迹测试模式 |

---

### 7.2 BSP 驱动层

---

#### 7.2.1 dcmotor — 直流电机驱动

**文件**: `BSP/dcmotor.c`, `BSP/dcmotor.h`

**职责**: 直流有刷电机抽象层，提供级联位置-速度 PID 控制。

**常量**:

```c
#define MOTOR_PWM_MAX         2499     // PWM 最大值 (2500-1)
#define Control_Period        10       // 控制周期 (ms)
#define PulseofCircle         1733     // 每圈脉冲数 (13*10*40/3)
#define ENCODER_TO_SPEED_MS   (100 * 0.065 * PI * 3 / 10 / 13 / 2 / 40)  // 脉冲 → m/s
#define ENCODER_TO_DISDAN_M   (0.065 * PI * 3 / 13 / 10 / 2 / 40)       // 脉冲 → 米
#define FILTER_NUM            20       // 速度滤波器窗宽
#define SPEED_SMOOTH_COEF     0.85     // 速度平滑系数
```

**核心结构体**:

```c
typedef struct {
    pid_type_def speed_pid;           // 速度 PID
    pid_type_def position_pid;        // 位置 PID
    ENCODER_RES *encoder;             // 编码器实例
    DCMotor_PortPin_s PortPin;        // 端口引脚
    Motor_Dir Input_Dir;              // 输入方向
    Motor_Dir Output_Dir;             // 输出方向
    uint8_t feedforward;              // 前馈使能
    Motor_Speed_Filter_e filter;      // 速度滤波器
    float speed_measure;              // 测量速度 (m/s)
    float speed_ref;                  // 目标速度 (m/s)
    float position_ref;               // 目标位置 (m)
    float position_measure;           // 测量位置 (m)
    float Trace_Compensation;         // 巡线修正量
    State State;                      // 电机使能状态
} DCMotorInstance;
```

**API**:

| 函数 | 说明 |
|------|------|
| `DCMotorInstance* DCMotor_Init(DCMotorInitConfig_s *config)` | 初始化电机实例（分配结构体、配置PID、编码器） |
| `void DCMotor_Cmd(DCMotorInstance* motor, State state)` | 使能/失能电机 |
| `void DCMotor_SetPosition(DCMotorInstance *motor, float Position)` | 设置目标位置 (米) |
| `void DCMotor_SetTraceCompensation(DCMotorInstance *motor, float compensation)` | 设置巡线差速修正 |
| `void Hw_Motor_Task(void)` | 主控制循环: 编码器更新 → 位置/速度计算 → 级联 PID → PWM 输出 |

**控制回路**:

```
Position_Ref ──→ [位置 PID] ──→ Speed_Ref ──→ [速度 PID] ──→ PWM ──→ 电机
                   ▲                            ▲
           Position_Measure             Speed_Measure
                   │                            │
                   └──────── 编码器 ────────────┘
```

`Hw_Motor_Task` 由 `HwMotorTask` 每 10ms 调用一次，包含：
1. 读取编码器累计脉冲 → 计算位置 & 速度
2. 速度滤波 (移动平均，窗宽 20)
3. 位置 PID → 速度 PID（级联结构）
4. 前馈补偿
5. 巡线修正 (Trace_Compensation) 叠加到输出
6. PWM 比较值写入硬件

---

#### 7.2.2 encoder — 编码器

**文件**: `BSP/encoder.c`, `BSP/encoder.h`, `BSP/encoder_timer.c`, `BSP/encoder_timer.h`

**职责**: 增量式编码器的正交解码（A/B 相）。

**核心结构体**:

```c
typedef struct {
    volatile int32_t temp_count;   // 中断累加 (临时)
    volatile int32_t count;        // 定时锁存值
    volatile int32_t total_count;  // 总累计
    ENCODER_DIR dir;               // 方向 FORWARD/REVERSAL
    encoder_PortPin_s PortPin;     // A/B 相引脚配置
} ENCODER_RES;
```

**API**:

| 函数 | 说明 |
|------|------|
| `ENCODER_RES* Encoder_Init(encoder_PortPin_s* init)` | 初始化编码器 |
| `void Encoder_InterruptBegin(void)` | 使能 GPIO 外部中断 |
| `void Encoder_Update(void)` | 定时锁存: temp_count → count (由 timer ISR 调用) |
| `void EncoderTimer_Init(void)` | 初始化编码器更新定时器 |

**正交解码逻辑** (`GROUP1_IRQHandler`):
- A 相上升沿/下降沿 + B 相电平判断 → 正转/反转 → temp_count++/--

---

#### 7.2.3 ICM42688 — 6 轴 IMU 驱动

**文件**: `BSP/icm42688.c`, `BSP/icm42688.h`

**职责**: ICM42688 六轴惯性传感器 (3 轴加速度 + 3 轴陀螺仪) 的 SPI 底层驱动。

**配置**:
- 加速度计: ±4g, 100Hz ODR
- 陀螺仪: ±1000dps, 100Hz ODR
- 通信: SPI 模式 1 (CPOL=1, CPHA=1)

**核心结构体**:

```c
typedef struct { int16_t x; int16_t y; int16_t z; } icm42688RawData_t;
typedef struct { float x; float y; float z; } icm42688RealData_t;
```

**API**:

| 函数 | 说明 |
|------|------|
| `int8_t bsp_Icm42688Init(void)` | 初始化 SPI + 验证 WhoAmI (0x47) |
| `int8_t bsp_Icm42688RegCfg(void)` | 配置寄存器 (量程、ODR、低噪声模式) |
| `int8_t bsp_IcmGetTemperature(int16_t* pTemp)` | 读取温度 |
| `int8_t bsp_IcmGetAccelerometer(icm42688RawData_t *accData)` | 读取加速度原始值 |
| `int8_t bsp_IcmGetGyroscope(icm42688RawData_t *GyroData)` | 读取陀螺仪原始值 |
| `int8_t bsp_IcmGetRawData(icm42688RealData_t* acc, icm42688RealData_t* gyro)` | 读取加速度+陀螺仪物理值 (mg, dps) |
| `float bsp_Icm42688GetAres(uint8_t Ascale)` | 加速度分辨率 |
| `float bsp_Icm42688GetGres(uint8_t Gscale)` | 陀螺仪分辨率 |

---

#### 7.2.4 IMU — 姿态解算

**文件**: `BSP/IMU.c`, `BSP/IMU.h`

**职责**: 基于 ICM42688 的姿态解算 (Madgwick AHRS 滤波器)。

**核心结构体**:

```c
typedef struct { float x; float y; float z; } xyz_f_t;
```

**API**:

| 函数 | 说明 |
|------|------|
| `void IMU_init(void)` | ICM42688 初始化 + 四元数归零 |
| `void IMU_getYawPitchRoll(float *ypr)` | 获取欧拉角 (yaw, pitch, roll) |
| `void IMU_TT_getgyro(float *zsjganda)` | 获取陀螺仪数据 |
| `void MPU6050_InitAng_Offset(void)` | 陀螺仪零偏校准 (遗留命名) |

**算法详情**:
- **陀螺仪校准**: 300 采样滑动窗口，方差 < 0.02 时锁定零偏
- **Madgwick 滤波器**: Kp=0.5, Ki=0.001，融合陀螺仪+加速度计（磁力计框架预留但未使用）
- **欧拉角提取**: 四元数 → atan2/asin 转换

---

#### 7.2.5 PAW3395 — 光流传感器

**文件**: `BSP/PAW3395.c`, `BSP/PAW3395.h`

**职责**: PAW3395 光流传感器的 SPI 驱动。

**API**:

| 函数 | 说明 |
|------|------|
| `void PAW3395_Init(void)` | ~148 条寄存器写入初始化 |
| `void PAW3395_Read_Motion(float* axis)` | 读取位移: axis[0]=dx(m), axis[1]=dy(m) |

**转换**: `位移(米) = delta_x / 5000 * 0.0254`

> **注意**: 该模块在当前固件中被注释掉，未激活。

---

#### 7.2.6 OLED — 图形显示驱动

**文件**: `BSP/OLED.c`, `BSP/OLED.h`, `BSP/OLED_Data.c`, `BSP/oledfont.h`

**职责**: 基于帧缓冲的 OLED 图形显示驱动，支持多种图形绘制和字体。

**API**:

| 函数 | 说明 |
|------|------|
| `void OLED_Init(void)` | 初始化 OLED (I2C + 帧缓冲) |
| `void OLED_Update(void)` | 全屏刷新 (帧缓冲 → 硬件) |
| `void OLED_UpdateArea(int16_t X, int16_t Y, uint8_t W, uint8_t H)` | 局部刷新 |
| `void OLED_Clear(void)` | 清屏 |
| `void OLED_ClearArea(int16_t X, int16_t Y, uint8_t W, uint8_t H)` | 局部清除 |
| `void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize)` | 显示单个字符 |
| `void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize)` | 显示字符串 |
| `void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)` | 显示无符号整数 |
| `void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize)` | 显示有符号整数 |
| `void OLED_ShowFloatNum(int16_t X, int16_t Y, double Number, uint8_t IntLen, uint8_t FraLen, uint8_t FontSize)` | 显示浮点数 |
| `void OLED_ShowHexNum(...)` / `OLED_ShowBinNum(...)` | 十六进制/二进制显示 |
| `void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...)` | 格式化输出 (类 printf) |
| `void OLED_DrawPoint(int16_t X, int16_t Y)` / `OLED_GetPoint(...)` | 像素点绘制/读取 |
| `void OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1)` | 直线 |
| `void OLED_DrawRectangle(int16_t X, int16_t Y, uint8_t W, uint8_t H, uint8_t Filled)` | 矩形 |
| `void OLED_DrawTriangle(...)` / `OLED_DrawCircle(...)` | 三角形/圆 |
| `void OLED_DrawEllipse(...)` / `OLED_DrawArc(...)` | 椭圆/弧线 |
| `void OLED_ShowImage(int16_t X, int16_t Y, uint8_t W, uint8_t H, const uint8_t *Image)` | 显示图片 |
| `void OLED_Animation(uint8_t X1,..., uint8_t W2)` | 平滑动画过渡 |
| `void OLED_AnimUpdate(void)` | 动画帧更新 |
| `void OLED_Reverse(void)` / `OLED_ReverseArea(...)` | 反色显示 |

---

#### 7.2.7 ZDT_Motor — 步进电机驱动

**文件**: `BSP/ZDT_Motor.c`, `BSP/ZDT_Motor.h`

**职责**: 两相步进电机的脉冲控制，用于云台的偏航和俯仰轴。

**API**:

| 函数 | 说明 |
|------|------|
| `ZDT_Motor_t* ZDT_Motor_Init(config)` | 初始化步进电机 (Step/Dir 引脚 + 定时器) |
| `void ZDT_Set_Position(ZDT_Motor_t *motor, float position)` | 设置目标位置 (角度) |
| `void ZDT_TICK_Init(void)` | 初始化步进脉冲定时器 (0.3ms) |

---

#### 7.2.8 K230 — AI 视觉模块

**文件**: `BSP/Comm/K230.c`, `BSP/Comm/K230.h`

**职责**: K230 AI 视觉模块的 UART 通信协议解析，接收钢球位置数据。

**数据包格式** (8 字节):

```
[0xA5] [x_low] [x_high] [dt_byte0] [dt_byte1] [dt_byte2] [dt_byte3] [crc8]
 帧头   位置X (uint16_LE)   时间间隔 dt (float32_LE)            CRC-8/MAXIM
```

**导出变量/类型**:

```c
typedef struct {
    uint16_t x_position;
    float    dt;
} steel_ball_movement_typedef;

extern volatile uint8_t k230_data_valid;
```

**API**:

| 函数 | 说明 |
|------|------|
| `void K230_Init(void)` | 清 UART FIFO + 使能 RX 中断 |
| `void K230_ReceiveData(const uint8_t RxData)` | 状态机解析 (2 态: 等待帧头→接收数据) |
| `uint8_t K230_Read(steel_ball_movement_typedef *data)` | 读取最新有效帧 (消费后 k230_data_valid 清零) |

**解析状态机**:
1. 等待 0xA5 (帧头) → 进入接收状态
2. 接收 7 字节数据 → CRC-8/MAXIM 校验
3. 校验通过 → 提取 x_position (uint16) + dt (float32) → `k230_data_valid = 1`
4. 帧头在接收中途出现时自动重新同步

---

#### 7.2.9 lichee_rec — LiChee RV Nano 识别模块

**文件**: `BSP/lichee_rec/lichee_rec.c`, `BSP/lichee_rec/lichee_rec.h`

**职责**: LiChee RV Nano 视觉识别模块的 UART 通信协议解析，接收滑槽长度和钢球相对位置数据。

**数据包格式** (11 字节):

```
[0xA5] [0x0A] [slider_len float32_LE] [relative_pos float32_LE] [crc8]
 帧头   cmd_id  滑槽长度 (cm)           相对位置 (cm)            CRC-8/MAXIM
```

**导出类型/变量**:

```c
typedef enum { OFFLINE = 0, ONLINE = 1 } LicheervnanoStatus_t;

typedef struct {
    float slider_length_cm;
    float relative_position;
} Licheervnano_Frame;

extern LicheervnanoStatus_t host_status;
```

**API**:

| 函数 | 说明 |
|------|------|
| `void LicheeRec_Init(void)` | 清 UART FIFO + 使能 RX 中断 |
| `void LicheeRec_ReceiveData(uint8_t RxData)` | 状态机解析 (3 态: 帧头→cmd_id→数据) |
| `uint8_t LicheeRec_GetFrame(Licheervnano_Frame *frame)` | 读取最近有效帧 (消费后清零) |

**依赖**: `crc8.h` (CRC-8/MAXIM 校验)

---

#### 7.2.10 CRC8 — CRC-8 校验算法

**文件**: `BSP/Algorithm/crc8.c`, `BSP/Algorithm/crc8.h`

**职责**: 提供两种 CRC-8 算法，供 K230 和 LiChee 通信协议使用。

**API**:

| 函数 | 说明 |
|------|------|
| `uint8_t crc_8(const uint8_t *data, uint16_t num_bytes)` | CRC-8/SHT75 (左移, 查表法) |
| `uint8_t update_crc_8(uint8_t crc, uint8_t val)` | CRC-8/SHT75 增量更新 |
| `uint8_t crc8_maxim(const uint8_t *data, size_t len)` | CRC-8/MAXIM (右移, 反射多项式 0x8C, 初值 0x00) |

---

#### 7.2.11 trace — 灰度巡线传感器

**文件**: `BSP/trace.c`, `BSP/trace.h`, `BSP/No_Mcu_Ganv_Grayscale_Sensor.c/.h`

**职责**: 基于灰度传感器阵列的巡线检测与 PID 控制。

**数据结构**:

```c
typedef struct {
    float pid_output;
} trace_fetch_data_q;
```

**API**:

| 函数 | 说明 |
|------|------|
| `void Trace_Init(void)` | 初始化灰度传感器 + 巡线 PID (Kp=0.0075, Kd=0, Ki=0, max_out=500) |
| `float Trace_task(void)` | 读取传感器 → 计算误差 → PID 输出 |
| `float Cal_Trace_Err(uint8_t current_trace)` | 加权平均计算线位置误差 |

**误差计算** (`Cal_Trace_Err`):
- 输入: 8 位传感器位掩码
- 权重: `{-4, -3, -2, -1, +1, +2, +3, +4}`
- 输出: 线位置偏差的加权平均值

---

#### 7.2.10 PID — 通用 PID 控制器

**文件**: `BSP/PID.c`, `BSP/PID.h`

**职责**: 通用 PID 控制器，支持位置式和增量式两种模式。

**枚举**:

```c
enum PID_MODE { PID_POSITION = 0, PID_DELTA };
```

**核心结构体**:

```c
typedef struct {
    uint8_t mode;         // PID_POSITION / PID_DELTA
    float Kp, Ki, Kd;     // PID 增益
    float max_out;        // 输出限幅
    float max_iout;       // 积分限幅
    float deadzone;       // 死区
    float Ref;            // 参考值
    float Measure;        // 测量值
    float out;            // 输出
    float Pout, Iout, Dout; // 各分量
    float Dbuf[3];        // 微分历史
    float error[3];       // 误差历史 [当前, 上次, 上上次]
} pid_type_def;

typedef struct {
    uint8_t mode;
    float Kp, Ki, Kd;
    float max_out, max_iout, deadzone;
} pid_init_config_s;
```

**API**:

| 函数 | 说明 |
|------|------|
| `void PID_init(pid_type_def *pid, pid_init_config_s *config)` | 初始化 PID |
| `float PID_calc(pid_type_def *pid, float ref, float Measure)` | 计算 PID 输出 |
| `void PID_clear(pid_type_def *pid)` | 清零所有状态 (积分/微分/误差历史) |

**位置式 PID**: `out = Kp*e + ∫Ki*e + Kd*de/dt`，带积分分离和输出限幅
**增量式 PID**: `Δout = Kp*Δe + Ki*e + Kd*Δ²e`，输出累加

---

#### 7.2.11 daemon — 守护进程

**文件**: `BSP/daemon.c`, `BSP/daemon.h`

**职责**: 软件看门狗/心跳超时监控框架。当被监控的设备在规定时间内未刷新时触发回调。

**核心结构体**:

```c
typedef void (*offline_callback)(void *);

typedef struct daemon_ins {
    uint16_t reload_count;        // 超时阈值
    offline_callback callback;    // 超时回调
    uint16_t temp_count;          // 当前倒计时
    void *owner_id;               // 回调参数
} DaemonInstance;
```

**API**:

| 函数 | 说明 |
|------|------|
| `DaemonInstance* DaemonRegister(Daemon_Init_Config_s *config)` | 注册守护实例 (动态分配) |
| `void DaemonReload(DaemonInstance *instance)` | 喂狗 (重置倒计时) |
| `uint8_t DaemonIsOnline(DaemonInstance *instance)` | 检查是否在线 (倒计时 > 0) |
| `void Daemon_Task(void)` | 守护任务: 所有实例倒计时递减，超时触发回调 |

---

#### 7.2.12 dwt — 高精度定时器

**文件**: `BSP/dwt.c`, `BSP/dwt.h`

**职责**: 基于 DWT (Data Watchpoint Timer) / GPTimer 的高精度周期计时。

**时间结构体**:

```c
typedef struct {
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;
```

**API**:

| 函数 | 说明 |
|------|------|
| `void DWT_Init(uint32_t CPU_Freq_mHz)` | 初始化定时器 (参数: CPU 频率 MHz) |
| `float DWT_GetDeltaT(uint32_t *cnt_last)` | 两次调用之间的时间差 (秒) |
| `double DWT_GetDeltaT64(uint32_t *cnt_last)` | 同上 (双精度) |
| `float DWT_GetTimeline_s(void)` | 自初始化以来的绝对时间 (秒) |
| `float DWT_GetTimeline_ms(void)` | 自初始化以来的绝对时间 (毫秒) |
| `uint64_t DWT_GetTimeline_us(void)` | 自初始化以来的绝对时间 (微秒) |
| `void DWT_Delay(float Delay)` | 高精度阻塞延时 (秒) |
| `void DWT_SysTimeUpdate(void)` | 系统时间更新 |

**宏**:

```c
#define TIME_ELAPSE(dt, code)  // 测量代码块执行时间
```

---

#### 7.2.13 menu — OLED 菜单系统

**文件**: `BSP/menu.c`, `BSP/menu.h`

**职责**: 基于 OLED 的三级层次菜单系统，支持按键导航。

**回调类型**:

```c
typedef void (*menu_callback)(uint8_t);
```

**核心结构体**:

```c
typedef struct MenuInstance {
    struct MenuInstance* next_menu[MAX_MENU_NUM];  // 子菜单 (最多6个)
    struct MenuInstance* pre_menu;                   // 父菜单
    menu_callback callback[MAX_MENU_NUM];            // 回调函数
    char* string[MAX_MENU_NUM];                      // 菜单项文本
    uint8_t CharNum[MAX_MENU_NUM];                   // 菜单项字符数
    uint8_t pre_idx;                                  // 返回索引
} MenuInstance;
```

**菜单层级**:

```
第一级:  [电机控制]  [控制方式]  [任务]
             │            │         │
第二级: 使能/失能   程序控制/按键控制  任务一/任务二
             │            │
第三级:     -      普通/巡线/陀螺仪/位置
```

**按键操作**:
- KEY0 (短按): 进入 / 确认
- KEY3 (短按): 返回
- KEY1 (短按): 下移
- KEY2 (短按): 上移

**API**:

| 函数 | 说明 |
|------|------|
| `void MenuInit(void)` | 构建三级菜单树 + 注册回调 |
| `void menu_task(void)` | 按键处理 + OLED 显示刷新 (由 KeyTask 调用) |

---

#### 7.2.14 bsp_log — 日志系统

**文件**: `Vendor/SEGGER/log/bsp_log.c`, `Vendor/SEGGER/log/bsp_log.h`

**职责**: 基于 SEGGER RTT (Real-Time Transfer) 的日志系统，通过 J-Link 输出调试信息到 PC 终端。

**日志宏**:

| 宏 | 级别 | 颜色 | 说明 |
|----|------|------|------|
| `LOG(format, ...)` | - | - | 普通输出，始终可用 |
| `LOGINFO(format, ...)` | INFO | 亮绿 | 信息输出，可通过 `DISABLE_LOG_SYSTEM` 禁用 |
| `LOGWARNING(format, ...)` | WARNING | 亮黄 | 警告输出 |
| `LOGERROR(format, ...)` | ERROR | 亮红 | 错误输出 |
| `LOG_CLEAR()` | - | - | 清除终端 |

**使用注意事项**:
- RTT 不直接支持 `%f` 格式，浮点值需通过 `Float2Str()` 手动转换
- 需要 J-Link 调试会话连接才能看到输出

**API**:

| 函数 | 说明 |
|------|------|
| `void BSPLogInit(void)` | 初始化 RTT |
| `int PrintLog(const char *fmt, ...)` | 格式化日志输出 |
| `void Float2Str(char *str, float va)` | 浮点数转字符串 |

---

#### 7.2.15 fifo — 环形缓冲区

**文件**: `BSP/fifo.c`, `BSP/fifo.h`

**职责**: 字节级环形 FIFO 队列。

**常量**: `FIFO_SIZE = 128`

**核心结构体**:

```c
typedef struct {
    uint8_t buffer[FIFO_SIZE];
    __IO uint8_t ptrWrite;
    __IO uint8_t ptrRead;
} FIFO_t;
```

**API**:

| 函数 | 说明 |
|------|------|
| `void fifo_initQueue(__IO FIFO_t *fifo)` | 初始化队列 |
| `void fifo_enQueue(__IO FIFO_t *fifo, uint8_t data)` | 入队 |
| `uint8_t fifo_deQueue(__IO FIFO_t *fifo)` | 出队 |
| `bool fifo_isEmpty(__IO FIFO_t *fifo)` | 判空 |
| `uint8_t fifo_queueLength(__IO FIFO_t *fifo)` | 获取队列长度 |

---

## 8. 关键 API 速查表

### 8.1 底盘控制

```c
// 初始化
void Chassis_Init(void);

// 控制循环 (由 ChassisTask 每 5ms 调用)
void Chassis(void);

// 电机管理
DCMotorInstance* DCMotor_Init(DCMotorInitConfig_s *config);
void DCMotor_Cmd(DCMotorInstance* motor, State state);               // 使能/失能
void DCMotor_SetPosition(DCMotorInstance *motor, float Position);    // 设置位置 (m)
void DCMotor_SetTraceCompensation(DCMotorInstance *motor, float c);  // 巡线修正
void Hw_Motor_Task(void);                                            // 控制循环 (10ms)
```

### 8.2 云台控制

```c
// 初始化
void Gimbal_Init(void);

// 控制循环 (由 GimbalTask 每 5ms 调用)
void Gimbal(void);

// 步进电机
ZDT_Motor_t* ZDT_Motor_Init(config);
void ZDT_Set_Position(ZDT_Motor_t *motor, float position);  // 设置角度
void ZDT_TICK_Init(void);
```

### 8.3 命令系统

```c
void RobotCmd_Init(void);                                  // 队列 + PID 初始化
void Robot_Cmd(void);                                      // 命令分发循环
void Task_Callback(uint8_t i);                             // 设置云台任务模式
void Chassis_Mode_Switch_Callback(uint8_t i);              // 切换底盘模式

// 队列句柄 (extern)
QueueHandle_t chassis_cmd_queue;                           // 底盘命令队列
QueueHandle_t gimbal_cmd_queue;                            // 云台命令队列
QueueHandle_t trace_fetch_data_queue;                      // 巡线数据队列
```

### 8.4 传感器

```c
// ICM42688 IMU
int8_t bsp_Icm42688Init(void);
int8_t bsp_Icm42688RegCfg(void);
int8_t bsp_IcmGetRawData(icm42688RealData_t* acc, icm42688RealData_t* gyro);

// AHRS 姿态解算
void IMU_init(void);
void IMU_getYawPitchRoll(float *ypr);                      // ypr[0]=Yaw, [1]=Pitch, [2]=Roll

// K230 AI 视觉
void K230_Init(void);
uint8_t K230_Read(steel_ball_movement_typedef *data);          // 读取钢球位置数据
extern volatile uint8_t k230_data_valid;

// LiChee RV Nano
void LicheeRec_Init(void);
uint8_t LicheeRec_GetFrame(Licheervnano_Frame *frame);         // 读取滑槽识别帧
extern LicheervnanoStatus_t host_status;

// 灰度巡线
void Trace_Init(void);
float Trace_task(void);                                    // 返回 PID 修正量

// PAW3395 光流
void PAW3395_Init(void);
void PAW3395_Read_Motion(float* axis);                     // axis[0]=dx(m), axis[1]=dy(m)
```

### 8.5 PID 控制

```c
void PID_init(pid_type_def *pid, pid_init_config_s *config);
float PID_calc(pid_type_def *pid, float ref, float Measure);
void PID_clear(pid_type_def *pid);
```

### 8.6 OLED 显示

```c
void OLED_Init(void);
void OLED_Update(void);
void OLED_Clear(void);
void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize);
void OLED_ShowFloatNum(int16_t X, int16_t Y, double Number, uint8_t IntLen, uint8_t FraLen, uint8_t FontSize);
void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...);
void OLED_DrawLine/Circle/Rectangle/...(...);
```

### 8.7 系统工具

```c
// 高精度定时
void DWT_Init(uint32_t CPU_Freq_mHz);
float DWT_GetTimeline_ms(void);
void DWT_Delay(float Delay);

// 守护进程
DaemonInstance* DaemonRegister(Daemon_Init_Config_s *config);
void DaemonReload(DaemonInstance *instance);
uint8_t DaemonIsOnline(DaemonInstance *instance);

// 日志
LOGINFO("format", ...);
LOGWARNING("format", ...);
LOGERROR("format", ...);

// FIFO
void fifo_enQueue(__IO FIFO_t *fifo, uint8_t data);
uint8_t fifo_deQueue(__IO FIFO_t *fifo);
bool fifo_isEmpty(__IO FIFO_t *fifo);

// 编码器
ENCODER_RES* Encoder_Init(encoder_PortPin_s* init);
void Encoder_InterruptBegin(void);
void Encoder_Update(void);

// 延时
void delay_us(unsigned long __us);
void delay_ms(unsigned long ms);
```

---

> 文档生成日期: 2026-08-01
> 项目分支: `master` (合并 clion_merge 的 ZDT 电机控制与 K230 通信代码)



