# Flash 参数双槽库说明

本文档说明当前工程中 Flash 参数双槽机制的上电逻辑、运行中改参逻辑、默认版本规则和典型用法。

## 文件分工

- `w25q128jv_spi.h/.c`
  - W25Q128JV 标准 SPI 底层驱动。
  - 负责 JEDEC ID 读取、状态寄存器读取、读数据、页写、跨页写、4K/32K/64K 擦除、整片擦除、复位和芯片自测。

- `flash_param_store.h/.c`
  - 参数双槽存储服务。
  - 负责参数打包、CRC 校验、双槽选择、损坏回退、自动修复、保存和运行期应用。

- `APP/Chassis.c`
  - 保留底盘相关默认参数。
  - 提供 `Chassis_FillDefaultParams()` 和 `Chassis_ApplyParams()`。

- `BSP/Trace/trace.c`
  - 保留循迹相关默认参数。
  - 提供 `Trace_FillDefaultParams()` 和 `Trace_ApplyParams()`。

## Flash 槽分配

参数区使用 W25Q128JV 最后两个 4KB 扇区：

```c
#define FLASH_PARAM_SLOT0_ADDR  0xFFE000
#define FLASH_PARAM_SLOT1_ADDR  0xFFF000
```

每个槽保存一整包参数，包含记录头和参数载荷。正常业务不要擦写这两个扇区。

`W25Q128JV_RunSelfTest()` 会擦除 `0xFFF000`，该地址现在是参数 `slot1`，正常启动禁止调用。

## 参数包内容

当前保存的是 `FlashParam_Data_s`：

```c
typedef struct
{
    pid_init_config_s trace_pid;
    pid_init_config_s line_yaw_pid;
    pid_init_config_s turn_pid;

    float line_distance_m[4];
    float turn_angle_deg[4];

    float line_accel_m;
    float line_slowdown_m;
    float line_min_speed_mps;
    float line_done_err_m;
    uint16_t line_done_ticks;

    float turn_done_err_deg;
    uint16_t turn_done_ticks;
    float action_speed_mps;
    float turn_speed_mps;
} FlashParam_Data_s;
```

PID 保存的是 `pid_init_config_s` 初始化参数，不保存 `pid_type_def` 运行状态。因此 `Ref`、`Measure`、`out`、积分项、微分缓存等运行过程数据不会写入 Flash。

## 默认参数放在哪里

默认值保留在使用模块内：

- 底盘默认值在 `Chassis_FillDefaultParams()`
- 循迹默认值在 `Trace_FillDefaultParams()`

Flash 参数库不直接保存业务默认值。需要默认参数时，`FlashParam_BuildDefault()` 会调用各模块的填充函数组装完整默认参数包。

这样后续修改某个模块的默认参数，只需要在使用处修改，不需要进 Flash 库找业务参数。

## 默认版本号规则

代码默认参数版本号定义在 `flash_param_store.h`：

```c
#define FLASH_PARAM_DEFAULT_REVISION  1U
```

每次你在编辑器里修改默认参数后，都应该把这个版本号加 1。

上电判断规则：

```text
1. Flash 参数无效
   -> 使用代码默认参数
   -> 写入 Flash

2. Flash 参数有效，但 Flash default_revision 小于代码 FLASH_PARAM_DEFAULT_REVISION
   -> 说明编辑器里的默认参数更新
   -> 使用代码默认参数
   -> 覆盖写入 Flash

3. Flash 参数有效，且 default_revision 相等
   -> 说明代码默认参数没有更新
   -> 使用 Flash 中 sequence 最大的槽

4. 单槽损坏
   -> 使用有效槽
   -> 用有效参数自动修复损坏槽
```

简化理解：

```text
版本号不同：代码默认参数优先
版本号相同：Flash 中 sequence 最大的参数优先
Flash 损坏：回退代码默认参数
```

注意：这是整包版本机制。代码默认版本号升级后，会整体覆盖 Flash 里通过屏幕调过的参数。

## 上电初始化逻辑

启动入口在 `Robot_Init()`：

```c
DWT_Init(80);
FlashParam_Init();
MenuInit();
Chassis_Init();
```

`FlashParam_Init()` 会执行：

```text
读取 JEDEC ID
读取 slot0
读取 slot1
校验 magic/version/payload_size/crc32
按 default_revision 和 sequence 选择最新参数
必要时修复损坏槽
必要时用代码默认参数覆盖旧版本 Flash
```

`Chassis_Init()` 会调用：

```c
Chassis_ApplyParams(FlashParam_GetActive());
```

`Trace_Init()` 会调用：

```c
Trace_ApplyParams(FlashParam_GetActive());
```

这样上电后使用的就是双槽机制选出的当前最新参数。

## 运行中通过屏幕改参数

屏幕端推荐调用：

```c
FlashParam_SaveAndApply(&params);
```

它内部会：

```text
FlashParam_Save()
-> 写入非活动槽
-> sequence + 1
-> 写后回读校验
-> 成功后切换当前活动参数

Chassis_ApplyParams()
-> 刷新底盘 PID、直线距离、转角、速度、加减速距离等

Trace_ApplyParams()
-> 刷新循迹 PID
```

典型代码：

```c
FlashParam_Data_s params = *FlashParam_GetActive();

params.line_yaw_pid.Kp = new_line_yaw_kp;
params.line_accel_m = new_line_accel_m;
params.line_distance_m[0] = new_line0_m;

if (FlashParam_SaveAndApply(&params) == FLASH_PARAM_OK)
{
    LOGINFO("[param] screen save OK");
}
else
{
    LOGERROR("[param] screen save FAIL");
}
```

注意：`FlashParam_SaveAndApply()` 应该在 `Chassis_Init()` 和 `Trace_Init()` 之后调用，也就是屏幕任务运行阶段调用，不要在模块初始化前调用。

## 只保存，不立即应用

如果只想写入 Flash，下次上电再生效，可以调用：

```c
FlashParam_Save(&params);
```

这个函数只负责保存并更新参数服务内部的 active 参数，不会主动刷新 `Chassis.c` 里的本地副本。

如果保存后还想手动应用，可以写：

```c
if (FlashParam_Save(&params) == FLASH_PARAM_OK)
{
    Chassis_ApplyParams(FlashParam_GetActive());
    Trace_ApplyParams(FlashParam_GetActive());
}
```

## 编辑器改默认参数

例如你在 `Chassis_FillDefaultParams()` 中改了默认直线距离：

```c
params->line_distance_m[0] = 1.00f;
```

需要同步修改：

```c
#define FLASH_PARAM_DEFAULT_REVISION  2U
```

下一次上电时，Flash 库会发现：

```text
Flash default_revision < 代码 FLASH_PARAM_DEFAULT_REVISION
```

然后使用代码默认参数覆盖写入 Flash。

如果你只改了默认参数值，但忘记增加 `FLASH_PARAM_DEFAULT_REVISION`，并且 Flash 里已有同版本参数，则上电仍会优先使用 Flash 中保存的参数。

## 增加新参数

增加新参数通常需要改这些位置：

1. 在 `FlashParam_Data_s` 中增加字段。
2. 在对应使用模块的默认填充函数中赋默认值。
3. 在对应使用模块的 `ApplyParams` 或运行逻辑中使用该字段。
4. 把 `FLASH_PARAM_DEFAULT_REVISION` 加 1。

示例：

```c
typedef struct
{
    ...
    float new_speed_limit;
} FlashParam_Data_s;
```

然后在使用模块中设置默认值：

```c
params->new_speed_limit = 0.25f;
```

结构体变化后，旧 Flash 记录通常会因为 `payload_size` 不一致被判无效，然后自动用代码默认参数重新写入。

## 常用接口

```c
FlashParam_Init();
```

上电初始化参数服务。

```c
const FlashParam_Data_s *FlashParam_GetActive(void);
```

获取当前活动参数，只读使用。

```c
FlashParam_GetDefault(&params);
```

获取代码默认参数，适合屏幕端提供“恢复默认值”功能。

```c
FlashParam_Save(&params);
```

保存参数到 Flash，但不主动应用到底盘和循迹。

```c
FlashParam_SaveAndApply(&params);
```

保存参数到 Flash，并立即应用到底盘和循迹，屏幕端改参推荐使用。

```c
FlashParam_Format();
```

擦除两个参数槽，慎用。

```c
FlashParam_RunSelfTest();
```

双槽机制自测，会擦写两个参数槽，测试结束会恢复测试前读取到的参数值。仅调试时临时调用。

## 上电日志判断

正常启动可以关注这些 log：

```text
[param] flash JEDEC mf=0xEF, dev=0x4018
[param] slot0 valid, rev=1, seq=3
[param] slot1 valid, rev=1, seq=4
[param] load slot1, rev=1, seq=4
[param] chassis apply source=2, rev=1, seq=4
```

含义：

- `rev` 是默认参数版本号。
- `seq` 是 Flash 保存序号，越大越新。
- `source=1` 表示 slot0。
- `source=2` 表示 slot1。
- `source=0` 表示当前使用代码默认值。

如果看到：

```text
[param] code defaults newer, flash rev=1, code rev=2
[param] update flash with code defaults OK, rev=2, seq=...
```

表示编辑器中的默认参数版本更新，系统已使用代码默认值覆盖 Flash。

## 注意事项

- Flash 写入前必须擦除，参数库内部已经处理，不要直接对参数槽调用底层写函数。
- 屏幕端改参数时，建议先复制 `FlashParam_GetActive()`，只改需要的字段，再整体保存。
- 不要直接修改 `FlashParam_GetActive()` 返回的内部指针内容。
- 正常业务不要调用 `W25Q128JV_ChipErase()`。
- 正常启动不要调用 `W25Q128JV_RunSelfTest()`。
- 如果通过编辑器改默认值，必须增加 `FLASH_PARAM_DEFAULT_REVISION`，否则已有 Flash 参数仍会优先生效。
