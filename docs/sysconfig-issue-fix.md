# TI SysConfig 无法打开 .syscfg 文件 — 解决方案

## 问题现象

双击 `robot.syscfg` 无法用 TI SysConfig 打开，或者使用 SysConfig GUI 的 File → Open 打开后提示找不到 SDK。

## 根因分析

### .syscfg 文件的 `@cliArgs` 使用命名引用而非路径引用

`robot.syscfg` 文件头部通过 `@cliArgs` 注释声明了所需的 SDK：

```
@cliArgs --product "mspm0_sdk@2.10.00.04"
```

这里用的是**按名称+版本号查找**的方式（`mspm0_sdk@2.10.00.04`），而不是直接的文件路径。

### 独立版 SysConfig 没有 SDK 注册表

SysConfig 分为两种形态：

| 形态 | SDK 发现机制 |
|------|-------------|
| CCS 内置的 SysConfig | CCS 安装 SDK 时自动注册到 SysConfig，按名称查找可用 |
| 独立安装的 SysConfig | 没有全局 SDK 注册表，不知道本地装了哪些 SDK |

当通过 GUI 的 **File → Open** 打开 `.syscfg` 文件时，SysConfig 读取 `@cliArgs` 中的 `--product "mspm0_sdk@2.10.00.04"`，然后尝试按名称搜索 SDK。独立版 SysConfig 没有 SDK 注册表，**即使 SDK 就放在硬盘上、版本完全匹配**，也找不到，直接报错：

```
No product with name "mspm0_sdk" and version "2.10.00.04" found
```

**这不是路径问题，也不是版本不匹配问题，而是命名解析机制在独立版 SysConfig 中根本不工作。**

### 关键结论

就算把 SDK 放在 `D:\Ti\mspm0_sdk\` 或任何位置，只要 `@cliArgs` 用的是命名引用（`mspm0_sdk@2.10.00.04`）而不是完整路径，独立版 SysConfig 的 GUI 就无法打开。只有通过 CLI 显式传入 `product.json` 的完整路径才能绕过这个限制。

## 解决方案

### 方案 A：CLI 直接指定 SDK 路径（推荐）

用 SysConfig 命令行/GUI 启动时，显式传入 `product.json` 的绝对路径：

```bat
D:\Ti\sysconfig_1.28.0\sysconfig_gui.bat ^
  --product "D:\Ti\mspm0_sdk\.metadata\product.json" ^
  --device "MSPM0G3507" ^
  --package "LQFP-64(PM)" ^
  robot.syscfg
```

- `--product`：SDK 的 `product.json` 完整路径
- `--device`：目标 MCU 型号
- `--package`：封装类型

### 方案 B：编写一键启动脚本（`syscfg.bat`）

在项目根目录放置 `syscfg.bat`，一键启动 GUI：

```bat
@echo off
set SYSCFG_GUI=D:\Ti\sysconfig_1.28.0\sysconfig_gui.bat
set SDK_PRODUCT=D:\Ti\mspm0_sdk\.metadata\product.json

start "" "%SYSCFG_GUI%" ^
  --product "%SDK_PRODUCT%" ^
  --device "MSPM0G3507" ^
  --package "LQFP-64(PM)" ^
  "%~dp0robot.syscfg"
```

`%~dp0` 表示 bat 文件所在目录，确保无论从哪个目录执行都能找到 `robot.syscfg`。

### 方案 C：确保 SDK 在 SysConfig 可自动发现的路径

如果希望双击 `.syscfg` 文件直接打开，需要满足两个条件：

1. **SDK 放对位置**：将 SDK 放在 SysConfig 能扫描到的目录，例如：
   - `%LOCALAPPDATA%\Texas Instruments\sysconfig\products\<sdk_name>\`
   - 或 SysConfig 安装目录的同级目录（如 `D:\Ti\mspm0_sdk\`）

2. **关联文件类型**（如未自动关联）：
   - 右键 `.syscfg` 文件 → 打开方式 → 选择其他应用
   - 浏览到 `D:\Ti\sysconfig_1.28.0\sysconfig_gui.bat`
   - 勾选「始终使用此应用打开 .syscfg 文件」

但独立版 SysConfig 的 SDK 发现机制有限，**方案 A/B 最稳定可靠**。

## 诊断命令

排查时使用的有用命令：

```bat
:: 查看 SDK 版本和最低工具版本要求
type D:\Ti\mspm0_sdk\.metadata\product.json

:: 查看 SysConfig 版本
D:\Ti\sysconfig_1.28.0\sysconfig_cli.bat --version

:: CLI 试运行（验证 SDK 是否可用）
D:\Ti\sysconfig_1.28.0\sysconfig_cli.bat ^
  --product "D:\Ti\mspm0_sdk\.metadata\product.json" ^
  --device "MSPM0G3507" ^
  --package "LQFP-64(PM)" ^
  -o output_dir ^
  robot.syscfg
```

## 关键文件路径

| 文件 | 路径 |
|------|------|
| SysConfig | `D:\Ti\sysconfig_1.28.0\sysconfig_gui.bat` |
| MSPM0 SDK (v2.10.00.04) | `D:\Ti\mspm0_sdk\` |
| SDK 元数据 | `D:\Ti\mspm0_sdk\.metadata\product.json` |
| 项目 syscfg 文件 | `D:\Diansai\electric-competition-project\robot.syscfg` |
| 一键启动脚本 | `D:\Diansai\electric-competition-project\syscfg.bat` |

## 总结

核心思路：**绕过 SysConfig 的 SDK 名称搜索，直接用 CLI 参数传 `product.json` 完整路径**。这样无需关心 SDK 注册位置、版本字符串匹配等问题，适用于任何 SDK 版本和安装路径。
