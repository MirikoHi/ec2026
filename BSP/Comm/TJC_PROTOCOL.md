# TJC 串口屏通信协议 v1.0

TJC 串口屏通过 UART_2 (PA1=RX, PA0=TX, 115200-8-N-1) 与 MCU 通信，使用 ASCII 文本协议，每条命令以 `\n` (0x0A) 结尾。

**TJC HMI 侧注意**：`prints` 不支持 `\r\n` 转义，必须用 `printh 0D 0A` 发送换行。MCU 端会自动过滤 `0x00` (prints 终止符) 和 `\r`。

---

## 一、命令总览

| 命令 | 格式 | 方向 | 说明 |
|------|------|------|------|
| `S` | `S,<page>,<comp>,<value>\n` | 屏幕→MCU | 设置参数值（numpad OK 后） |
| `W` | `W\n` | 屏幕→MCU | 保存当前参数到 Flash |
| `R` | `R\n` | 屏幕→MCU | 恢复编译期默认参数 |

---

## 二、S 命令 — 设置参数

### 格式

```
S,<page>,<comp>,<value>\n
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `S` | char | 命令标识 |
| `page` | 整数 (1~5) | 参数所在的 HMI 页面号 |
| `comp` | 整数 (0~7) | 参数对应的数字键盘控件 ID (n*) |
| `value` | ASCII 浮点数 | 参数值，如 `0.000880`, `1.2`, `-0.003` |

### 页面映射表

#### page 1 — 循迹 PID (`trace_pid`)

| comp | 控件 | 参数 | 类型 | 默认值 |
|------|------|------|------|--------|
| 0 | n0 | Kp | float | 0.006 |
| 1 | n1 | Ki | float | 0.0 |
| 2 | n2 | Kd | float | 0.0 |
| 3 | n3 | max_out | float | 500.0 |
| 4 | n4 | max_iout | float | 200.0 |
| 5 | n5 | deadzone | float | 0.0 |

#### page 2 — 设定长度 (`line_distance_m`)

| comp | 控件 | 参数 | 类型 | 默认值 |
|------|------|------|------|--------|
| 1 | n1 | 边1 = line_distance_m[0] | float | 0.96 (米) |
| 3 | n3 | 边2 = line_distance_m[1] | float | 0.96 (米) |
| 5 | n5 | 边3 = line_distance_m[2] | float | 0.96 (米) |
| 7 | n7 | 边4 = line_distance_m[3] | float | 0.96 (米) |

#### page 3 — 转向/直线 PID 导航页

该页面仅导航，不发送数据。两个按钮分别跳转 page 4 和 page 5。

#### page 4 — 直线 PID (`line_yaw_pid`)

| comp | 控件 | 参数 | 类型 | 默认值 |
|------|------|------|------|--------|
| 0 | n0 | Kp | float | 0.0008 |
| 1 | n1 | Ki | float | 0.0 |
| 2 | n2 | Kd | float | 0.000003 |
| 3 | n3 | max_out | float | 0.08 |
| 4 | n4 | max_iout | float | 0.0 |
| 5 | n5 | deadzone | float | 0.5 |

#### page 5 — 转向 PID (`turn_pid`)

| comp | 控件 | 参数 | 类型 | 默认值 |
|------|------|------|------|--------|
| 0 | n0 | Kp | float | 0.0008 |
| 1 | n1 | Ki | float | 0.000012 |
| 2 | n2 | Kd | float | 0.000003 |
| 3 | n3 | max_out | float | 0.08 |
| 4 | n4 | max_iout | float | 0.01 |
| 5 | n5 | deadzone | float | 0.001 |

---

## 三、W 命令 — 保存至 Flash

将当前 RAM 中的所有参数写入 W25Q128JV 外部 Flash（双槽保存），下电不丢失。

---

## 四、R 命令 — 恢复默认

加载编译期默认参数，覆盖 RAM 中的当前值，并写入 Flash。

---

## 五、TJC HMI 端发送代码

### 5.1 numpad 页面 — OK 按钮逻辑

在数字键盘页面的 `if()` 代码块中填入：

```
prints "S,",0
covx tempstr.txt,loadpageid.val,0,0
prints tempstr.txt,0
prints ",",0
covx tempstr.txt,loadcmpid.val,0,0
prints tempstr.txt,0
prints ",",0
prints input.txt,0
printh 0D 0A
```

**说明**：
- `loadpageid.val` 和 `loadcmpid.val` 必须在打开数字键盘前赋值
- 最后一行 `printh 0D 0A` 发送 `\r\n` 作为帧尾，**不能用 `prints "\r\n"`**
- 前面的 `prints` 会自动追加 `0x00`，MCU 端会过滤掉

### 5.2 存至 Flash 按钮 (Page 0)

Touch Release 事件：

```
prints "W",0
printh 0D 0A
```

### 5.3 恢复默认按钮

Touch Release 事件：

```
prints "R",0
printh 0D 0A
```

---

## 六、串口测试参考

用串口助手连接 UART_2 调试时，手动发送以下帧(勾选"发送新行"或手动加 `\n`)：

| 测试目的 | 发送内容 (ASCII) | 预期效果 |
|---------|------------------|---------|
| 修改循迹Kp | `S,1,0,0.010` + 换行 | trace_pid.Kp = 0.01，PID 立即生效 |
| 修改边1长度 | `S,2,1,1.500` + 换行 | line_distance_m[0] = 1.5 |
| 修改转向Kd | `S,5,2,0.000005` + 换行 | turn_pid.Kd = 0.000005 |
| 保存至Flash | `W` + 换行 | LOGINFO "[param] save slot... OK" |
| 恢复默认 | `R` + 换行 | 所有参数恢复为编译期默认值 |

---

## 七、错误处理

| 情况 | MCU 行为 |
|------|---------|
| page 不在 1/2/4/5 范围 | 忽略该帧 |
| comp 不在对应页面的映射表中 | 忽略该帧 |
| value 解析失败（乱码） | 忽略该帧 |
| Flash 写入失败 | LOGERROR，参数只在 RAM 中生效 |
| 帧格式错误（缺少逗号等） | 忽略该帧 |
