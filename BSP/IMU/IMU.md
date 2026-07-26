## 📊 IMU姿态解算完整流程 (Mahony 互补滤波)

### 1️⃣ 函数调用链与频率

```
Chassis任务运行 (每5ms调用一次 = 200Hz)
    ↓
IMU_getYawPitchRoll(IMU_data)
    ↓
bsp_IcmGetRawData(&acc, &gyro)  ← SPI 突发读取 ICM42688 (14 bytes)
    │                              返回 acc [m/s²], gyro [rad/s]
    ↓
MahonyAHRSupdate(gyro, acc, dt, dt)  ← Mahony 互补滤波器
    ├─ 归一化加速度
    ├─ 预测重力方向 (从四元数)
    ├─ 叉积误差 = acc_meas × acc_pred
    ├─ PI 控制器 (Kp=1.5, Ki=0.05)
    ├─ 四元数积分: q += 0.5 * q⊗ω * dt
    └─ 归一化四元数
    ↓
MahonyGetEuler(&roll, &pitch, &yaw)  ← 四元数 → 欧拉角 [度]
    ↓
返回 ypr[0]=Yaw, ypr[1]=Pitch, ypr[2]=Roll
```

**调用频率**：
- **Chassis 函数**：200Hz（5ms 周期）
- **IMU_getYawPitchRoll()**：**200Hz**（在 Chassis 中被调用）

---

### 2️⃣ ICM42688 驱动 (BSP/IMU/icm42688.c)

**修复的问题**：
| 问题 | 旧代码 | 新代码 |
|------|--------|--------|
| 符号错误 | `uint16_t` 拼接 → 负值变正值 | `int16_t` → 正确 |
| 加速度单位 | mg (milli-g) | **m/s²** (×9.8) |
| 陀螺仪单位 | dps (度/秒) | **rad/s** (×π/180) |
| Bank 切换 | 缺失 | `icm_select_bank()` |
| 零偏校准 | 运行时方差检测 (不可靠) | 启动时 500 采样平均 |

**输出单位**：
- 加速度计：**m/s²** （静止时 Z 轴≈ +9.8 或 -9.8，取决于安装方向）
- 陀螺仪：**rad/s**
- 量程：±2000dps, ±16g, 1kHz ODR

---

### 3️⃣ Mahony 互补滤波算法

**参数**：
| 参数 | 值 | 说明 |
|------|-----|------|
| Kp | 1.5 | 比例增益 — 加速度计修正强度 |
| Ki | 0.05 | 积分增益 — 陀螺仪零偏估计速度 |
| 积分限幅 | ±0.1 rad/s | 防止积分饱和 |

**算法特点**：
- 加速度计仅在静态时有效（测量重力方向）
- 动态时主要靠陀螺仪积分，加速度计修正减弱
- 积分项自动估计陀螺仪零偏漂移

---

### 4️⃣ 数据传输路径

```
硬件层：
┌─────────────────────────┐
│  ICM42688 传感器        │ ← 6轴 IMU (加速度 + 陀螺仪)
│  输出：ax,ay,az,gx,gy,gz│
└──────────────┬──────────┘
               │ (SPI 突发读取 14 bytes)
               ↓
┌─────────────────────────┐
│  bsp_IcmGetRawData()    │ ← BSP 驱动层
│  返回 acc [m/s²]        │
│  返回 gyro [rad/s]      │
│  自动减去零偏           │
└──────────────┬──────────┘
               │
               ↓
应用层：
┌─────────────────────────┐
│  Chassis 任务 (200Hz)   │
│  ↓                      │
│  IMU_getYawPitchRoll()  │
│  ↓                      │
│  MahonyAHRSupdate()     │ ← Mahony 互补滤波 (from Legacy)
│  ↓                      │
│  MahonyGetEuler()       │ ← 四元数 → 欧拉角 (ZYX)
│  ↓                      │
│  IMU_data[0]=Yaw        │
│  IMU_data[1]=Pitch      │ ← 输出单位: 度
│  IMU_data[2]=Roll       │
└─────────────────────────┘
```

---

### 5️⃣ 关键参数汇总

| 参数 | 值 | 单位 | 说明 |
|------|-----|------|------|
| **采样频率** | 200 | Hz | Chassis 任务周期 |
| **陀螺仪量程** | ±2000 | dps | GFS_2000DPS |
| **加速度计量程** | ±16 | g | AFS_16G |
| **传感器 ODR** | 1000 | Hz | 内部采样率 |
| **Kp** | 1.5 | - | 比例增益 |
| **Ki** | 0.05 | - | 积分增益 |
| **陀螺仪零偏** | 启动校准 | rad/s | 500 次采样平均 |

---

### 6️⃣ 调用示例

```c
// Chassis_Init() 中初始化
IMU_init();  // 初始化 ICM42688 + Mahony 滤波器

// Chassis() 中每 5ms 读取姿态
float IMU_data[3];
IMU_getYawPitchRoll(IMU_data);
// IMU_data[0] = Yaw   [度]
// IMU_data[1] = Pitch [度]
// IMU_data[2] = Roll  [度]
```
