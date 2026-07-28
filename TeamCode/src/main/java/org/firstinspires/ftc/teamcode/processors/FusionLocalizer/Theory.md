# FusionLocalizer 原理说明

## 概述

`FusionLocalizer` 是一个基于 **扩展卡尔曼滤波器 (EKF)** 的多传感器融合定位器，用于 FTC 机器人竞赛。它将 **Pinpoint 里程计**（高频、短时精确、长时漂移）与 **Limelight MegaTag1 视觉定位**（低帧率、绝对位置、偶发不可用）的优势互补，输出平滑、鲁棒的全局位姿估计。

核心思想：**用 Pinpoint 做帧间预测，用 Limelight 做绝对修正，用 Hub IMU 检测异常运动，用 Ambiguity 评估视觉置信度。**

---

## 1. 系统架构

```
┌──────────────────────────────────────────────────────────┐
│                     每帧 update() 流程                      │
├──────────────────────────────────────────────────────────┤
│                                                          │
│   Pinpoint.odometry ──→ 局部速度 (vx, vy, ω)  [in/s]     │
│        │                                                  │
│        ▼                                                  │
│   Hub IMU (BHI260IMU)                                     │
│     ├─ pitch/roll 角加速度 → 旋转到场坐标 →  adaptQ_x, adaptQ_y   │
│     └─ yaw 角速度 jerk    →  adaptQ_θ                     │
│        │                                                  │
│        ▼                                                  │
│   adaptQ(dt) ──→ SimpleMatrix Q (3x3, in²/s)              │
│        │                                                  │
│        ▼                                                  │
│   EKF.setQ(Q) + EKF.predict(vx, vy, ω, timestamp)         │
│        │           状态 x, y 单位: 英寸                    │
│        ▼                                                  │
│   Limelight MT1 ──→ {x_m, y_m} [米] ──→ ×39.37 ──→ {x, y} [英寸] │
│        │                                                  │
│        ▼                                                  │
│   adaptR() ──→ SimpleMatrix R (3x3, in²)                  │
│        │                                                  │
│        ▼                                                  │
│   EKF.setR(R) + EKF.update(x, y, θ, timestamp)            │
│                    (仅当视觉有效)                           │
└──────────────────────────────────────────────────────────┘
```

### 状态定义

```
状态向量:  x = [x, y, θ]ᵀ
  - x, y: 全局坐标系下的机器人位置 (英寸)
  - θ:    全局朝向 (弧度, [-π, π])

控制输入:  u = [vx, vy, ω]ᵀ  (Pinpoint 提供的机器人局部速度)
  - vx: 前进速度 (in/s)
  - vy: 横移速度 (in/s)
  - ω:  角速度 (rad/s)

观测:     z = [x_m, y_m, θ_m]ᵀ  (Limelight 测得的全局位姿)
  - 原始单位: 米 -> 在传入 EKF 前转换为英寸
```

---

## 2. EKF 预测步骤 (Predict)

### 2.1 非线性运动模型

将 Pinpoint 提供的局部速度转换到全局坐标系：

```
x' = x + Δt · (vx·cosθ - vy·sinθ)
y' = y + Δt · (vx·sinθ + vy·cosθ)
θ' = θ + Δt · ω
```

### 2.2 雅可比矩阵

计算状态传播函数对状态的偏导，用于协方差传播：

```
      ┌                                        ┐
      │ 1    0    Δt·(-vx·sinθ - vy·cosθ)      │
A =   │ 0    1    Δt·( vx·cosθ - vy·sinθ)      │
      │ 0    0    1                             │
      └                                        ┘
```

### 2.3 协方差传播

```
P' = A · P · Aᵀ + Q · Δt
```

其中 `Q · Δt` 体现了过程噪声随时间累积的特性：Δt 越大，预测的不确定度增长越多。

---

## 3. EKF 更新步骤 (Update)

### 3.1 新息 (Innovation)

```
y = z - H·x          (H = I, 观测直接对应状态)
```

角度新息需要归一化到 `[-π, π]`，避免 ±2π 跳变导致错误。

### 3.2 卡尔曼增益

```
S = H·P·Hᵀ + R
K = P·Hᵀ · S⁻¹
```

### 3.3 状态与协方差更新

```
x = x + K·y
P = (I - K·H)·P
```

---

## 4. 自适应 Q 调节 (Hub IMU 冲击检测)

### 4.1 动机

当机器人发生碰撞、急停、被其他机器人撞击时，**Pinpoint 里程计会瞬间产生不可靠的位移**（编码器打滑、IMU 震荡）。如果此时 EKF 仍然信任里程计预测，融合位姿会偏离真实位置。

**解决方案**：使用 Hub IMU (BHI260IMU) 检测冲击，**各方向独立**调节 Q 倍增因子，使滤波器在受冲击的方向上更快收敛。

### 4.2 检测方法

Hub IMU 无法直接提供线性加速度，因此使用 pitch/roll 角加速度作为机器人受到冲击的间接指标，再通过当前航向角将体坐标系扰动转换到场坐标系。

| 方向 | 传感器 | 物理量 | 物理含义 |
|---|---|---|---|
| **x** | Hub IMU pitch 角加速度 | `\|fieldX\|` (rad/s²) | 前向/后向碰撞（体坐标系 pitch → 旋转到场 x） |
| **y** | Hub IMU roll 角加速度 | `\|fieldY\|` (rad/s²) | 侧向碰撞（体坐标系 roll → 旋转到场 y） |
| **θ** | Hub IMU yaw 角速度 jerk | `\|Δω_z/Δt\|` (rad/s²) | 旋转方向的碰撞急停 |

数据来源：
- `getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate` / `.yRotationRate` — pitch/roll 角速度
- `getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate` — yaw 角速度

体坐标系 → 场坐标系旋转公式：
```
pitchAccel = (pitchRate - lastPitchRate) / dt
rollAccel  = (rollRate  - lastRollRate)  / dt

fieldX = pitchAccel · cos(heading) - rollAccel · sin(heading)
fieldY = pitchAccel · sin(heading) + rollAccel · cos(heading)

qBoostX = updateBoost(qBoostX, |fieldX|, ANGULAR_ACCEL_THRESHOLD)
qBoostY = updateBoost(qBoostY, |fieldY|, ANGULAR_ACCEL_THRESHOLD)
```

### 4.3 Q 调整策略

每个方向独立执行相同的 boost/decay 逻辑：

```
if magnitude > threshold:
    qBoost = min(Q_BOOST_MAX, qBoost × (1 + magnitude / threshold))
else:
    qBoost = max(1.0, qBoost × Q_DECAY)

Q_diag[i] = Q_base × qBoost[i]
```

- **冲击时**：qBoost 指数增长（最大 10x），冲击幅度越大提升越快
- **平稳后**：qBoost 指数衰减（每帧 ×0.85），回到基线状态

最终 `adaptQ(dt)` 返回一个 3x3 对角 `SimpleMatrix`，直接传入 `EKF.setQ(SimpleMatrix)`。

### 4.4 参数调优指南

| 参数 | 推荐值 | 含义 |
|---|---|---|
| `ANGULAR_ACCEL_THRESHOLD` | 5.0 rad/s² | pitch/roll 角加速度触发阈值 |
| `JERK_THRESHOLD` | 4.0 rad/s² | yaw 角速度 jerk 触发阈值 |
| `Q_BOOST_MAX` | 10.0 | 最大 Q 倍增因子 |
| `Q_DECAY` | 0.85 | 每帧衰减系数 |

---

## 5. 自适应 R 调节 (Limelight stdDev)

### 5.1 动机

Limelight MegaTag1 的定位精度在不同条件下差异很大：
- **多标签、近距离、光照好** → 精度高（stddev ~0.01m）
- **单标签、远距离、倾斜角大** → 精度低（stddev ~0.1m+）
- **遮挡、运动模糊** → 完全不可用

**解决方案**：利用 Limelight 固件输出的 `stddevMt1` 各方向分量，**独立调节 R_x, R_y, R_θ**。每个方向的不确定度只影响该方向的观测噪声。

### 5.2 stdDev 来源与单位转换

```
double[] stdDevs = mt1.getStdDevs();  // {x, y, z, roll, pitch, yaw} (米, 度)
```

来自 `LLResult.getStddevMt1()` — Limelight 固件内部 MegaTag1 算法直接输出的标准偏差。

**位置 vs 角度使用不同阈值**:
- **x, y**: stdDev 从米 → 英寸 (×39.37)，与位置阈值比较
- **θ (yaw)**: stdDev 从度 → 弧度，与角度专用阈值比较

### 5.3 R 调整策略 (每方向独立)

**位置方向 (x, y)**:
```
std_inch = std_meter × 39.37

if std_inch < STD_LOW_INCH (2.0 in):     R = 0.01          # 完全信任
elif std_inch > STD_HIGH_INCH (6.0 in):  R = 0.01 × 20     # 高度怀疑
else:  t = (std_inch - 2.0) / (6.0 - 2.0)                  # 线性插值
       R = 0.01 × (1 + t × 19)
```

**角度方向 (θ)**:
```
std_rad = Math.toRadians(std_yaw_deg)    # 度 → 弧度

if std_rad < STD_LOW_ANGLE (0.035 rad):     R = 0.01        # 完全信任 (≈2°)
elif std_rad > STD_HIGH_ANGLE (0.175 rad):  R = 0.01 × 20  # 高度怀疑 (≈10°)
else:  t = (std_rad - 0.035) / (0.175 - 0.035)             # 线性插值
       R = 0.01 × (1 + t × 19)
```

最终 `adaptR()` 返回一个 3x3 对角 `SimpleMatrix`，直接传入 `EKF.setR(SimpleMatrix)`。

### 5.4 参数调优指南

| 参数 | 推荐值 | 含义 |
|---|---|---|
| `STD_LOW_INCH` | 2.0 in (≈0.05m) | 位置：低于此值完全信任视觉 |
| `STD_HIGH_INCH` | 6.0 in (≈0.15m) | 位置：高于此值高度怀疑视觉 |
| `STD_LOW_ANGLE` | 0.035 rad (≈2°) | 角度：低于此值完全信任视觉 |
| `STD_HIGH_ANGLE` | 0.175 rad (≈10°) | 角度：高于此值高度怀疑视觉 |
| `R_MAX_SCALE` | 20.0 | 最大 R 放大倍数 |

---

## 6. 自适应策略的协同效果

### 典型场景 1：正常行驶

```
Pinpoint 精度高, Limelight 偶尔更新
→ Hub IMU 加速度/角速度平稳 → qBoost_x/y/θ ≈ 1.0
→ Q = Q_base (正常), R 根据 stdDev 缩放
→ EKF 主要依赖 Pinpoint 预测, 视觉仅做缓慢修正
```

### 典型场景 2：机器人碰撞

```
Hub IMU 检测到 ax/ay >> 1 m/s² 或 jerk >> 4 rad/s²
→ 对应方向的 qBoost 瞬间提升到 3~10x
→ 该方向 Q 变大, 滤波器不再信任 Pinpoint 预测
→ 下次视觉更新时, 该方向卡尔曼增益 K 很大
→ 位姿迅速收敛到视觉观测值
→ 各方向独立: 仅受冲击的方向提升 Q, 其他方向保持正常
```

### 典型场景 3：Limelight 部分遮挡（单标签，远距离）

```
MT1 仍然有效, 但各方向 stdDev 不均匀
→ 例: stdX=0.08m(3.1in), stdY=0.12m(4.7in), stdYaw=8°
→ R_x ≈ 0.05, R_y ≈ 0.12, R_θ ≈ 0.17
→ 视觉对 x 方向修正较强, 对 y 和 θ 方向修正较弱
→ 避免单标签在各方向均匀拉偏
```

### 典型场景 4：视觉完全丢失

```
MT1 长时间无效 (无标签或遮挡)
→ 只有 predict 步骤, 没有 update 步骤
→ EKF 纯靠 Pinpoint 里程计, 协方差 P 随时间增长
→ 一旦视觉恢复 (stdDev 低), K 自动增大, 快速修正
```

---

## 7. 单位一致性

EKF 内部状态统一使用 **英寸 + 弧度**，所有传感器数据在输入前完成单位转换：

| 输入 | 原始单位 | 转换 | 输入 EKF 时单位 |
|---|---|---|---|
| Pinpoint 速度 (vx, vy) | in/s | 无需转换 | in/s |
| Pinpoint 角速度 (ω) | rad/s | 无需转换 | rad/s |
| Limelight 位置 (x, y) | 米 (m) | × 39.37 | 英寸 (in) |
| Limelight 朝向 (θ) | 度 (°) | × π/180 | 弧度 (rad) |
| MT1 stdDev 位置 (x, y) | 米 (m) | × 39.37 | 英寸 (in) |
| MT1 stdDev 角度 (yaw) | 度 (°) | × π/180 | 弧度 (rad) |
| **EKF 输出位姿** | — | — | **英寸 + 弧度** |

> **角度阈值独立**：`mapStdToR` 使用 `STD_LOW_INCH`/`STD_HIGH_INCH` 比较位置 stdDev；`mapStdToRAngle` 使用 `STD_LOW_ANGLE`/`STD_HIGH_ANGLE` 比较角度 stdDev，避免弧度值与英寸值误比较。

转换常数定义：
```java
private static final double M_TO_INCH = 39.37007874;
```

---

## 8. API 数据流

```
adaptQ(double dt)
  │
  ├─ hubImu.getRobotAngularVelocity() → pitchRate, rollRate
  │   → pitchAccel, rollAccel → 旋转到场坐标 → fieldX, fieldY → qBoostX, qBoostY
  └─ hubImu.getRobotAngularVelocity() →  yawRate → jerk → qBoostTheta
  │
  └─→ 构造 SimpleMatrix(3x3) 对角 Q (in²/s)  →  return Q

adaptR()
  │
  ├─ mt1.getStdDevs()  →  stdX_m, stdY_m, stdYaw_deg
  ├─ stdX_m × 39.37 → stdX_inch → mapStdToR  → rX       (位置阈值)
  ├─ stdY_m × 39.37 → stdY_inch → mapStdToR  → rY       (位置阈值)
  └─ Math.toRadians(stdYaw_deg) → mapStdToRAngle → rTheta (角度阈值)
  │
  └─→ 构造 SimpleMatrix(3x3) 对角 R (in²)  →  return R

EKF:
  setQ(SimpleMatrix Q)   — 直接接收完整 Q 矩阵
  setR(SimpleMatrix R)   — 直接接收完整 R 矩阵
  setQ(double, double, double)  — 标量接口 (保留)
  setR(double, double, double)  — 标量接口 (保留)
```

---

## 9. 时间戳管理与数据一致性

### 9.1 两种时间基准

| 来源 | 时间基准 | 用途 |
|---|---|---|
| `System.nanoTime()` | Java 单调时钟 | predict 的 dt 计算 |
| `LLResult.getTimestamp()` | Limelight 硬件时间戳 | update 的观测时间 |

### 9.2 防过时机制

EKF 内部维护 `lastUpdateTime`，拒绝 timestamp 小于等于上次更新的观测，避免网络延迟或帧率错乱导致的数据回退。

### 9.3 首次 Predict 处理

首次调用 `predict` 时只记录时间戳不做状态传播，避免 dt 为 0 导致除以零。

---

## 10. 参考

- Kou & Haggenmiller (2023), "Extended Kalman Filter State Estimation for Autonomous Competition Robots"
- Limelight MegaTag1 文档: https://docs.limelightvision.io/docs/docs-megatag
- GoBilda Pinpoint 文档: https://docs.gobilda.com/pinpoint-odometer
- BHI260IMU 文档: https://www.bosch-sensortec.com/products/smart-sensors/bhi260/
- EJML (Efficient Java Matrix Library): https://ejml.org