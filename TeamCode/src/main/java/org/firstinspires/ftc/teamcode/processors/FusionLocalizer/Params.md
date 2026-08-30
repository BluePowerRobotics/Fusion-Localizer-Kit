# 融合定位器可调参数速查表

> 对应测试 OpMode：[`FusionTestOpMode`](../../opmodes/FusionTestOpMode.java)（D2）与 [`FusionD3TestOpmode`](../../opmodes/FusionD3TestOpmode.java)（D3/5D）。
>
> 所有 `public static` 字段可在 FTC Dashboard 的 "Config" 页实时修改。注意：**固定 Q/R 版本（EKFLocalizer / UKFLocalizer）的参数在构造时一次性读取**，运行中修改需重启 OpMode 生效；**自适应版本每帧实时读取**，修改立即生效。

## 1. OpMode 级参数（两个 OpMode 均有）

| 参数 | 默认值 | 单位 | 含义 |
|---|---|---|---|
| `testX` | 63.0 | 英寸 | 误差测量目标点 X（到达后按 A 记录误差） |
| `testY` | 60.7 | 英寸 | 误差测量目标点 Y |
| `testHeading` | π/2 | 弧度 | 误差测量目标航向 |

## 2. FusionTestOpMode（D2 模式）

### 2.1 PinpointLocalizer（里程计基准）

> 构造时使用 `IN_PER_TICK = 0.001999`（每 tick 英寸数）。

| 参数 | 默认值 | 单位 | 含义 |
|---|---|---|---|
| `PARAMS.parYTicks` | 2460 | tick | 平行（X 方向）编码器相对旋转中心的位置（Y 偏移） |
| `PARAMS.perpXTicks` | -1970 | tick | 垂直（Y 方向）编码器相对旋转中心的位置（X 偏移） |

### 2.2 MT1Localizer（视觉基准）

无可调参数，仅提供实时质量指标（telemetry 只读）：`valid / tagCount / avgDist / avgArea / span / maxFiducialSkew / ambiguity / angularAmb / captureLatency / timestamp / stdDevs[6]`。

### 2.3 EKFLocalizer（固定 Q/R）

> 构造时读取，运行中修改无效。

| 参数 | 默认值 | 单位 | 含义 |
|---|---|---|---|
| `QbasePos` | 0.01 | in²/s | 过程噪声（位置），越大越信任视觉、越"灵活"跟随 |
| `QbaseAngle` | 0.01 | rad²/s | 过程噪声（航向） |
| `RbasePos` | 0.01 | in² | 观测噪声（位置），越大越信任里程计预测、抑制视觉抖动 |
| `RbaseAngle` | 0.05 | rad² | 观测噪声（航向） |

### 2.4 AdaptiveEKFLocalizer（D2 自适应）

> 每帧实时读取。自适应策略：D2 用 **roll/pitch 角加速度** 检测冲击。

| 参数 | 默认值 | 单位 | 含义 |
|---|---|---|---|
| `qBase` | 0.002 | in²/s | Q 基值（实际 Q = qBase × 各方向 boost） |
| `qBoostX / qBoostY / qBoostTheta` | 1.0 | - | 各方向 Q 倍增因子（**运行时状态**，仅 telemetry 只读） |
| `ANGULAR_ACCEL_THRESHOLD` | 5.0 | rad/s² | D2：roll/pitch 角加速度阈值，超过即判定冲击 |
| `JERK_THRESHOLD` | 4.0 | rad/s² | yaw 角加速度阈值（旋转碰撞检测） |
| `ANGULAR_VEL_THRESHOLD` | 1.0 | rad/s | D3 专用（D2 中不参与） |
| `VEL_BOOST_MAX` | 10.0 | - | D3 专用（D2 中不参与） |
| `ACCEL_BOOST_MAX` | 4.0 | - | 已声明但**当前代码未接线**，D2 实际使用 `Q_BOOST_MAX` |
| `Q_BOOST_MAX` | 10.0 | - | Q 倍增因子上限（冲击检测用） |
| `Q_DECAY` | 0.85 | - | 无冲击时 Q 倍增因子的衰减系数（每帧 ×0.85，下限 1.0） |
| `STD_LOW_INCH` | 2.0 | 英寸 | 视觉位置 stdDev 低阈值（低于此 → 完全信任，R=0.01） |
| `STD_HIGH_INCH` | 6.0 | 英寸 | 视觉位置 stdDev 高阈值（高于此 → 完全怀疑，R=0.01×R_MAX_SCALE） |
| `STD_LOW_ANGLE` | 0.035 | rad (≈2°) | 视觉航向 stdDev 低阈值 |
| `STD_HIGH_ANGLE` | 0.175 | rad (≈10°) | 视觉航向 stdDev 高阈值 |
| `R_MAX_SCALE` | 20.0 | - | R 放大上限（std 高于阈值时 R = 0.01 × R_MAX_SCALE） |
| `rBase` | 0.01 | - | R 基值缓存（运行时状态，仅调试展示） |
| `M_TO_INCH` | 39.37 | - | 米→英寸单位换算（一般不改） |

**R 自适应映射**：`std ≤ LOW → R=0.01`；`std ≥ HIGH → R=0.01×R_MAX_SCALE`；中间线性插值。

### 2.5 UKFLocalizer（固定 Q/R）

与 2.3 EKFLocalizer 完全相同：`QbasePos / QbaseAngle / RbasePos / RbaseAngle`（同为构造时读取）。

### 2.6 AdaptiveUKFLocalizer（D2 自适应）

参数与 2.4 AdaptiveEKFLocalizer 完全相同（滤波器换成 UKF）。

## 3. FusionD3TestOpmode（D3 / 5D 模式）

### 3.1 PinpointD3Localizer（3D 斜坡补偿里程计基准）

| 参数 | 默认值 | 单位 | 含义 |
|---|---|---|---|
| `PARAMS.parYTicks` | 2460 | tick | 平行编码器 Y 偏移（同 2.1） |
| `PARAMS.perpXTicks` | -1970 | tick | 垂直编码器 X 偏移（同 2.1） |

> 编码器方向在构造时硬编码：平行 FORWARD、垂直 REVERSED。

### 3.2 MT1Localizer

同 2.2，无可调参数。

### 3.3 AdaptiveEKFLocalizer（D3 自适应）

> 每帧实时读取。自适应策略：D3 用 **roll/pitch 角速度** 检测坡度变化。

| 参数 | 默认值 | 单位 | 含义 |
|---|---|---|---|
| `qBase` | 0.002 | in²/s | Q 基值 |
| `qBoostX / qBoostY / qBoostTheta` | 1.0 | - | 各方向 Q 倍增因子（运行时状态，只读） |
| `ANGULAR_VEL_THRESHOLD` | 1.0 | rad/s | **D3 核心**：pitch/roll 角速度阈值，超过判定坡度变化 |
| `VEL_BOOST_MAX` | 10.0 | - | **D3 核心**：角速度触发的 Q 倍增上限（较高，因为坡度误差大） |
| `JERK_THRESHOLD` | 4.0 | rad/s² | yaw 角加速度阈值（旋转碰撞检测） |
| `Q_BOOST_MAX` | 10.0 | - | yaw 方向（及 D2 分支）Q 倍增上限 |
| `Q_DECAY` | 0.85 | - | Q 倍增因子衰减系数 |
| `ANGULAR_ACCEL_THRESHOLD` | 5.0 | rad/s² | D2 专用（D3 中不参与） |
| `ACCEL_BOOST_MAX` | 4.0 | - | 未接线（同 2.4） |
| `STD_LOW_INCH / STD_HIGH_INCH` | 2.0 / 6.0 | 英寸 | 视觉位置 stdDev 映射阈值 |
| `STD_LOW_ANGLE / STD_HIGH_ANGLE` | 0.035 / 0.175 | rad | 视觉航向 stdDev 映射阈值 |
| `R_MAX_SCALE` | 20.0 | - | R 放大上限 |
| `rBase` | 0.01 | - | R 基值缓存（只读） |

### 3.4 AdaptiveUKFLocalizer（D3 自适应）

参数与 3.3 完全相同（滤波器换成 UKF）。

### 3.5 AdaptiveEKF5DLocalizer（5D 加速度计驱动）

> 每帧实时读取（**加速度计朝向除外，构造时读取**）。自适应对象改为 **R_pinpoint**（Pinpoint 速度观测噪声），替代旧 Q 自适应。加速度计为外接 BNO055（硬件名 "accel"，封装 [Rev9axisIMU](../../Accelerometer/Rev9axisIMU.java)）。

| 参数 | 默认值 | 单位 | 含义 |
|---|---|---|---|
| `R_PIN_BASE` | 0.1 | in²/s² | Pinpoint 速度观测噪声基值（实际 = R_PIN_BASE × 对应 boost） |
| `R_PIN_THETA_BASE` | 0.05 | rad² | Pinpoint 航向观测噪声基值 |
| `AZ_THRESHOLD` | 2.0 | in/s² | 竖直加速度 |az| 触发阈值（颠簸/弹跳检测） |
| `ANGULAR_VEL_THRESHOLD` | 1.0 | rad/s | pitch/roll 角速度触发阈值 |
| `JERK_THRESHOLD` | 4.0 | rad/s² | yaw 角加速度阈值（旋转冲击） |
| `R_BOOST_MAX` | 20.0 | - | R 倍增因子上限 |
| `R_DECAY` | 0.85 | - | R 倍增因子衰减系数（无事件时每帧 ×0.85） |
| `ZERO_VEL_ACCEL_THRESHOLD` | 0.5 | in/s² | 零速检测：水平加速度小于此值… |
| `ZERO_VEL_PINPOINT_THRESHOLD` | 0.5 | in/s | …且 Pinpoint 速度小于此值 → 强制清零速度估计（防积分漂移） |
| `rBoostX / rBoostY / rBoostTheta` | 1.0 | - | 各方向 R 倍增因子（运行时状态，只读） |
| `STD_LOW_INCH / STD_HIGH_INCH` | 2.0 / 6.0 | 英寸 | 视觉位置 stdDev 映射阈值 |
| `STD_LOW_ANGLE / STD_HIGH_ANGLE` | 0.035 / 0.175 | rad | 视觉航向 stdDev 映射阈值 |
| `R_MAX_SCALE` | 20.0 | - | 视觉 R 放大上限 |
| `M_TO_INCH` | 39.37 | - | 米→英寸换算（一般不改） |
| `ACCEL_PARAMS.xFacing` | FORWARD | - | 加速度计 +X 物理轴指向的机器人方向（**构造时读取**，修改需重启） |
| `ACCEL_PARAMS.yFacing` | LEFT | - | 加速度计 +Y 物理轴指向的机器人方向；+Z 轴按右手系自动推算（Z = X × Y） |

> 默认值 {X 前 / Y 左 / Z 上} 对应恒等映射；实际加速度轴符号依赖 BNO055 安装朝向，**需实车静置/倾斜标定确认**后调整。

**R_pinpoint 自适应公式**（方向相关）：
- X（前后）：`magX = |az|/AZ_THRESHOLD + |pitchRate|/ANGULAR_VEL_THRESHOLD`，`magX > 1` 时放大 R_PIN_BASE
- Y（左右）：`magY = |az|/AZ_THRESHOLD + |rollRate|/ANGULAR_VEL_THRESHOLD`
- θ（航向）：`|yaw 角加速度|/JERK_THRESHOLD`

### 3.6 AdaptiveUKF5DLocalizer（5D 加速度计驱动）

参数与 3.5 完全相同（滤波器换成 UKF5D，含加速度计朝向 `ACCEL_PARAMS`）。

## 4. 调参提示（优先级顺序）

1. **先调基准**（固定 EKF/UKF 的 `Qbase/Rbase`）：跑一段直线+转向，观察跟踪是否滞后（Q 大）或抖动（R 小）。
2. **再调自适应阈值**：在 Dashboard 上看 `qBoostX/Y/Theta` 或 `rBoostX/Y/Theta` 是否在正常运动时误触发（阈值太小）或真冲击时不触发（阈值太大）。
3. **视觉信任曲线**：`STD_LOW/HIGH` 决定视觉在什么距离内可信；距离越远 stdDev 越大，对应 R 越大。
4. **5D 零速**：`ZERO_VEL_*` 阈值过小会导致静止时速度估计不归零、积分漂移；过大则正常慢速运动被误清零。
