# 融合定位器可调参数速查表

> 对应测试 OpMode：[`FusionTestOpMode`](../../opmodes/FusionTestOpMode.java)（D2）与 [`FusionD3TestOpmode`](../../opmodes/FusionD3TestOpmode.java)（D3/5D）。
>
> 所有 `public static` 字段可在 FTC Dashboard 的 "Config" 页实时修改。注意：**EKFLocalizer / UKFLocalizer 的过程噪声 Q 与初始 R 在构造时一次性读取**（视觉 R 仍每帧自适应，见下方共用说明），运行中修改需重启 OpMode 生效；**自适应版本每帧实时读取**，修改立即生效。

## 视觉 R 自适应与马氏门控（所有定位器共用，9/2 改进）

所有 8 个定位器（EKF/UKF、3D/5D、D2/D3、基础/自适应）在视觉更新前使用同一套自适应 R 逻辑与马氏距离门控：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `M_TO_INCH` | 39.37007874 | 米 → 英寸换算 |
| `STD_LOW_INCH` / `STD_HIGH_INCH` | 2.0 / 6.0 in | 视觉位置 stdDev 线性映射低/高阈值 |
| `STD_LOW_ANGLE` / `STD_HIGH_ANGLE` | 0.035 / 0.175 rad | 视觉航向 stdDev 线性映射低/高阈值 |
| `R_MAX_SCALE` | 20.0 | std→R 线性映射斜率（**不再截断上界**） |
| `DIST_REF_M` | 1.0 m | 距离缩放参考距离（超出后二次放大） |
| `TAG_REF` | 2.0 | 标签数缩放参考标签数 |
| `TAG_SCALE_MAX` | 4.0 | 标签过少时 R 放大上限 |
| `GATE_THRESHOLD` | 4.0 | 马氏距离门控阈值（无量纲） |

映射与门控规则：

```
R_dir   = 0.01                                  # std <= LOW
        = 0.01 × (1 + t × (R_MAX_SCALE - 1))    # 否则, t=(std-LOW)/(HIGH-LOW), 无上界
rX      = R_x × distFactor × tagFactor
rY      = R_y × distFactor × tagFactor
rTheta  = R_theta                               # 角度不乘距离/标签因子
distFactor = dist <= DIST_REF_M ? 1 : (dist/DIST_REF_M)²
tagFactor  = tags >= TAG_REF ? 1 : min(TAG_SCALE_MAX, TAG_REF/tags)   (tags<=0 → TAG_SCALE_MAX)
```

更新前先 `gateVision()` 计算马氏距离 `d² = innovᵀ·S⁻¹·innov`，仅 `d² <= GATE_THRESHOLD²` 才执行视觉更新。

> 注：`R_MAX_SCALE` 旧语义是「R 放大上限」；9/2 起改为「线性映射斜率」，实际 R 可随 stdDev/距离/标签数继续增大，不再封顶。

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
| `R_MAX_SCALE` | 20.0 | - | std→R 线性映射斜率（**不再截断上界**） |
| `rBase` | 0.01 | - | R 基值缓存（运行时状态，仅调试展示） |
| `M_TO_INCH` | 39.37 | - | 米→英寸单位换算（一般不改） |

**R 自适应映射**：`std ≤ LOW → R=0.01`；否则线性放大 `R=0.01×(1+t×(R_MAX_SCALE-1))`（**无上界**），位置项再乘 `distFactor×tagFactor`，更新前过马氏门控（见顶部共用说明）。

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
| `R_MAX_SCALE` | 20.0 | - | std→R 线性映射斜率（不再截断上界） |
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
| `R_MAX_SCALE` | 20.0 | - | std→R 线性映射斜率（不再截断上界） |
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

## 4. 5D D2 简化变体（AdaptiveEKF5DLocalizer_D2 / AdaptiveUKF5DLocalizer_D2）

> 作为 D3 5D 定位器的简化版，固定使用标准 2D 里程计 `PinpointLocalizer`，其余（外接加速度计预测、Q 固定、视觉 R 逻辑）与 D3 5D 相同。区别仅在 **R_pin 判定改为「仅用角度」**：pitch 倾角 → X 速度 R、roll 倾角 → Y 速度 R、yaw 角加速度 → θ（yaw 仍用角加速度）。

| 参数 | 默认值 | 单位 | 含义 |
|---|---|---|---|
| `R_PIN_BASE` | 0.1 | in²/s² | Pinpoint 速度观测噪声基值 |
| `R_PIN_THETA_BASE` | 0.05 | rad² | Pinpoint 航向观测噪声基值 |
| `ANGLE_THRESHOLD` | 0.15 | rad (≈8.6°) | pitch/roll 倾角判定阈值（超过则放大对应速度 R） |
| `JERK_THRESHOLD` | 4.0 | rad/s² | yaw 角加速度阈值（旋转冲击） |
| `R_BOOST_MAX` | 20.0 | - | R_pin 倍增因子上限 |
| `R_DECAY` | 0.85 | - | R_pin 倍增因子衰减系数 |
| `ZERO_VEL_ACCEL_THRESHOLD` | 0.5 | in/s² | 零速检测加速度阈值 |
| `ZERO_VEL_PINPOINT_THRESHOLD` | 0.5 | in/s | 零速检测速度阈值 |

- 本变体**不使用** `AZ_THRESHOLD` / `ANGULAR_VEL_THRESHOLD`（那是 D3 5D 的判据）。
- 视觉 R 参数（`STD_*`、`R_MAX_SCALE`、`DIST_REF_M`、`TAG_REF`、`TAG_SCALE_MAX`、`GATE_THRESHOLD`）与所有定位器共用，见顶部说明。
- 加速度计朝向参数 `ACCEL_PARAMS.xFacing / yFacing` 构造时读取（同 §3.5）。

## 5. 调参提示（优先级顺序）

1. **先调基准**（固定 EKF/UKF 的 `Qbase/Rbase`）：跑一段直线+转向，观察跟踪是否滞后（Q 大）或抖动（R 小）。
2. **再调自适应阈值**：在 Dashboard 上看 `qBoostX/Y/Theta` 或 `rBoostX/Y/Theta` 是否在正常运动时误触发（阈值太小）或真冲击时不触发（阈值太大）。
3. **视觉信任曲线**：`STD_LOW/HIGH` 决定视觉在什么距离内可信；距离越远 stdDev 越大，对应 R 越大。
4. **5D 零速**：`ZERO_VEL_*` 阈值过小会导致静止时速度估计不归零、积分漂移；过大则正常慢速运动被误清零。
