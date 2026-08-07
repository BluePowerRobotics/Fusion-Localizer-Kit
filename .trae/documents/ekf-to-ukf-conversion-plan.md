# EKF → UKF 转换实施计划

## Context

当前 FusionLocalizer 使用 **EKF (扩展卡尔曼滤波器)** 进行 Pinpoint 里程计 + Limelight MegaTag1 视觉融合定位。EKF 通过一阶泰勒展开 (雅可比矩阵) 线性化非线性运动模型，在高动态旋转场景下存在线性化误差。**UKF (无迹卡尔曼滤波器)** 通过 sigma 点无迹变换直接传播分布，无需计算雅可比矩阵，可精确到三阶矩，更适合 FTC 机器人高动态场景。

## 技术方案

### UKF 参数设计

| 参数 | 值 | 说明 |
|------|-----|------|
| 状态维度 n | 3 | [x, y, θ] |
| α | 1.0 | 控制 sigma 点散布 |
| β | 2.0 | 最优 (高斯分布) |
| κ | 0.0 | 次级缩放参数 |
| λ | 0 | α²(n+κ) - n = 0 |
| n+λ | 3 | 缩放因子 |
| Sigma 点数 | 7 | 2n+1 |
| w₀_m | 0 | 均值权重 |
| w₀_c | 2 | 协方差权重 |
| wᵢ (i=1..6) | 1/6 | 均值/协方差权重 |

### 核心算法差异

**Predict 步骤**:
- EKF: 1 次状态传播 + 雅可比 `A = ∂f/∂x` + `P = A·P·Aᵀ + Q·dt`
- UKF: 生成 7 个 sigma 点 → 分别传播 → 加权平均 (均值用圆形均值) → 加权协方差 + Q·dt

**Update 步骤** (H=I 线性观测):
- EKF: `K = P·(P+R)⁻¹` → `x = x + K·(z-x)` → `P = (I-K)·P`
- UKF: sigma 点 → `S = Σw_c·(Zᵢ-ẑ)(Zᵢ-ẑ)ᵀ + R` → `Pxz = Σw_c·(Xᵢ-x̂)(Zᵢ-ẑ)ᵀ` → `K = Pxz·S⁻¹` → Joseph 形式 `P = P - K·S·Kᵀ`

### 矩阵平方根

使用 EJML `DecompositionFactory_DDRM.chol()` 计算 `sqrt((n+λ)·P)`，三级正则化降级策略：
1. 直接 Cholesky
2. 失败 → `P + 1e-6·I`
3. 仍失败 → `P + 1e-3·I`
4. 仍失败 → 对角 fallback `sqrt(trace(P)/3)·I`

## 新建文件清单

### 1. UKF.java (核心滤波器)
**路径**: `f:\github\FusionLocalizerKit\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\utility\filter\UKF\UKF.java`

完全兼容 EKF.java 的 API，新增内容：
- `choleskyDecompose()` — 三级正则化 Cholesky 分解
- `generateSigmaPoints()` — 生成 7 个 sigma 点
- `predict()` — 无迹变换预测 (圆形均值)
- `update()` — Joseph 形式无迹更新
- 新增 imports: `DecompositionFactory_DDRM`, `CholeskyDecomposition_F64`, `DMatrixRMaj`

### 2. UKFLocalizer.java (简易定位器)
**路径**: `f:\github\FusionLocalizerKit\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\processors\FusionLocalizer\UKFLocalizer.java`

EKFLocalizer.java 的 UKF 版本，差异：
- `import EKF` → `import UKF`
- `private final EKF ekf` → `private final UKF ukf`
- `new EKF(...)` → `new UKF(...)`
- `getEKF()` → `getUKF()`
- 其余逻辑 (Q/R 设置、update 循环、D2/D3 构造) 完全不变

### 3. AdaptiveUKFLocalizer.java (自适应定位器)
**路径**: `f:\github\FusionLocalizerKit\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\processors\FusionLocalizer\AdaptiveUKFLocalizer.java`

AdaptiveEKFLocalizer.java 的 UKF 版本，差异：
- `import EKF` → `import UKF`
- `private final EKF ekf` → `private final UKF ukf`
- `new EKF(...)` → `new UKF(...)`
- 所有 `ekf.xxx()` → `ukf.xxx()`
- `getEKF()` → `getUKF()`
- 自适应 Q/R 逻辑 (IMU 检测、stdDev 映射、D2/D3 策略、boost/decay) 完全不变

## 不修改的现有文件

- `EKF.java` — 保留原样
- `EKFLocalizer.java` — 保留原样
- `AdaptiveEKFLocalizer.java` — 保留原样
- `Theory.md` — 后续可补充 UKF 原理章节

## 数值稳定性保障

| 防护措施 | 触发条件 | 策略 |
|---------|---------|------|
| Cholesky 正则化 | P 非正定 | 三级 εI 递增 → 对角 fallback |
| Joseph 形式协方差 | 每次 update | `P = P - K·S·Kᵀ` 替代 `(I-KH)·P` |
| 对称性强制 | 每次 update 后 | `P = (P + Pᵀ)/2` |
| 圆形均值 | 每次 predict | `θ_mean = atan2(Σ w·sinθ, Σ w·cosθ)` |
| 角度差归一化 | 协方差计算 | 所有 dθ 都 normalize 到 [-π, π] |

## 注意事项
实现时务必注意每个量的单位，正方向，定义一致性，相对还是绝对坐标系等；可参照现有代码和文档检查。

## 验证方案

1. **编译验证**: 确保 `./gradlew :TeamCode:compileDebugJava` 通过
2. **静态数值验证**: 编写测试 OpMode 对 EKF 和 UKF 使用相同输入序列，对比轨迹
3. **Sigma 点验证**: 确认对角 P 产生的 sigma 点值正确 (`sqrt(0.03) ≈ 0.1732`)
4. **权重验证**: 确认 `Σwm = 1.0`, `Σwc = 3.0`
5. **Cholesky 降级测试**: 人工构造非正定 P，验证正则化 fallback 不崩溃
6. **实机集成测试**: 替换 OpMode 中的定位器，运行标准自动程序，对比定位精度