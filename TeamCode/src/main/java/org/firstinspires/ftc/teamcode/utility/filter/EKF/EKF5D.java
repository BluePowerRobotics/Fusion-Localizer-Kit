package org.firstinspires.ftc.teamcode.utility.filter.EKF;

import org.ejml.simple.SimpleMatrix;

/**
 * 5 维扩展卡尔曼滤波器 (Extended Kalman Filter, 5D)。
 *
 * <p>状态向量 (5 维):
 * <pre>
 * X = [x, y, θ, Vx, Vy]ᵀ
 * </pre>
 * <ul>
 *   <li>{@code x, y}  : 场地水平位置 (英寸)</li>
 *   <li>{@code θ}     : 航向 (弧度)</li>
 *   <li>{@code Vx, Vy}: 世界坐标系下的线速度 (英寸/秒)</li>
 * </ul>
 *
 * <p>与 3 维 {@link EKF} 的核心区别在于数据角色的重构 (见
 * {@code FusionLocalizer/Theory5D.md})：
 * <ol>
 *   <li><b>外接加速度计</b> 作为 <b>控制输入</b>（体加速度，经完整姿态旋转到世界坐标系）
 *       驱动位置/速度积分；</li>
 *   <li><b>Pinpoint 里程计</b> 从控制输入变为 <b>速度/航向观测</b>（体坐标系）；</li>
 *   <li><b>Limelight</b> 仍为 <b>低频绝对位置观测</b>。</li>
 * </ol>
 *
 * <p>体加速度 → 世界加速度旋转 (pitch 绕 Robot X(右)=RR -Y 轴，roll 绕 Robot Y(前)=RR +X 轴，
 * 见 Theory5D.md §4.2):
 * <pre>
 * A = ax·cosθp - ay·sinθp·sinφ - az·sinθp·cosφ
 * B = ay·cosφ - az·sinφ
 * ax_w = A·cosψ - B·sinψ
 * ay_w = A·sinψ + B·cosψ
 * </pre>
 * 其中 {@code θp = pitch}、{@code φ = roll} (来自 Hub IMU)，{@code ψ = state θ} (航向)。
 */
public class EKF5D {

    // ==================== 状态索引 ====================

    private static final int IDX_X = 0;
    private static final int IDX_Y = 1;
    private static final int IDX_THETA = 2;
    private static final int IDX_VX = 3;
    private static final int IDX_VY = 4;

    /** 状态维度 */
    private static final int N = 5;

    // ==================== 滤波器状态 ====================

    /** 状态向量 [x, y, θ, Vx, Vy] (5x1) */
    private SimpleMatrix state;

    /** 状态协方差矩阵 (5x5) */
    private SimpleMatrix P;

    /** 过程噪声协方差矩阵 Q (5x5)，predict 中乘以 dt */
    private SimpleMatrix Q;

    /** 里程计 (速度/航向) 观测噪声协方差矩阵 R (3x3) */
    private SimpleMatrix odomR;

    /** Limelight (位置/航向) 观测噪声协方差矩阵 R (3x3) */
    private SimpleMatrix visionR;

    /** 上一次 predict 的时间戳 */
    private Double lastPredictTime = null;

    /** 上一次里程计观测的时间戳 */
    private Double lastOdomTime = null;

    /** 上一次视觉观测的时间戳 */
    private Double lastVisionTime = null;

    // ==================== 构造 ====================

    /**
     * @param initialX      初始 x (英寸)
     * @param initialY      初始 y (英寸)
     * @param initialTheta  初始航向 (弧度)
     */
    public EKF5D(double initialX, double initialY, double initialTheta) {
        state = new SimpleMatrix(new double[][]{
                {initialX},
                {initialY},
                {initialTheta},
                {0},
                {0}
        });

        // 初始协方差：位置/航向中等，速度高度不确定
        P = new SimpleMatrix(new double[][]{
                {0.5, 0,  0,    0,    0   },
                {0,   0.5, 0,    0,    0   },
                {0,   0,  0.05, 0,    0   },
                {0,   0,  0,    4.0,  0   },
                {0,   0,  0,    0,    4.0 }
        });

        // 默认过程噪声 (参考 Theory5D.md §7)
        Q = new SimpleMatrix(new double[][]{
                {0.01, 0,     0,     0,    0   },
                {0,    0.01,  0,     0,    0   },
                {0,    0,     0.002, 0,    0   },
                {0,    0,     0,     0.5,  0   },
                {0,    0,     0,     0,    0.5 }
        });

        // 默认里程计观测噪声 (vx_body, vy_body, θ)
        odomR = new SimpleMatrix(new double[][]{
                {0.1, 0,   0   },
                {0,   0.1, 0   },
                {0,   0,   0.05}
        });

        // 默认视觉观测噪声 (x, y, θ)；heading 与 mapStdToR 下限(0.01)一致，实际每帧由 adaptVisionR 覆盖
        visionR = new SimpleMatrix(new double[][]{
                {0.01, 0,    0   },
                {0,    0.01, 0   },
                {0,    0,    0.01}
        });
    }

    // ==================== 预测 ====================

    /**
     * 预测步骤 —— 每帧调用一次，使用外接加速度计 (重力已剔除的体加速度)
     * 与 Hub IMU 姿态角驱动状态传播。
     *
     * @param ax        体坐标系 X (前) 线加速度 (英寸/秒²，已剔除重力)
     * @param ay        体坐标系 Y (左) 线加速度 (英寸/秒²，已剔除重力)
     * @param az        体坐标系 Z (上) 线加速度 (英寸/秒²，已剔除重力)
     * @param pitch     俯仰角 (弧度，Hub IMU getPitch)
     * @param roll      横滚角 (弧度，Hub IMU getRoll)
     * @param omega     航向角速度 ω_pin (弧度/秒，来自 Pinpoint)
     * @param timestamp 当前时间戳 (秒)
     */
    public void predict(double ax, double ay, double az,
                        double pitch, double roll, double omega,
                        double timestamp) {
        if (lastPredictTime == null) {
            lastPredictTime = timestamp;
            return;
        }

        double dt = timestamp - lastPredictTime;
        lastPredictTime = timestamp;

        // 异常 dt 保护
        if (dt <= 0 || dt > 1.0) {
            return;
        }

        double x = state.get(IDX_X, 0);
        double y = state.get(IDX_Y, 0);
        double theta = state.get(IDX_THETA, 0);
        double vx = state.get(IDX_VX, 0);
        double vy = state.get(IDX_VY, 0);

        double cosPitch = Math.cos(pitch);
        double sinPitch = Math.sin(pitch);
        double cosRoll = Math.cos(roll);
        double sinRoll = Math.sin(roll);
        // ψ = 航向 = 状态 θ
        double cosYaw = Math.cos(theta);
        double sinYaw = Math.sin(theta);

        // ---- 体加速度 → 世界加速度 (Theory5D.md §4.2, pitch 绕右轴 → sinθ 项取负) ----
        double A = ax * cosPitch - ay * sinPitch * sinRoll - az * sinPitch * cosRoll;
        double B = ay * cosRoll - az * sinRoll;
        double axw = A * cosYaw - B * sinYaw;
        double ayw = A * sinYaw + B * cosYaw;

        // ---- 状态传播 (Theory5D.md §6.1) ----
        double dt2 = 0.5 * dt * dt;
        double xNew = x + vx * dt + axw * dt2;
        double yNew = y + vy * dt + ayw * dt2;
        double thetaNew = normalizeAngle(theta + omega * dt);
        double vxNew = vx + axw * dt;
        double vyNew = vy + ayw * dt;

        // ---- 雅可比矩阵 J = ∂f/∂state ----
        // 利用恒等式: ∂ax_w/∂θ = -ay_w, ∂ay_w/∂θ = ax_w
        SimpleMatrix J = new SimpleMatrix(N, N);
        J.set(IDX_X, IDX_X, 1);
        J.set(IDX_X, IDX_THETA, -ayw * dt2);
        J.set(IDX_X, IDX_VX, dt);

        J.set(IDX_Y, IDX_Y, 1);
        J.set(IDX_Y, IDX_THETA, axw * dt2);
        J.set(IDX_Y, IDX_VY, dt);

        J.set(IDX_THETA, IDX_THETA, 1);

        J.set(IDX_VX, IDX_THETA, -ayw * dt);
        J.set(IDX_VX, IDX_VX, 1);

        J.set(IDX_VY, IDX_THETA, axw * dt);
        J.set(IDX_VY, IDX_VY, 1);

        // ---- 协方差传播 P = J·P·Jᵀ + Q·dt ----
        P = J.mult(P).mult(J.transpose()).plus(Q.scale(dt));

        state = new SimpleMatrix(new double[][]{
                {xNew},
                {yNew},
                {thetaNew},
                {vxNew},
                {vyNew}
        });
    }

    // ==================== 里程计观测 (速度 + 航向) ====================

    /**
     * 里程计观测更新 —— 每帧调用，使用 Pinpoint 提供的体坐标系水平速度与航向。
     *
     * <p>观测向量 {@code z = [vx_body, vy_body, θ_pin]ᵀ}，由状态映射 (Theory5D.md §6.2):
     * <pre>
     * vx_body_pred = Vx·cosθ + Vy·sinθ
     * vy_body_pred = -Vx·sinθ + Vy·cosθ
     * θ_pred       = θ
     * </pre>
     *
     * @param vxBody    体坐标系 X (前) 水平速度 (英寸/秒)
     * @param vyBody    体坐标系 Y (左) 水平速度 (英寸/秒)
     * @param thetaOdom 里程计航向 (弧度)
     * @param timestamp 当前时间戳 (秒)
     */
    public void updateOdom(double vxBody, double vyBody, double thetaOdom, double timestamp) {
        if (lastOdomTime != null && timestamp <= lastOdomTime) {
            return;
        }
        lastOdomTime = timestamp;

        double theta = state.get(IDX_THETA, 0);
        double vx = state.get(IDX_VX, 0);
        double vy = state.get(IDX_VY, 0);
        double cosT = Math.cos(theta);
        double sinT = Math.sin(theta);

        double h1 = vx * cosT + vy * sinT;
        double h2 = -vx * sinT + vy * cosT;

        // ---- 新息 y = z - h ----
        SimpleMatrix z = new SimpleMatrix(new double[][]{{vxBody}, {vyBody}, {thetaOdom}});
        SimpleMatrix h = new SimpleMatrix(new double[][]{{h1}, {h2}, {theta}});
        SimpleMatrix innov = z.minus(h);
        innov.set(2, 0, normalizeAngle(innov.get(2, 0)));

        // ---- 观测雅可比 H (3x5) ----
        SimpleMatrix H = new SimpleMatrix(3, N);
        H.set(0, IDX_THETA, h2);
        H.set(0, IDX_VX, cosT);
        H.set(0, IDX_VY, sinT);
        H.set(1, IDX_THETA, -h1);
        H.set(1, IDX_VX, -sinT);
        H.set(1, IDX_VY, cosT);
        H.set(2, IDX_THETA, 1);

        // ---- S = H·P·Hᵀ + R ----
        SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(odomR);

        // ---- K = P·Hᵀ·S⁻¹ ----
        SimpleMatrix K = P.mult(H.transpose()).mult(safeInvert(S));

        // ---- 状态更新 ----
        state = state.plus(K.mult(innov));
        state.set(IDX_THETA, 0, normalizeAngle(state.get(IDX_THETA, 0)));

        // ---- 协方差更新 ----
        SimpleMatrix I = SimpleMatrix.identity(N);
        P = I.minus(K.mult(H)).mult(P);
    }

    // ==================== 视觉观测 (位置 + 航向) ====================

    /**
     * 视觉观测更新 —— 仅在 Limelight 检测有效时调用，接收全局位姿。
     *
     * @param xMeas     全局 x (英寸)
     * @param yMeas     全局 y (英寸)
     * @param thetaMeas 全局航向 (弧度)
     * @param timestamp 观测时间戳 (秒)
     */
    public void updateVision(double xMeas, double yMeas, double thetaMeas, double timestamp) {
        if (lastVisionTime != null && timestamp <= lastVisionTime) {
            return;
        }
        lastVisionTime = timestamp;

        // ---- H = [I₃ | 0] (3x5)，仅观测 x/y/θ ----
        SimpleMatrix H = new SimpleMatrix(3, N);
        H.set(0, IDX_X, 1);
        H.set(1, IDX_Y, 1);
        H.set(2, IDX_THETA, 1);

        double x = state.get(IDX_X, 0);
        double y = state.get(IDX_Y, 0);
        double theta = state.get(IDX_THETA, 0);

        SimpleMatrix z = new SimpleMatrix(new double[][]{{xMeas}, {yMeas}, {thetaMeas}});
        SimpleMatrix h = new SimpleMatrix(new double[][]{{x}, {y}, {theta}});
        SimpleMatrix innov = z.minus(h);
        innov.set(2, 0, normalizeAngle(innov.get(2, 0)));

        SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(visionR);
        SimpleMatrix K = P.mult(H.transpose()).mult(safeInvert(S));

        state = state.plus(K.mult(innov));
        state.set(IDX_THETA, 0, normalizeAngle(state.get(IDX_THETA, 0)));

        SimpleMatrix I = SimpleMatrix.identity(N);
        P = I.minus(K.mult(H)).mult(P);
    }

    /**
     * 视觉观测门控 —— 计算马氏距离 (Mahalanobis distance) 判断该视觉观测是否为离群点。
     *
     * <p>视觉观测 H = [I₃ | 0]，仅观测 x/y/θ，因此新息协方差
     * {@code S = P(0..2, 0..2) + visionR}。远离 AprilTag 时视觉误差极大，直接用其更新
     * 会将滤波器带偏；通过马氏距离门控拒绝离群观测。
     *
     * @param xMeas         视觉全局 x (英寸)
     * @param yMeas         视觉全局 y (英寸)
     * @param thetaMeas     视觉全局朝向 (弧度)
     * @param gateThreshold 马氏距离门控阈值 (无量纲)
     * @return true 表示测量可接受 (马氏距离 ≤ gateThreshold)
     */
    public boolean gateVision(double xMeas, double yMeas, double thetaMeas, double gateThreshold) {
        double x = state.get(IDX_X, 0);
        double y = state.get(IDX_Y, 0);
        double theta = state.get(IDX_THETA, 0);

        SimpleMatrix innov = new SimpleMatrix(new double[][]{
                {xMeas - x},
                {yMeas - y},
                {normalizeAngle(thetaMeas - theta)}
        });

        // S = P(0..2, 0..2) + visionR
        SimpleMatrix Psub = new SimpleMatrix(3, 3);
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                Psub.set(r, c, P.get(r, c));
            }
        }
        SimpleMatrix S = Psub.plus(visionR);
        SimpleMatrix d2 = innov.transpose().mult(safeInvert(S)).mult(innov);
        return d2.get(0, 0) <= gateThreshold * gateThreshold;
    }

    // ==================== 零速检测辅助 ====================

    /**
     * 安全求逆 —— S = H·P·Hᵀ + R 在 R 配置过小或数值退化时可能奇异。
     * 求逆失败时对角加扰动 (jitter) 后重试, 避免抛出异常或产生爆炸增益。
     */
    private SimpleMatrix safeInvert(SimpleMatrix m) {
        try {
            return m.invert();
        } catch (RuntimeException e) {
            SimpleMatrix jittered = m.plus(SimpleMatrix.identity(m.numRows()).scale(1e-9));
            return jittered.invert();
        }
    }

    /**
     * 将速度状态 Vx, Vy 强制置零 (Theory5D.md §6.3)。
     * 由外部零速检测逻辑在机器人静止时调用，防止加速度积分漂移。
     */
    public void zeroVelocity() {
        state.set(IDX_VX, 0, 0);
        state.set(IDX_VY, 0, 0);
        // 同步收缩速度协方差, 避免零速状态下不确定度仍按原值传播
        P.set(IDX_VX, IDX_VX, 0.01);
        P.set(IDX_VY, IDX_VY, 0.01);
        P.set(IDX_VX, IDX_VY, 0);
        P.set(IDX_VY, IDX_VX, 0);
    }

    // ==================== 输出 ====================

    /** @return 融合位姿 double[3] = {x, y, θ} */
    public double[] getPose() {
        return new double[]{
                state.get(IDX_X, 0),
                state.get(IDX_Y, 0),
                state.get(IDX_THETA, 0)
        };
    }

    /** @return 世界坐标系速度 double[2] = {Vx, Vy} (英寸/秒) */
    public double[] getVelocity() {
        return new double[]{
                state.get(IDX_VX, 0),
                state.get(IDX_VY, 0)
        };
    }

    /** @return 完整状态 double[5] = {x, y, θ, Vx, Vy} */
    public double[] getState() {
        double[] s = new double[N];
        for (int i = 0; i < N; i++) {
            s[i] = state.get(i, 0);
        }
        return s;
    }

    /** @return 状态协方差矩阵对角线 {var_x, var_y, var_θ, var_vx, var_vy} */
    public double[] getCovarianceDiag() {
        double[] d = new double[N];
        for (int i = 0; i < N; i++) {
            d[i] = P.get(i, i);
        }
        return d;
    }

    // ==================== 噪声参数调整 ====================

    /** 设置完整的过程噪声协方差矩阵 Q (5x5)。 */
    public void setQ(SimpleMatrix Q) {
        this.Q = Q;
    }

    /** 设置完整的里程计观测噪声协方差矩阵 R (3x3)。 */
    public void setOdomR(SimpleMatrix odomR) {
        this.odomR = odomR;
    }

    /** 设置完整的视觉观测噪声协方差矩阵 R (3x3)。 */
    public void setVisionR(SimpleMatrix visionR) {
        this.visionR = visionR;
    }

    // ==================== 重置 ====================

    /**
     * 重置滤波器到新的初始位姿，并清空时间戳与速度状态。
     */
    public void reset(double x, double y, double theta) {
        state = new SimpleMatrix(new double[][]{
                {x},
                {y},
                {theta},
                {0},
                {0}
        });
        P = new SimpleMatrix(new double[][]{
                {0.5, 0,  0,    0,    0   },
                {0,   0.5, 0,    0,    0   },
                {0,   0,  0.05, 0,    0   },
                {0,   0,  0,    4.0,  0   },
                {0,   0,  0,    0,    4.0 }
        });
        lastPredictTime = null;
        lastOdomTime = null;
        lastVisionTime = null;
    }

    // ==================== 内部工具 ====================

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}