package org.firstinspires.ftc.teamcode.utility.filter.UKF;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.CholeskyDecomposition_F64;
import org.ejml.simple.SimpleMatrix;

/**
 * 5 维无迹卡尔曼滤波器 (Unscented Kalman Filter, 5D)。
 *
 * <p>状态向量与 {@link org.firstinspires.ftc.teamcode.utility.filter.EKF.EKF5D EKF5D}
 * 一致：
 * <pre>
 * X = [x, y, θ, Vx, Vy]ᵀ
 * </pre>
 *
 * <p>与 3 维 {@link UKF} 相比，本类通过 sigma 点无迹变换传播 5 维状态分布，
 * 每个 sigma 点拥有独立的航向，因此加速度旋转对每个 sigma 点单独进行
 * (Theory5D.md §10 注 4)。数据角色与 EKF5D 相同：
 * <ul>
 *   <li>外接加速度计 → 控制输入 (体加速度，重力已剔除)</li>
 *   <li>Pinpoint → 速度/航向观测 (体坐标系)</li>
 *   <li>Limelight → 低频绝对位置观测</li>
 * </ul>
 *
 * <p>UKF 参数: α=1, β=2, κ=0 → λ=0, 共 2n+1 = 11 个 sigma 点。
 */
public class UKF5D {

    // ==================== UKF 参数 (n=5) ====================

    private static final int IDX_X = 0;
    private static final int IDX_Y = 1;
    private static final int IDX_THETA = 2;
    private static final int IDX_VX = 3;
    private static final int IDX_VY = 4;

    /** 状态维度 */
    private static final int N = 5;

    private static final double ALPHA = 1.0;
    private static final double BETA = 2.0;
    private static final double KAPPA = 0.0;

    /** λ = α²(n+κ) - n = 5 - 5 = 0 */
    private final double lambda;
    /** n + λ = 5 */
    private final double nPlusLambda;

    private static final int NUM_SIGMA = 2 * N + 1;

    private final double[] wm;
    private final double[] wc;

    // ==================== Cholesky 正则化 ====================

    private static final double CHOL_EPS_1 = 1e-6;
    private static final double CHOL_EPS_2 = 1e-3;

    // ==================== 滤波器状态 ====================

    private SimpleMatrix state;   // 5x1
    private SimpleMatrix P;       // 5x5
    private SimpleMatrix Q;       // 5x5
    private SimpleMatrix odomR;   // 3x3
    private SimpleMatrix visionR; // 3x3

    private Double lastPredictTime = null;
    private Double lastOdomTime = null;
    private Double lastVisionTime = null;

    // ==================== 构造 ====================

    public UKF5D(double initialX, double initialY, double initialTheta) {
        state = new SimpleMatrix(new double[][]{
                {initialX}, {initialY}, {initialTheta}, {0}, {0}
        });

        P = new SimpleMatrix(new double[][]{
                {0.5, 0,  0,    0,    0   },
                {0,   0.5, 0,    0,    0   },
                {0,   0,  0.05, 0,    0   },
                {0,   0,  0,    4.0,  0   },
                {0,   0,  0,    0,    4.0 }
        });

        Q = new SimpleMatrix(new double[][]{
                {0.01, 0,     0,     0,    0   },
                {0,    0.01,  0,     0,    0   },
                {0,    0,     0.002, 0,    0   },
                {0,    0,     0,     0.5,  0   },
                {0,    0,     0,     0,    0.5 }
        });

        odomR = new SimpleMatrix(new double[][]{
                {0.1, 0,   0   },
                {0,   0.1, 0   },
                {0,   0,   0.05}
        });

        visionR = new SimpleMatrix(new double[][]{
                {0.01, 0,    0   },
                {0,    0.01, 0   },
                {0,    0,    0.01}
        });

        this.lambda = ALPHA * ALPHA * (N + KAPPA) - N;
        this.nPlusLambda = N + lambda;

        this.wm = new double[NUM_SIGMA];
        this.wc = new double[NUM_SIGMA];

        wm[0] = lambda / nPlusLambda;
        wc[0] = lambda / nPlusLambda + (1 - ALPHA * ALPHA + BETA);

        double commonWeight = 1.0 / (2.0 * nPlusLambda);
        for (int i = 1; i < NUM_SIGMA; i++) {
            wm[i] = commonWeight;
            wc[i] = commonWeight;
        }
    }

    // ==================== 预测 ====================

    /**
     * 预测步骤 —— 使用外接加速度计与 Hub IMU 姿态驱动状态传播。
     * 逐 sigma 点通过非线性模型，再计算加权均值与协方差。
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
        if (dt <= 0 || dt > 1.0) {
            return;
        }

        double cosPitch = Math.cos(pitch);
        double sinPitch = Math.sin(pitch);
        double cosRoll = Math.cos(roll);
        double sinRoll = Math.sin(roll);
        // A/B 仅依赖 pitch/roll 与体加速度，与 sigma 点航向无关 (pitch 绕右轴 → sinθ 项取负)
        double A = ax * cosPitch - ay * sinPitch * sinRoll - az * sinPitch * cosRoll;
        double B = ay * cosRoll - az * sinRoll;

        SimpleMatrix[] sigma = generateSigmaPoints();
        SimpleMatrix[] prop = new SimpleMatrix[NUM_SIGMA];
        double dt2 = 0.5 * dt * dt;

        for (int i = 0; i < NUM_SIGMA; i++) {
            double sx = sigma[i].get(IDX_X, 0);
            double sy = sigma[i].get(IDX_Y, 0);
            double st = sigma[i].get(IDX_THETA, 0);
            double svx = sigma[i].get(IDX_VX, 0);
            double svy = sigma[i].get(IDX_VY, 0);

            double cosYaw = Math.cos(st);
            double sinYaw = Math.sin(st);
            double axw = A * cosYaw - B * sinYaw;
            double ayw = A * sinYaw + B * cosYaw;

            prop[i] = new SimpleMatrix(new double[][]{
                    {sx + svx * dt + axw * dt2},
                    {sy + svy * dt + ayw * dt2},
                    {st + omega * dt},
                    {svx + axw * dt},
                    {svy + ayw * dt}
            });
        }

        SimpleMatrix mean = weightedMean(prop, N);
        mean.set(IDX_THETA, 0, normalizeAngle(mean.get(IDX_THETA, 0)));
        P = covariance(prop, mean).plus(Q.scale(dt));
        state = mean;
    }

    // ==================== 里程计观测 (速度 + 航向) ====================

    /**
     * 里程计观测更新。观测 z = [vx_body, vy_body, θ_pin]。
     */
    public void updateOdom(double vxBody, double vyBody, double thetaOdom, double timestamp) {
        if (lastOdomTime != null && timestamp <= lastOdomTime) {
            return;
        }
        lastOdomTime = timestamp;

        SimpleMatrix[] sigma = generateSigmaPoints();
        SimpleMatrix[] zSigma = new SimpleMatrix[NUM_SIGMA];
        for (int i = 0; i < NUM_SIGMA; i++) {
            zSigma[i] = odomObservation(sigma[i]);
        }

        SimpleMatrix zMean = weightedMean(zSigma, 3);

        SimpleMatrix z = new SimpleMatrix(new double[][]{{vxBody}, {vyBody}, {thetaOdom}});
        SimpleMatrix innov = z.minus(zMean);
        innov.set(2, 0, normalizeAngle(innov.get(2, 0)));

        SimpleMatrix S = covariance(zSigma, zMean).plus(odomR);
        SimpleMatrix Pxz = crossCovariance(sigma, state, zSigma, zMean);

        SimpleMatrix K = Pxz.mult(safeInvert(S));

        state = state.plus(K.mult(innov));
        state.set(IDX_THETA, 0, normalizeAngle(state.get(IDX_THETA, 0)));

        P = P.minus(K.mult(S).mult(K.transpose()));
        P = P.plus(P.transpose()).scale(0.5);
    }

    // ==================== 视觉观测 (位置 + 航向) ====================

    /**
     * 视觉观测更新。观测 z = [x, y, θ]。
     */
    public void updateVision(double xMeas, double yMeas, double thetaMeas, double timestamp) {
        if (lastVisionTime != null && timestamp <= lastVisionTime) {
            return;
        }
        lastVisionTime = timestamp;

        SimpleMatrix[] sigma = generateSigmaPoints();
        SimpleMatrix[] zSigma = new SimpleMatrix[NUM_SIGMA];
        for (int i = 0; i < NUM_SIGMA; i++) {
            zSigma[i] = visionObservation(sigma[i]);
        }

        SimpleMatrix zMean = weightedMean(zSigma, 3);

        SimpleMatrix z = new SimpleMatrix(new double[][]{{xMeas}, {yMeas}, {thetaMeas}});
        SimpleMatrix innov = z.minus(zMean);
        innov.set(2, 0, normalizeAngle(innov.get(2, 0)));

        SimpleMatrix S = covariance(zSigma, zMean).plus(visionR);
        SimpleMatrix Pxz = crossCovariance(sigma, state, zSigma, zMean);

        SimpleMatrix K = Pxz.mult(safeInvert(S));

        state = state.plus(K.mult(innov));
        state.set(IDX_THETA, 0, normalizeAngle(state.get(IDX_THETA, 0)));

        P = P.minus(K.mult(S).mult(K.transpose()));
        P = P.plus(P.transpose()).scale(0.5);
    }

    // ==================== 零速检测辅助 ====================

    public void zeroVelocity() {
        state.set(IDX_VX, 0, 0);
        state.set(IDX_VY, 0, 0);
    }

    // ==================== 输出 ====================

    public double[] getPose() {
        return new double[]{
                state.get(IDX_X, 0),
                state.get(IDX_Y, 0),
                state.get(IDX_THETA, 0)
        };
    }

    public double[] getVelocity() {
        return new double[]{
                state.get(IDX_VX, 0),
                state.get(IDX_VY, 0)
        };
    }

    public double[] getState() {
        double[] s = new double[N];
        for (int i = 0; i < N; i++) {
            s[i] = state.get(i, 0);
        }
        return s;
    }

    public double[] getCovarianceDiag() {
        double[] d = new double[N];
        for (int i = 0; i < N; i++) {
            d[i] = P.get(i, i);
        }
        return d;
    }

    // ==================== 噪声参数调整 ====================

    public void setQ(SimpleMatrix Q) {
        this.Q = Q;
    }

    public void setOdomR(SimpleMatrix odomR) {
        this.odomR = odomR;
    }

    public void setVisionR(SimpleMatrix visionR) {
        this.visionR = visionR;
    }

    // ==================== 重置 ====================

    public void reset(double x, double y, double theta) {
        state = new SimpleMatrix(new double[][]{
                {x}, {y}, {theta}, {0}, {0}
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

    // ==================== 观测模型 ====================

    private SimpleMatrix odomObservation(SimpleMatrix s) {
        double theta = s.get(IDX_THETA, 0);
        double vx = s.get(IDX_VX, 0);
        double vy = s.get(IDX_VY, 0);
        double h1 = vx * Math.cos(theta) + vy * Math.sin(theta);
        double h2 = -vx * Math.sin(theta) + vy * Math.cos(theta);
        return new SimpleMatrix(new double[][]{{h1}, {h2}, {theta}});
    }

    private SimpleMatrix visionObservation(SimpleMatrix s) {
        return new SimpleMatrix(new double[][]{
                {s.get(IDX_X, 0)},
                {s.get(IDX_Y, 0)},
                {s.get(IDX_THETA, 0)}
        });
    }

    // ==================== Sigma 点生成 ====================

    private SimpleMatrix[] generateSigmaPoints() {
        SimpleMatrix[] sigma = new SimpleMatrix[NUM_SIGMA];
        DMatrixRMaj L = choleskyDecompose(P, nPlusLambda);

        sigma[0] = state.copy();
        for (int j = 0; j < N; j++) {
            SimpleMatrix col = new SimpleMatrix(N, 1);
            for (int r = 0; r < N; r++) {
                col.set(r, 0, L.get(r, j));
            }
            sigma[j + 1] = state.plus(col);
            sigma[j + N + 1] = state.minus(col);
        }
        return sigma;
    }

    // ==================== Cholesky 分解 (含正则化降级) ====================

    private DMatrixRMaj choleskyDecompose(SimpleMatrix Pmat, double scale) {
        CholeskyDecomposition_F64<DMatrixRMaj> chol =
                DecompositionFactory_DDRM.chol(N, true);

        DMatrixRMaj S = toDenseScaled(Pmat, scale);
        if (chol.decompose(S)) {
            return extractL(chol);
        }

        S = toDenseScaledRegularized(Pmat, scale, CHOL_EPS_1);
        if (chol.decompose(S)) {
            return extractL(chol);
        }

        S = toDenseScaledRegularized(Pmat, scale, CHOL_EPS_2);
        if (chol.decompose(S)) {
            return extractL(chol);
        }

        double trace = 0;
        for (int i = 0; i < N; i++) {
            trace += Pmat.get(i, i);
        }
        double diag = Math.sqrt(Math.max(scale * trace / N, 1e-12));

        DMatrixRMaj L = new DMatrixRMaj(N, N);
        for (int i = 0; i < N; i++) {
            L.set(i, i, diag);
        }
        return L;
    }

    private DMatrixRMaj toDenseScaled(SimpleMatrix Pmat, double scale) {
        DMatrixRMaj result = new DMatrixRMaj(N, N);
        for (int r = 0; r < N; r++) {
            for (int c = 0; c < N; c++) {
                result.set(r, c, scale * Pmat.get(r, c));
            }
        }
        return result;
    }

    private DMatrixRMaj toDenseScaledRegularized(SimpleMatrix Pmat, double scale, double eps) {
        DMatrixRMaj result = new DMatrixRMaj(N, N);
        for (int r = 0; r < N; r++) {
            for (int c = 0; c < N; c++) {
                double v = scale * Pmat.get(r, c);
                if (r == c) {
                    v += eps;
                }
                result.set(r, c, v);
            }
        }
        return result;
    }

    private DMatrixRMaj extractL(CholeskyDecomposition_F64<DMatrixRMaj> chol) {
        DMatrixRMaj L = new DMatrixRMaj(N, N);
        chol.getT(L);
        return L;
    }

    // ==================== 统计辅助 ====================

    private SimpleMatrix weightedMean(SimpleMatrix[] pts, int dim) {
        SimpleMatrix mean = new SimpleMatrix(dim, 1);
        for (int k = 0; k < dim; k++) {
            boolean circular = (dim >= 3 && k == IDX_THETA);
            if (circular) {
                double s = 0, c = 0;
                for (int i = 0; i < NUM_SIGMA; i++) {
                    s += wm[i] * Math.sin(pts[i].get(k, 0));
                    c += wm[i] * Math.cos(pts[i].get(k, 0));
                }
                mean.set(k, 0, Math.atan2(s, c));
            } else {
                double s = 0;
                for (int i = 0; i < NUM_SIGMA; i++) {
                    s += wm[i] * pts[i].get(k, 0);
                }
                mean.set(k, 0, s);
            }
        }
        return mean;
    }

    private SimpleMatrix covariance(SimpleMatrix[] pts, SimpleMatrix mean) {
        int dim = mean.numRows();
        SimpleMatrix cov = new SimpleMatrix(dim, dim);
        for (int i = 0; i < NUM_SIGMA; i++) {
            SimpleMatrix d = pts[i].minus(mean);
            if (dim >= 3) {
                d.set(IDX_THETA, 0, normalizeAngle(d.get(IDX_THETA, 0)));
            }
            cov = cov.plus(d.mult(d.transpose()).scale(wc[i]));
        }
        return cov;
    }

    private SimpleMatrix crossCovariance(SimpleMatrix[] x, SimpleMatrix xMean,
                                         SimpleMatrix[] z, SimpleMatrix zMean) {
        int dz = zMean.numRows();
        SimpleMatrix p = new SimpleMatrix(N, dz);
        for (int i = 0; i < NUM_SIGMA; i++) {
            SimpleMatrix dx = x[i].minus(xMean);
            dx.set(IDX_THETA, 0, normalizeAngle(dx.get(IDX_THETA, 0)));

            SimpleMatrix dzv = z[i].minus(zMean);
            if (dz >= 3) {
                dzv.set(IDX_THETA, 0, normalizeAngle(dzv.get(IDX_THETA, 0)));
            }
            p = p.plus(dx.mult(dzv.transpose()).scale(wc[i]));
        }
        return p;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    /**
     * 安全求逆 —— S = Pzz + R 在 R 配置过小或数值退化时可能奇异。
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
}