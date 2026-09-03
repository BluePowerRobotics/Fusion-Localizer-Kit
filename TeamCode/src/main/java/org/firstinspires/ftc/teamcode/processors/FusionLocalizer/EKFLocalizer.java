package org.firstinspires.ftc.teamcode.processors.FusionLocalizer;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.processors.D3Localizer.PinpointD3Localizer;
import org.firstinspires.ftc.teamcode.processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.utility.filter.EKF.EKF;
import org.ejml.simple.SimpleMatrix;

/**
 * 简易 EKF 定位器 —— Pinpoint(里程计) + Limelight MegaTag1(视觉) + EKF 融合。
 *
 * <p>支持两种里程计模式：
 * <ul>
 *   <li><b>D2</b> (默认): {@link PinpointLocalizer}，标准 2D 里程计，无斜坡补偿</li>
 *   <li><b>D3</b>: {@link PinpointD3Localizer}，3D 斜坡补偿里程计，需要 Hub IMU</li>
 * </ul>
 *
 * <p>与 {@link AdaptiveEKFLocalizer} 的区别在于 <b>不使用自适应 Q</b>：
 * <ul>
 *   <li>Q 保持 EKF 构造时的默认值</li>
 *   <li>视觉 R 随 stdDev / 距离 / 标签数自适应，并带马氏距离门控</li>
 *   <li>不需要 IMU 硬件 (D2 模式)</li>
 * </ul>
 *
 * <p>每帧调用 {@link #update()} 即可完成：
 * <ol>
 *   <li>里程计更新 → 获取速度</li>
 *   <li>EKF 预测 (固定 Q)</li>
 *   <li>Limelight 有效时 → EKF 更新 (固定 R)</li>
 * </ol>
 */
@Config
public class EKFLocalizer implements Localizer {

    private final EKF ekf;
    /** 里程计定位器 (D2: PinpointLocalizer, D3: PinpointD3Localizer) */
    private final Localizer odom;
    private final MT1Localizer mt1;
    private final boolean useD3;

    // ---- Q/R 参数 (位置与角度独立) ----
    /** 过程噪声 — 位置 (in²/s) */
    public static double QbasePos = 0.01;
    /** 过程噪声 — 角度 (rad²/s) */
    public static double QbaseAngle = 0.01;
    /** 观测噪声 — 位置 (in²) */
    public static double RbasePos = 0.01;
    /** 观测噪声 — 角度 (rad²) */
    public static double RbaseAngle = 0.05;

    // ---- 视觉 R 自适应 (9/2 改进: 去除上界 + 距离/标签缩放 + 门控) ----
    /** 单位转换: 1 m = 39.3701 in */
    public static double M_TO_INCH = 39.37007874;
    /** stdDev 阈值 (英寸) */
    public static double STD_LOW_INCH = 2.0;
    public static double STD_HIGH_INCH = 6.0;
    /** stdDev 阈值 (弧度) */
    public static double STD_LOW_ANGLE = 0.035;
    public static double STD_HIGH_ANGLE = 0.175;
    public static double R_MAX_SCALE = 20.0;
    /** 参考距离 (米)，超过该距离视觉位置 R 随距离二次放大 */
    public static double DIST_REF_M = 1.0;
    /** 参考标签数，标签数低于该值视觉位置 R 放大 */
    public static double TAG_REF = 2.0;
    /** 标签数过少时 R 的最大放大倍数 */
    public static double TAG_SCALE_MAX = 4.0;
    /** 马氏距离门控阈值 (无量纲) */
    public static double GATE_THRESHOLD = 4.0;

    /** 最近一次里程计速度缓存 */
    private PoseVelocity2d lastVel = new PoseVelocity2d(new Vector2d(0, 0), 0);

    // ==================== 构造 ====================

    /**
     * D2 模式构造 (标准 2D 里程计)。
     *
     * @param hardwareMap  硬件映射
     * @param limelight    已启动的 Limelight3A 实例
     * @param initialPose  初始位姿 (x, y, heading)
     */
    public EKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight, Pose2d initialPose) {
        this.ekf = new EKF(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble());
        ekf.setQ(QbasePos, QbasePos, QbaseAngle);
        ekf.setR(RbasePos, RbasePos, RbaseAngle);
        this.odom = new PinpointLocalizer(hardwareMap, 0.001999, initialPose);
        this.mt1 = new MT1Localizer(limelight);
        this.useD3 = false;
    }

    /**
     * D2 模式简化构造: 初始位姿 (0, 0, 0)。
     */
    public EKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight) {
        this(hardwareMap, limelight, new Pose2d(0, 0, 0));
    }

    /**
     * D3 模式构造 (3D 斜坡补偿里程计)。
     *
     * @param hardwareMap   硬件映射
     * @param limelight     已启动的 Limelight3A 实例
     * @param imuDeviceName Hub IMU 设备名 (如 "imu")
     * @param initialPose   初始位姿 (x, y, heading)
     */
    public EKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                        String imuDeviceName, Pose2d initialPose) {
        this.ekf = new EKF(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble());
        ekf.setQ(QbasePos, QbasePos, QbaseAngle);
        ekf.setR(RbasePos, RbasePos, RbaseAngle);
        this.odom = new PinpointD3Localizer(hardwareMap, 0.001999, imuDeviceName, initialPose);
        this.mt1 = new MT1Localizer(limelight);
        this.useD3 = true;
    }

    // ==================== 核心循环 ====================

    /**
     * 每帧调用一次，完成：
     * <ol>
     *   <li>里程计更新 → 获取速度</li>
     *   <li>EKF 预测 (固定 Q)</li>
     *   <li>Limelight 有效时 → EKF 更新 (固定 R)</li>
     * </ol>
     *
     * @return 当前速度估计
     */
    @Override
    public PoseVelocity2d update() {
        double now = getNow();

        // ---- 1. 里程计速度 ----
        lastVel = odom.update();

        // ---- 2. EKF 预测 (使用固定 Q) ----
        ekf.predict(lastVel.linearVel.x, lastVel.linearVel.y, lastVel.angVel, now);

        // ---- 3. MT1 视觉 → 自适应视觉 R + 门控 + EKF 更新 ----
        mt1.update();
        if (mt1.isValid()) {
            ekf.setR(adaptVisionR());
            Pose2d visionPose = mt1.getPose();              // (英寸, 英寸, 弧度)
            if (ekf.gateVision(
                    visionPose.position.x,
                    visionPose.position.y,
                    visionPose.heading.toDouble(),
                    GATE_THRESHOLD
            )) {
                ekf.update(
                        visionPose.position.x,                  // 英寸
                        visionPose.position.y,                  // 英寸
                        visionPose.heading.toDouble(),          // 弧度
                        mt1.getTimestamp()
                );
            }
        }

        return lastVel;
    }

    // ==================== 视觉 R 自适应 (与 AdaptiveEKFLocalizer 一致) ====================

    /**
     * 基于 MT1 stdDev + 距离 + 标签数构建 3x3 对角视觉 R 矩阵 (去除上界)。
     */
    private SimpleMatrix adaptVisionR() {
        double[] stdDevs = mt1.getStdDevs();  // {x, y, z, roll, pitch, yaw} (米, 度)

        double distF = computeDistFactor();
        double tagF = computeTagFactor();

        double rX = mapStdToR(stdDevs[0] * M_TO_INCH, STD_LOW_INCH, STD_HIGH_INCH) * distF * tagF;
        double rY = mapStdToR(stdDevs[1] * M_TO_INCH, STD_LOW_INCH, STD_HIGH_INCH) * distF * tagF;
        double rTheta = mapStdToR(Math.toRadians(stdDevs[5]), STD_LOW_ANGLE, STD_HIGH_ANGLE);

        SimpleMatrix R = new SimpleMatrix(3, 3);
        R.set(0, 0, rX);
        R.set(1, 1, rY);
        R.set(2, 2, rTheta);
        return R;
    }

    private double mapStdToR(double std, double low, double high) {
        if (std <= low) {
            return 0.01;
        }
        // 线性增长, 超过 high 时继续线性放大, 不再截断上界 (9/2 改进)
        double t = (std - low) / (high - low);
        return 0.01 * (1.0 + t * (R_MAX_SCALE - 1.0));
    }

    /**
     * 视觉位置观测噪声的距离缩放因子（远离 tag 时二次放大）。
     */
    private double computeDistFactor() {
        double dist = mt1.getAvgDist();  // 米
        if (Double.isNaN(dist) || Double.isInfinite(dist) || dist <= DIST_REF_M) {
            return 1.0;
        }
        double ratio = dist / DIST_REF_M;
        return ratio * ratio;
    }

    /**
     * 视觉位置观测噪声的标签数缩放因子（标签过少时放大）。
     */
    private double computeTagFactor() {
        int tags = mt1.getTagCount();
        if (tags >= TAG_REF) {
            return 1.0;
        }
        if (tags <= 0) {
            return TAG_SCALE_MAX;
        }
        double ratio = TAG_REF / tags;
        return Math.min(TAG_SCALE_MAX, ratio);
    }

    // ==================== Localizer 接口 ====================

    /** 设置定位器位姿。 */
    @Override
    public void setPose(Pose2d pose) {
        ekf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        odom.setPose(pose);
    }

    // ==================== Q/R 设置接口 ====================

    /**
     * 允许外部手动调整 Q 矩阵。
     * 如需自适应 Q，可调用此接口传入自定义值。
     */
    public void setQ(double qx, double qy, double qtheta) {
        ekf.setQ(qx, qy, qtheta);
    }

    /**
     * 允许外部手动调整 R 矩阵。
     * 如需自适应 R，可调用此接口传入自定义值。
     */
    public void setR(double rx, double ry, double rtheta) {
        ekf.setR(rx, ry, rtheta);
    }

    // ==================== 输出 ====================

    /** @return 融合后的位姿 {x, y, heading} (英寸, 英寸, 弧度) */
    @Override
    public Pose2d getPose() {
        double[] pose = ekf.getPose();
        return new Pose2d(pose[0], pose[1], pose[2]);
    }

    /** @return 原始 EKF 实例 */
    public EKF getEKF() { return ekf; }

    /** @return MT1 视觉定位器 */
    public MT1Localizer getMT1() { return mt1; }

    /** @return 里程计定位器 (D2: PinpointLocalizer, D3: PinpointD3Localizer) */
    public Localizer getOdom() { return odom; }

    /** @return 是否为 D3 模式 */
    public boolean isD3() { return useD3; }

    // ==================== 重置 ====================

    /** 重置定位到指定位姿。 */
    public void reset(Pose2d pose) {
        ekf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        odom.setPose(pose);
    }

    // ==================== 内部工具 ====================

    private double getNow() {
        return System.nanoTime() / 1e9;
    }
}