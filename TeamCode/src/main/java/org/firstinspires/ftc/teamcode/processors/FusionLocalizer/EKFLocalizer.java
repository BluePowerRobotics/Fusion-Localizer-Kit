package org.firstinspires.ftc.teamcode.processors.FusionLocalizer;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.utility.filter.EKF.EKF;

/**
 * 简易 EKF 定位器 —— Pinpoint(里程计) + Limelight MegaTag1(视觉) + EKF 融合。
 *
 * <p>与 {@link AdaptiveEKFLocalizer} 的区别在于 <b>不使用自适应 Q/R</b>：
 * <ul>
 *   <li>Q 和 R 保持 EKF 构造时的默认值</li>
 *   <li>不需要 IMU 硬件</li>
 *   <li>不需要视觉 stdDev 映射</li>
 * </ul>
 *
 * <p>每帧调用 {@link #update()} 即可完成：
 * <ol>
 *   <li>Pinpoint 更新 → 获取速度</li>
 *   <li>EKF 预测 (固定 Q)</li>
 *   <li>Limelight 有效时 → EKF 更新 (固定 R)</li>
 * </ol>
 */
public class EKFLocalizer implements Localizer {

    private final EKF ekf;
    private final PinpointLocalizer pinpoint;
    private final MT1Localizer mt1;

    /** 单位转换: 1 m = 39.3701 in */
    private static final double M_TO_INCH = 39.37007874;

    // ---- 时间基准 ----
    private double lastTimestamp = 0;

    /** 最近一次 Pinpoint 速度缓存 */
    private PoseVelocity2d lastVel = new PoseVelocity2d(new Vector2d(0, 0), 0);

    public static double Qbase = 0.01;

    public static double Rbase = 0.01;

    // ==================== 构造 ====================

    /**
     * @param hardwareMap  硬件映射
     * @param limelight    已启动的 Limelight3A 实例
     * @param initialPose  初始位姿 (x, y, heading)
     */
    public EKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight, Pose2d initialPose) {
        this.ekf = new EKF(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble());
        ekf.setQ(Qbase, Qbase, Qbase);
        ekf.setR(Rbase, Rbase, Rbase);
        this.pinpoint = new PinpointLocalizer(hardwareMap, 0.001999, initialPose);
        this.mt1 = new MT1Localizer(limelight);
        this.lastTimestamp = getNow();
    }

    /**
     * 简化构造: 初始位姿 (0, 0, 0)。
     */
    public EKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight) {
        this(hardwareMap, limelight, new Pose2d(0, 0, 0));
    }

    // ==================== 核心循环 ====================

    /**
     * 每帧调用一次，完成：
     * <ol>
     *   <li>Pinpoint 更新 → 获取速度</li>
     *   <li>EKF 预测 (固定 Q)</li>
     *   <li>Limelight 有效时 → EKF 更新 (固定 R)</li>
     * </ol>
     *
     * @return 当前速度估计
     */
    @Override
    public PoseVelocity2d update() {
        double now = getNow();
        lastTimestamp = now;

        // ---- 1. Pinpoint 速度 ----
        lastVel = pinpoint.update();

        // ---- 2. EKF 预测 (使用固定 Q) ----
        ekf.predict(lastVel.linearVel.x, lastVel.linearVel.y, lastVel.angVel, now);

        // ---- 3. MT1 视觉 → EKF 更新 (使用固定 R) ----
        mt1.update();
        if (mt1.isValid()) {
            Pose2d visionPose = mt1.getPose();              // (米, 米, 弧度)
            ekf.update(
                    visionPose.position.x * M_TO_INCH,      // 米 → 英寸
                    visionPose.position.y * M_TO_INCH,      // 米 → 英寸
                    visionPose.heading.toDouble(),          // 弧度不变
                    mt1.getTimestamp()
            );
        }

        return lastVel;
    }

    // ==================== Localizer 接口 ====================

    /** 设置定位器位姿。 */
    @Override
    public void setPose(Pose2d pose) {
        ekf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        pinpoint.setPose(pose);
        lastTimestamp = getNow();
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

    /** @return Pinpoint 定位器 */
    public PinpointLocalizer getPinpoint() { return pinpoint; }

    // ==================== 重置 ====================

    /** 重置定位到指定位姿。 */
    public void reset(Pose2d pose) {
        ekf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        pinpoint.setPose(pose);
        lastTimestamp = getNow();
    }

    // ==================== 内部工具 ====================

    private double getNow() {
        return System.nanoTime() / 1e9;
    }
}