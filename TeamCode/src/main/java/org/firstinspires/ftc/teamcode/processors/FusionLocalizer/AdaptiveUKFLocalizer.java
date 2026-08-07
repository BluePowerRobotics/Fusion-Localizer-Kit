package org.firstinspires.ftc.teamcode.processors.FusionLocalizer;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.processors.D3Localizer.PinpointD3Localizer;
import org.firstinspires.ftc.teamcode.processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.utility.filter.UKF.UKF;

/**
 * 融合定位器 —— 里程计 + Limelight MegaTag1(视觉) + UKF 融合 + Hub IMU 自适应 Q。
 *
 * <p>支持两种里程计模式：
 * <ul>
 *   <li><b>D2</b> (默认): {@link PinpointLocalizer}，标准 2D 里程计，无斜坡补偿</li>
 *   <li><b>D3</b>: {@link PinpointD3Localizer}，3D 斜坡补偿里程计</li>
 * </ul>
 *
 * <p>每帧调用 {@link #update()} 即可完成：
 * <ol>
 *   <li>里程计速度预测 ({@link UKF#predict})</li>
 *   <li>Limelight 视觉更新 (条件触发, {@link UKF#update})</li>
 *   <li>自适应调整 Q (IMU 角速度/角加速度) 和 R (视觉 stdDev)</li>
 * </ol>
 *
 * <p><b>自适应 Q 策略</b> (D2 与 D3 不同)：
 * <ul>
 *   <li><b>D2</b>: pitch/roll 角加速度 → 冲击检测；yaw 角速度 jerk → 旋转冲击</li>
 *   <li><b>D3</b>: pitch/roll 角速度 (坡度变化) + 角加速度 (冲击) 共同调节；
 *        yaw 角加速度 (旋转冲击)；yaw 角速度与坡度无关，不参与计算</li>
 * </ul>
 *
 * <p><b>R 自适应</b>: MT1 各方向 stdDev → 各方向独立 R 矩阵
 */
@Config
public class AdaptiveUKFLocalizer implements Localizer {

    private final UKF ukf;
    /** 里程计定位器 (D2: PinpointLocalizer, D3: PinpointD3Localizer) */
    private final Localizer odom;
    private final MT1Localizer mt1;
    private final IMU hubImu;
    private final boolean useD3;

    // ---- 时间基准 ----
    private double lastTimestamp = 0;

    /** 最近一次里程计速度缓存 */
    private PoseVelocity2d lastVel = new PoseVelocity2d(new Vector2d(0, 0), 0);

    // ---- Q 自适应: IMU 检测 ----
    /** 上一帧 pitch 角速度 (rad/s) — 用于计算角加速度 */
    private double lastPitchRate = 0;
    /** 上一帧 roll 角速度 (rad/s) — 用于计算角加速度 */
    private double lastRollRate = 0;
    /** 上一帧 yaw 角速度 (rad/s) — 用于计算角加速度 (jerk) */
    private double lastYawRate = 0;

    /** Q 基值 (in²/s) */
    public static double qBase = 0.002;
    public static double qBoostX = 1.0;
    public static double qBoostY = 1.0;
    public static double qBoostTheta = 1.0;

    // ---- D2: 角加速度阈值 (冲击检测) ----
    public static double ANGULAR_ACCEL_THRESHOLD = 5.0;  // rad/s² (pitch/roll 角加速度阈值)
    public static double JERK_THRESHOLD = 4.0;           // rad/s² (yaw 角速度 jerk 阈值)

    // ---- D3: 角速度阈值 (坡度变化检测) ----
    public static double ANGULAR_VEL_THRESHOLD = 1.0;    // rad/s (pitch/roll 角速度阈值，坡度变化)
    public static double VEL_BOOST_MAX = 10.0;            // 角速度最大 Q 倍增因子 (坡度变化，较高)
    public static double ACCEL_BOOST_MAX = 4.0;         // 角加速度最大 Q 倍增因子 (冲击，较低)

    public static double Q_BOOST_MAX = 10.0;
    public static double Q_DECAY = 0.85;

    // ---- R 自适应: MT1 stdDev ----
    /** 单位转换: 1 m = 39.3701 in */
    public static double M_TO_INCH = 39.37007874;

    /** stdDev 阈值 (英寸) — 对应原 0.05m / 0.15m */
    public static double STD_LOW_INCH = 2.0;
    public static double STD_HIGH_INCH = 6.0;

    /** stdDev 阈值 (弧度) — 角度分量专用，对应 ≈2° / ≈10° */
    public static double STD_LOW_ANGLE = 0.035;   // rad (≈2°)
    public static double STD_HIGH_ANGLE = 0.175;  // rad (≈10°)

    public static double R_MAX_SCALE = 20.0;

    /** R 基值缓存 (调试用) */
    public static double rBase = 0.01;

    // ==================== 构造 ====================

    /**
     * D2 模式构造 (标准 2D 里程计)。
     *
     * @param hardwareMap    硬件映射
     * @param limelight      已启动的 Limelight3A 实例
     * @param imuDeviceName  IMU 设备名 (如 "imu")
     * @param initialPose    初始位姿 (x, y, heading)
     */
    public AdaptiveUKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                           String imuDeviceName, Pose2d initialPose) {
        this.ukf = new UKF(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble());
        this.odom = new PinpointLocalizer(hardwareMap, 0.001999, initialPose);
        this.mt1 = new MT1Localizer(limelight);
        this.hubImu = hardwareMap.get(IMU.class, imuDeviceName);
        this.useD3 = false;
        this.lastTimestamp = getNow();
    }

    /**
     * D2 模式简化构造: 使用默认 IMU 名称 "imu", 初始位姿 (0,0,0)。
     */
    public AdaptiveUKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight) {
        this(hardwareMap, limelight, "imu", new Pose2d(0, 0, 0));
    }

    /**
     * D3 模式构造 (3D 斜坡补偿里程计)。
     *
     * @param hardwareMap    硬件映射
     * @param limelight      已启动的 Limelight3A 实例
     * @param imuDeviceName  IMU 设备名 (如 "imu")，供里程计和 adaptQ 共用
     * @param initialPose    初始位姿 (x, y, heading)
     */
    public AdaptiveUKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                           String imuDeviceName, Pose2d initialPose, boolean useD3) {
        this.ukf = new UKF(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble());
        if (useD3) {
            this.odom = new PinpointD3Localizer(hardwareMap, 0.001999, imuDeviceName, initialPose);
        } else {
            this.odom = new PinpointLocalizer(hardwareMap, 0.001999, initialPose);
        }
        this.mt1 = new MT1Localizer(limelight);
        this.hubImu = hardwareMap.get(IMU.class, imuDeviceName);
        this.useD3 = useD3;
        this.lastTimestamp = getNow();
    }

    // ==================== 核心循环 ====================

    /**
     * 每帧调用一次，完成：
     * <ol>
     *   <li>里程计更新 → 获取速度</li>
     *   <li>IMU 角速度/角加速度检测 → 自适应 Q (D2/D3 策略不同)</li>
     *   <li>UKF 预测</li>
     *   <li>Limelight 更新 → 自适应 R (3x3 矩阵) + UKF 更新</li>
     * </ol>
     *
     * @return 当前速度估计
     */
    @Override
    public PoseVelocity2d update() {
        double now = getNow();
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        // ---- 1. 里程计速度 ----
        lastVel = odom.update();

        // ---- 2. IMU 检测 → 自适应 Q 矩阵 (D2/D3 策略不同) ----
        ukf.setQ(adaptQ(dt));

        // ---- 3. UKF 预测 ----
        ukf.predict(lastVel.linearVel.x, lastVel.linearVel.y, lastVel.angVel, now);

        // ---- 4. MT1 视觉 → 自适应 R 矩阵 + UKF 更新 ----
        mt1.update();
        if (mt1.isValid()) {
            ukf.setR(adaptR());
            Pose2d visionPose = mt1.getPose();              // (英寸, 英寸, 弧度)
            ukf.update(
                    visionPose.position.x,                  // 英寸
                    visionPose.position.y,                  // 英寸
                    visionPose.heading.toDouble(),          // 弧度
                    mt1.getTimestamp()
            );
        }

        return lastVel;
    }

    // ==================== 自适应 Q (SimpleMatrix 输出) ====================

    /**
     * 基于 IMU 角速度/角加速度构建 3x3 对角 Q 矩阵。
     *
     * <p><b>D2 模式</b> (仅角加速度)：
     * <ul>
     *   <li><b>x, y</b>: pitch/roll 角加速度 → 旋转到绝对坐标系 → 检测碰撞/急加速</li>
     *   <li><b>θ</b>: yaw 角速度 jerk → 检测旋转碰撞</li>
     * </ul>
     *
     * <p><b>D3 模式</b> (角速度 + 角加速度共同调节)：
     * <ul>
     *   <li><b>x, y</b>: pitch/roll 角速度 (坡度变化, 较低 boost) + 角加速度 (冲击, 较高 boost)</li>
     *   <li><b>θ</b>: yaw 角加速度 (旋转冲击)；yaw 角速度与坡度无关，不参与计算</li>
     * </ul>
     *
     * @param dt 帧间隔 (秒)
     * @return 3x3 对角过程噪声协方差矩阵 Q
     */
    private SimpleMatrix adaptQ(double dt) {
        double safeDt = Math.max(dt, 1e-6);

        // 使用补偿后航向 (含 setPose 偏移) 而非 pinpoint 原始航向:
        // 旋转 pitch/roll 角速度到场地坐标系需要机器人在场地中的真实朝向,
        // 补偿后航向 = txWorldPinpoint.heading + pinpointTheta, 代表了 UKF 跟踪的位姿朝向
        double theta = getPose().heading.toDouble();
        double cosT = Math.cos(theta);
        double sinT = Math.sin(theta);

        if (hubImu != null) {
            AngularVelocity angVel = hubImu.getRobotAngularVelocity(AngleUnit.RADIANS);
            if (angVel != null) {
                double pitchRate = angVel.xRotationRate;
                double rollRate  = angVel.yRotationRate;
                double yawRate   = angVel.zRotationRate;

                // ---- pitch/roll 角加速度 (冲击) ----
                double pitchAccel = (pitchRate - lastPitchRate) / safeDt;
                double rollAccel  = (rollRate  - lastRollRate)  / safeDt;

                // 体坐标系 → 场坐标系旋转
                double fieldX = pitchAccel * cosT - rollAccel * sinT;
                double fieldY = pitchAccel * sinT + rollAccel * cosT;

                if (useD3) {
                    // ========== D3: 角速度 + 角加速度共同调节 ==========
                    // 将角速度和角加速度均旋转到场地坐标系，各方向独立计算

                    // 角速度旋转到场地坐标系
                    double fieldVelX = pitchRate * cosT - rollRate * sinT;
                    double fieldVelY = pitchRate * sinT + rollRate * cosT;

                    // 角加速度已在上方旋转到场地坐标系 (fieldX, fieldY)
                    double absVelX   = Math.abs(fieldVelX);
                    double absAccelX = Math.abs(fieldX);
                    double absVelY   = Math.abs(fieldVelY);
                    double absAccelY = Math.abs(fieldY);

                    // 将角速度归一化到角加速度尺度，便于比较
                    // 1 rad/s 角速度 ≈ (ACCEL_THRESHOLD/VEL_THRESHOLD) rad/s² 等效角加速度
                    double velEquivX = absVelX * (ANGULAR_ACCEL_THRESHOLD / ANGULAR_VEL_THRESHOLD);
                    double velEquivY = absVelY * (ANGULAR_ACCEL_THRESHOLD / ANGULAR_VEL_THRESHOLD);

                    double effectiveMagX = velEquivX + absAccelX;
                    double effectiveMagY = velEquivY + absAccelY;

                    // 角速度主导 → 坡度变化 (较高 boost), 角加速度主导 → 冲击 (较低 boost)
                    double maxBoostX = (velEquivX > absAccelX) ? VEL_BOOST_MAX : ACCEL_BOOST_MAX;
                    double maxBoostY = (velEquivY > absAccelY) ? VEL_BOOST_MAX : ACCEL_BOOST_MAX;

                    qBoostX = updateBoost(qBoostX, effectiveMagX, ANGULAR_ACCEL_THRESHOLD, maxBoostX);
                    qBoostY = updateBoost(qBoostY, effectiveMagY, ANGULAR_ACCEL_THRESHOLD, maxBoostY);

                    // yaw: 仅角加速度 (旋转冲击)，角速度与坡度无关
                    double yawAccel = Math.abs((yawRate - lastYawRate) / safeDt);
                    qBoostTheta = updateBoost(qBoostTheta, yawAccel, JERK_THRESHOLD);

                } else {
                    // ========== D2: 仅角加速度 (冲击检测) ==========

                    qBoostX = updateBoost(qBoostX, Math.abs(fieldX), ANGULAR_ACCEL_THRESHOLD);
                    qBoostY = updateBoost(qBoostY, Math.abs(fieldY), ANGULAR_ACCEL_THRESHOLD);

                    // yaw 角速度 jerk
                    double jerk = Math.abs((yawRate - lastYawRate) / safeDt);
                    qBoostTheta = updateBoost(qBoostTheta, jerk, JERK_THRESHOLD);
                }

                lastPitchRate = pitchRate;
                lastRollRate  = rollRate;
                lastYawRate   = yawRate;
            }
        }

        // 构建 3x3 对角 Q 矩阵
        SimpleMatrix Q = new SimpleMatrix(3, 3);
        Q.set(0, 0, qBase * qBoostX);
        Q.set(1, 1, qBase * qBoostY);
        Q.set(2, 2, qBase * qBoostTheta);
        return Q;
    }

    // ==================== 自适应 R (SimpleMatrix 输出) ====================

    /**
     * 基于 MT1 各方向 stdDev 构建 3x3 对角 R 矩阵。
     *
     * <p>每个方向独立映射 (std → R):
     * <pre>
     *   std &lt; STD_LOW (0.05)   →  R = 0.01
     *   std &gt; STD_HIGH (0.15)  →  R = 0.01 × R_MAX_SCALE
     *   中间 → 线性插值
     * </pre>
     *
     * @return 3x3 对角观测噪声协方差矩阵 R
     */
    private SimpleMatrix adaptR() {
        double[] stdDevs = mt1.getStdDevs();  // {x, y, z, roll, pitch, yaw} (米, 度)

        // 位置: stdDev 从米 → 英寸，与 UKF 状态单位一致
        double rX = mapStdToR(stdDevs[0] * M_TO_INCH);
        double rY = mapStdToR(stdDevs[1] * M_TO_INCH);
        // 角度: yaw 从度 → 弧度，使用角度专用阈值
        double rTheta = mapStdToRAngle(Math.toRadians(stdDevs[5]));

        // 缓存 rBase 供调试展示
        rBase = rTheta;

        SimpleMatrix R = new SimpleMatrix(3, 3);
        R.set(0, 0, rX);
        R.set(1, 1, rY);
        R.set(2, 2, rTheta);
        return R;
    }

    // ==================== 工具函数 ====================

    /**
     * 单个方向的 Q 倍增因子更新 (带自定义 maxBoost)。
     *
     * @param current   当前 boost 值
     * @param magnitude 信号幅值
     * @param threshold 触发阈值
     * @param maxBoost  最大 boost 上限
     * @return 更新后的 boost 值
     */
    private double updateBoost(double current, double magnitude, double threshold, double maxBoost) {
        if (magnitude > threshold) {
            return Math.min(maxBoost, current * (1.0 + magnitude / threshold));
        } else {
            return Math.max(1.0, current * Q_DECAY);
        }
    }

    /**
     * 单个方向的 Q 倍增因子更新 (D2 兼容，使用默认 Q_BOOST_MAX)。
     */
    private double updateBoost(double current, double magnitude, double threshold) {
        return updateBoost(current, magnitude, threshold, Q_BOOST_MAX);
    }

    /**
     * 单方向 std → R 映射 (三段式: 信任 / 线性插值 / 怀疑)。
     * 使用位置阈值 (STD_LOW_INCH / STD_HIGH_INCH)。
     *
     * @param std 标准偏差 (英寸)
     */
    private double mapStdToR(double std) {
        return mapStdToR(std, STD_LOW_INCH, STD_HIGH_INCH);
    }

    /**
     * 角度方向 std → R 映射，使用角度专用阈值。
     *
     * @param std 角度标准偏差 (弧度)
     */
    private double mapStdToRAngle(double std) {
        return mapStdToR(std, STD_LOW_ANGLE, STD_HIGH_ANGLE);
    }

    /**
     * 通用 std → R 映射 (三段式: 信任 / 线性插值 / 怀疑)。
     */
    private double mapStdToR(double std, double low, double high) {
        if (std <= low) {
            return 0.01;
        } else if (std >= high) {
            return 0.01 * R_MAX_SCALE;
        } else {
            double t = (std - low) / (high - low);
            return 0.01 * (1.0 + t * (R_MAX_SCALE - 1.0));
        }
    }

    // ==================== Localizer 接口 ====================

    /** 设置定位器位姿。 */
    @Override
    public void setPose(Pose2d pose) {
        ukf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        odom.setPose(pose);
        lastTimestamp = getNow();
        qBoostX = 1.0;
        qBoostY = 1.0;
        qBoostTheta = 1.0;
        lastPitchRate = 0;
        lastRollRate = 0;
        lastYawRate = 0;
    }

    // ==================== 输出 ====================

    /** @return 融合后的位姿 {x, y, heading} (英寸, 英寸, 弧度) */
    @Override
    public Pose2d getPose() {
        double[] pose = ukf.getPose();
        return new Pose2d(pose[0], pose[1], pose[2]);
    }

    /** @return 原始 UKF 实例 */
    public UKF getUKF() { return ukf; }

    /** @return MT1 视觉定位器 */
    public MT1Localizer getMT1() { return mt1; }

    /** @return 里程计定位器 (D2: PinpointLocalizer, D3: PinpointD3Localizer) */
    public Localizer getOdom() { return odom; }

    /** @return 是否为 D3 模式 */
    public boolean isD3() { return useD3; }

    /** @return x 方向 Q 倍增因子 */
    public double getQBoostX() { return qBoostX; }
    /** @return y 方向 Q 倍增因子 */
    public double getQBoostY() { return qBoostY; }
    /** @return θ 方向 Q 倍增因子 */
    public double getQBoostTheta() { return qBoostTheta; }

    // ==================== 重置 ====================

    /** 重置定位到指定位姿。 */
    public void reset(Pose2d pose) {
        ukf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        odom.setPose(pose);
        lastTimestamp = getNow();
        qBoostX = 1.0;
        qBoostY = 1.0;
        qBoostTheta = 1.0;
        lastPitchRate = 0;
        lastRollRate = 0;
        lastYawRate = 0;
    }

    // ==================== 内部工具 ====================

    private double getNow() {
        return System.nanoTime() / 1e9;
    }
}