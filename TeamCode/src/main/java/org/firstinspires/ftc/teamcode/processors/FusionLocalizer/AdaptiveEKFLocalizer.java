package org.firstinspires.ftc.teamcode.processors.FusionLocalizer;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.utility.filter.EKF.EKF;

/**
 * 融合定位器 —— Pinpoint(里程计) + Limelight MegaTag1(视觉) + EKF 融合。
 *
 * <p>每帧调用 {@link #update()} 即可完成：
 * <ol>
 *   <li>Pinpoint 速度预测 ({@link EKF#predict})</li>
 *   <li>Limelight 视觉更新 (条件触发, {@link EKF#update})</li>
 *   <li>自适应调整 Q (IMU 加速度/角速度) 和 R (视觉 stdDev)</li>
 * </ol>
 *
 * <p>自适应策略:
 * <ul>
 *   <li><b>Q 自适应</b>: IMU 线性加速度 (x, y) + 角速度 jerk (θ) → 各方向独立 Q 矩阵</li>
 *   <li><b>R 自适应</b>: MT1 各方向 stdDev → 各方向独立 R 矩阵</li>
 * </ul>
 */
@Config
public class AdaptiveEKFLocalizer implements Localizer {

    private final EKF ekf;
    private final PinpointLocalizer pinpoint;
    private final MT1Localizer mt1;
    private final IMU hubImu;  // Hub IMU: 同时提供线性加速度和角速度

    // ---- 时间基准 ----
    private double lastTimestamp = 0;

    /** 最近一次 Pinpoint 速度缓存 */
    private PoseVelocity2d lastVel = new PoseVelocity2d(new Vector2d(0, 0), 0);

    // ---- Q 自适应: IMU 冲击检测 ----
    /** 上一帧 pitch 角速度 (rad/s) — 用于计算角加速度 */
    private double lastPitchRate = 0;
    /** 上一帧 roll 角速度 (rad/s) — 用于计算角加速度 */
    private double lastRollRate = 0;
    /** 上一帧 yaw 角速度 (rad/s) — 用于计算角速度 jerk */
    private double lastYawRate = 0;

    /** Q 基值 (in²/s) */
    public static double qBase = 0.002;
    public static double qBoostX = 1.0;
    public static double qBoostY = 1.0;
    public static double qBoostTheta = 1.0;

    public static double ANGULAR_ACCEL_THRESHOLD = 5.0;  // rad/s² (pitch/roll 角加速度阈值)
    public static double JERK_THRESHOLD = 4.0;           // rad/s² (yaw 角速度 jerk 阈值)
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
     * @param hardwareMap    硬件映射
     * @param limelight      已启动的 Limelight3A 实例
     * @param imuDeviceName  IMU 设备名 (配置中的名称, 如 "imu")
     * @param initialPose    初始位姿 (x, y, heading)
     */
    public AdaptiveEKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                           String imuDeviceName, Pose2d initialPose) {
        this.ekf = new EKF(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble());
        this.pinpoint = new PinpointLocalizer(hardwareMap, 0.001999, initialPose);
        this.mt1 = new MT1Localizer(limelight);
        this.hubImu = hardwareMap.get(IMU.class, imuDeviceName);
        this.lastTimestamp = getNow();
    }

    /**
     * 简化构造: 使用默认 IMU 名称 "imu", 初始位姿 (0,0,0)。
     */
    public AdaptiveEKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight) {
        this(hardwareMap, limelight, "imu", new Pose2d(0, 0, 0));
    }

    // ==================== 核心循环 ====================

    /**
     * 每帧调用一次，完成：
     * <ol>
     *   <li>Pinpoint 更新 → 获取速度</li>
     *   <li>IMU 加速度/角速度检测 → 自适应 Q (3x3 矩阵)</li>
     *   <li>EKF 预测</li>
     *   <li>Limelight 更新 → 自适应 R (3x3 矩阵) + EKF 更新</li>
     * </ol>
     *
     * @return 当前速度估计
     */
    @Override
    public PoseVelocity2d update() {
        double now = getNow();
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        // ---- 1. Pinpoint 速度 ----
        lastVel = pinpoint.update();

        // ---- 2. IMU 加速度/角速度 → 自适应 Q 矩阵 ----
        ekf.setQ(adaptQ(dt));

        // ---- 3. EKF 预测 ----
        ekf.predict(lastVel.linearVel.x, lastVel.linearVel.y, lastVel.angVel, now);

        // ---- 4. MT1 视觉 → 自适应 R 矩阵 + EKF 更新 ----
        mt1.update();
        if (mt1.isValid()) {
            ekf.setR(adaptR());
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

    // ==================== 自适应 Q (SimpleMatrix 输出) ====================

    /**
     * 基于 IMU 角速度构建 3x3 对角 Q 矩阵。
     *
     * <ul>
     *   <li><b>x, y</b>: pitch/roll 角加速度 → 旋转到绝对坐标系 → 检测碰撞/急加速</li>
     *   <li><b>θ</b>: IMU yaw 角速度 jerk → 检测旋转碰撞</li>
     * </ul>
     *
     * <p>Hub IMU 无法直接提供线性加速度，因此利用 pitch/roll 角加速度作为
     * 机器人受到冲击的间接指标，再通过当前航向角将体坐标系扰动转换到场坐标系。
     *
     * @param dt 帧间隔 (秒)
     * @return 3x3 对角过程噪声协方差矩阵 Q
     */
    private SimpleMatrix adaptQ(double dt) {
        double safeDt = Math.max(dt, 1e-6);

        // ---- x, y: pitch/roll 角加速度 → 绝对坐标系 ----
        double pitchAccel = 0, rollAccel = 0;
        if (hubImu != null) {
            AngularVelocity angVel = hubImu.getRobotAngularVelocity(AngleUnit.RADIANS);
            if (angVel != null) {
                pitchAccel = (angVel.xRotationRate - lastPitchRate) / safeDt;
                rollAccel  = (angVel.yRotationRate - lastRollRate)  / safeDt;
                lastPitchRate = angVel.xRotationRate;
                lastRollRate  = angVel.yRotationRate;
            }
        }

        // 体坐标系 → 场坐标系旋转
        // pitch 角加速度 → 机器人前向 (x_body) 扰动
        // roll  角加速度 → 机器人侧向 (y_body) 扰动
        double heading = getPose().heading.toDouble();  // 当前航向 (弧度)
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double fieldX =  pitchAccel * cosH - rollAccel * sinH;
        double fieldY =  pitchAccel * sinH + rollAccel * cosH;

        qBoostX = updateBoost(qBoostX, Math.abs(fieldX), ANGULAR_ACCEL_THRESHOLD);
        qBoostY = updateBoost(qBoostY, Math.abs(fieldY), ANGULAR_ACCEL_THRESHOLD);

        // ---- θ: Hub IMU yaw 角速度 jerk ----
        if (hubImu != null) {
            AngularVelocity angVel = hubImu.getRobotAngularVelocity(AngleUnit.RADIANS);
            if (angVel != null) {
                double jerk = Math.abs((angVel.zRotationRate - lastYawRate) / safeDt);
                lastYawRate = angVel.zRotationRate;
                qBoostTheta = updateBoost(qBoostTheta, jerk, JERK_THRESHOLD);
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

        // 位置: stdDev 从米 → 英寸，与 EKF 状态单位一致
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
     * 单个方向的 Q 倍增因子更新。
     */
    private double updateBoost(double current, double magnitude, double threshold) {
        if (magnitude > threshold) {
            return Math.min(Q_BOOST_MAX, current * (1.0 + magnitude / threshold));
        } else {
            return Math.max(1.0, current * Q_DECAY);
        }
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
        ekf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        pinpoint.setPose(pose);
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
        double[] pose = ekf.getPose();
        return new Pose2d(pose[0], pose[1], pose[2]);
    }

    /** @return 原始 EKF 实例 */
    public EKF getEKF() { return ekf; }

    /** @return MT1 视觉定位器 */
    public MT1Localizer getMT1() { return mt1; }

    /** @return Pinpoint 定位器 */
    public PinpointLocalizer getPinpoint() { return pinpoint; }

    /** @return x 方向 Q 倍增因子 */
    public double getQBoostX() { return qBoostX; }
    /** @return y 方向 Q 倍增因子 */
    public double getQBoostY() { return qBoostY; }
    /** @return θ 方向 Q 倍增因子 */
    public double getQBoostTheta() { return qBoostTheta; }

    // ==================== 重置 ====================

    /** 重置定位到指定位姿。 */
    public void reset(Pose2d pose) {
        ekf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        pinpoint.setPose(pose);
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