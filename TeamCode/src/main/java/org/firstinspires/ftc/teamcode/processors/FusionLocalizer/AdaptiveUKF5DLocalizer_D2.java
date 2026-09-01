package org.firstinspires.ftc.teamcode.processors.FusionLocalizer;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.processors.Accelerometer.Rev9axisIMU;
import org.firstinspires.ftc.teamcode.processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.utility.filter.UKF.UKF5D;

/**
 * D2 简化版 5 维自适应融合定位器 —— 外接加速度计 + Pinpoint + Limelight + UKF5D。
 *
 * <p>相对 {@link AdaptiveUKF5DLocalizer}（D3 模式，3D 斜坡补偿里程计）的简化版本，固定采用
 * <b>标准 2D 里程计</b>（{@link PinpointLocalizer}），核心差异与数据角色如下：
 * <ol>
 *   <li><b>外接加速度计</b>（{@link Rev9axisIMU}, BNO055）测出体坐标系线性加速度，结合 Hub IMU
 *       pitch/roll 姿态角<b>旋转后作为控制输入</b>驱动 {@link UKF5D} 预测；<b>过程噪声 Q 保持固定</b>
 *       （沿用 {@link UKF5D} 默认 Q，不做自适应）；</li>
 *   <li><b>Pinpoint</b> 提供体坐标系水平速度与航向（观测），观测噪声 {@code R_pin} 自适应，
 *       与 {@link AdaptiveUKF5DLocalizer} 相比<b>仅用角度判定</b>：
 *       <ul>
 *         <li>X（前后速度）由 <b>pitch 倾角</b>大小判定；</li>
 *         <li>Y（左右速度）由 <b>roll 倾角</b>大小判定；</li>
 *         <li>θ（航向）仍由 <b>yaw 角加速度</b>（旋转冲击）判定，与 {@link AdaptiveUKF5DLocalizer} 一致。</li>
 *       </ul></li>
 *   <li><b>Limelight</b>（{@link MT1Localizer}）提供低频绝对位置观测，观测噪声 {@code R_vision}
 *       自适应逻辑与 {@link AdaptiveUKF5DLocalizer} 完全一致（MT1 stdDev → R 映射）。</li>
 * </ol>
 */
@Config
public class AdaptiveUKF5DLocalizer_D2 implements Localizer {

    private final UKF5D ukf;
    /** 里程计 (标准 2D: PinpointLocalizer) */
    private final PinpointLocalizer odom;
    private final MT1Localizer mt1;
    private final IMU hubImu;
    /** 外置加速度计 (BNO055 封装) */
    private final Rev9axisIMU accel;

    // ---- 时间基准 ----
    private double lastTimestamp = 0;

    /** 最近一次里程计速度缓存 (体坐标系水平速度 + 航向角速度) */
    private PoseVelocity2d lastVel = new PoseVelocity2d(new Vector2d(0, 0), 0);

    // ---- 自适应 R_pinpoint 状态 (实例级) ----
    private double rBoostX = 1.0;
    private double rBoostY = 1.0;
    private double rBoostTheta = 1.0;
    /** 上一帧 yaw 角速度 (rad/s)，用于计算 yaw 角加速度 */
    private double lastYawRate = 0;
    /** 是否已采样首帧 yaw 角速度 (防止首帧 (rate - 0) / dt 误触发 R 提升) */
    private boolean ratesInitialized = false;

    // ---- 可调参数 (R_pin 自适应) ----
    /** Pinpoint 速度观测噪声基值 (in²/s²) */
    public static double R_PIN_BASE = 0.1;
    /** Pinpoint 航向观测噪声基值 (rad²) */
    public static double R_PIN_THETA_BASE = 0.05;
    /** pitch/roll 倾角判定阈值 (弧度，≈8.6°)，超过则放大对应速度观测噪声 */
    public static double ANGLE_THRESHOLD = 0.15;
    /** yaw 角加速度 (旋转冲击) 判定阈值 (rad/s²) */
    public static double JERK_THRESHOLD = 4.0;
    public static double R_BOOST_MAX = 20.0;
    public static double R_DECAY = 0.85;

    // ---- 零速检测阈值 ----
    public static double ZERO_VEL_ACCEL_THRESHOLD = 0.5;     // in/s²
    public static double ZERO_VEL_PINPOINT_THRESHOLD = 0.5;  // in/s

    // ---- R 自适应: MT1 stdDev (与 AdaptiveUKF5DLocalizer 一致) ----
    public static double M_TO_INCH = 39.37007874;
    public static double STD_LOW_INCH = 2.0;
    public static double STD_HIGH_INCH = 6.0;
    public static double STD_LOW_ANGLE = 0.035;   // rad (≈2°)
    public static double STD_HIGH_ANGLE = 0.175;  // rad (≈10°)
    public static double R_MAX_SCALE = 20.0;

    // ==================== 构造 ====================

    /**
     * 完整构造。
     *
     * @param hardwareMap     硬件映射
     * @param limelight       已启动的 Limelight3A 实例
     * @param imuDeviceName   Hub IMU 设备名 (如 "imu", 提供 pitch/roll 姿态与 yaw 角速度)
     * @param accelDeviceName 外置 BNO055 加速度计设备名 (如 "accel")
     * @param initialPose     初始位姿 (x, y, heading)
     */
    public AdaptiveUKF5DLocalizer_D2(HardwareMap hardwareMap, Limelight3A limelight,
                                     String imuDeviceName, String accelDeviceName,
                                     Pose2d initialPose) {
        this.ukf = new UKF5D(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble());
        this.mt1 = new MT1Localizer(limelight);
        this.hubImu = hardwareMap.get(IMU.class, imuDeviceName);
        this.odom = new PinpointLocalizer(hardwareMap, 0.001999, initialPose);

        // 外置加速度计 (与 AdaptiveUKF5DLocalizer D3 相同安装朝向约定)
        this.accel = new Rev9axisIMU(hardwareMap, accelDeviceName);
        this.accel.setOrientationXY(Rev9axisIMU.FacingDirection.FORWARD, Rev9axisIMU.FacingDirection.UP);
        if (!this.accel.initialize()) {
            android.util.Log.w("FusionLocalizer", "外置加速度计(" + accelDeviceName + ")初始化失败, 5D 滤波将以零加速度运行");
        }

        this.lastTimestamp = getNow();
    }

    /** 简化构造：IMU 名称 "imu"、加速度计名称 "accel"、初始位姿 (0,0,0)。 */
    public AdaptiveUKF5DLocalizer_D2(HardwareMap hardwareMap, Limelight3A limelight) {
        this(hardwareMap, limelight, "imu", "accel", new Pose2d(0, 0, 0));
    }

    // ==================== 核心循环 ====================

    @Override
    public PoseVelocity2d update() {
        double now = getNow();
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        // ---- 1. 外接 BNO055 线性加速度 (机器人坐标系, 已剔除重力) ----
        double axLin = 0, ayLin = 0, azLin = 0;
        if (accel.isConnected()) {
            accel.readAccelerometer();
            axLin = accel.getXAcceleration();
            ayLin = accel.getYAcceleration();
            azLin = accel.getZAcceleration();
        }

        // ---- 2. Hub IMU 姿态 (pitch/roll, 用于旋转加速度 + 倾角判定) ----
        double pitch = 0, roll = 0;
        YawPitchRollAngles angles = hubImu.getRobotYawPitchRollAngles();
        if (angles != null) {
            pitch = angles.getPitch(AngleUnit.RADIANS);
            roll = angles.getRoll(AngleUnit.RADIANS);
        }

        // ---- 3. Pinpoint 体坐标系水平速度 + 航向角速度 ----
        lastVel = odom.update();
        double vxBody = lastVel.linearVel.x;
        double vyBody = lastVel.linearVel.y;
        double omega = lastVel.angVel;

        // ---- 4. 自适应 R_pinpoint (角度判定: pitch/roll; θ: yaw 角加速度) ----
        ukf.setOdomR(adaptOdomR(dt, pitch, roll));

        // ---- 5. UKF5D 预测 (Q 固定, 加速度旋转由 pitch/roll 完成) ----
        ukf.predict(axLin, ayLin, azLin, pitch, roll, omega, now);

        // ---- 6. Pinpoint 速度/航向观测更新 ----
        double thetaOdom = odom.getPose().heading.toDouble();
        ukf.updateOdom(vxBody, vyBody, thetaOdom, now);

        // ---- 7. 零速检测 ----
        double linAccelMag = Math.hypot(axLin, ayLin);
        double pinSpeed = Math.hypot(vxBody, vyBody);
        if (linAccelMag < ZERO_VEL_ACCEL_THRESHOLD && pinSpeed < ZERO_VEL_PINPOINT_THRESHOLD) {
            ukf.zeroVelocity();
        }

        // ---- 8. MT1 视觉 → 自适应 R (与 AdaptiveUKF5DLocalizer 一致) + 视觉更新 ----
        mt1.update();
        if (mt1.isValid()) {
            ukf.setVisionR(adaptVisionR());
            Pose2d visionPose = mt1.getPose();
            ukf.updateVision(
                    visionPose.position.x,
                    visionPose.position.y,
                    visionPose.heading.toDouble(),
                    mt1.getTimestamp()
            );
        }

        return lastVel;
    }

    // ==================== 自适应 R_pinpoint ====================

    /**
     * 基于倾角与 yaw 角加速度构建 3x3 对角 R 矩阵 (vx_body, vy_body, θ)。
     * <ul>
     *   <li>X (前后): |pitch| 倾角大小 → 前后速度观测不可靠</li>
     *   <li>Y (左右): |roll| 倾角大小 → 左右速度观测不可靠</li>
     *   <li>θ (航向): yaw 角加速度 (旋转冲击)</li>
     * </ul>
     */
    private SimpleMatrix adaptOdomR(double dt, double pitch, double roll) {
        double safeDt = Math.max(dt, 1e-6);

        // X (前后): pitch 倾角
        rBoostX = updateRBoost(rBoostX, Math.abs(pitch), ANGLE_THRESHOLD);

        // Y (左右): roll 倾角
        rBoostY = updateRBoost(rBoostY, Math.abs(roll), ANGLE_THRESHOLD);

        // θ (航向): yaw 角加速度
        double yawRate = 0;
        if (hubImu != null) {
            AngularVelocity av = hubImu.getRobotAngularVelocity(AngleUnit.RADIANS);
            if (av != null) {
                yawRate = av.zRotationRate;
            }
        }
        if (!ratesInitialized) {
            // 首帧仅采样, 避免 (yawRate - 0) / safeDt 误触发 R 提升
            lastYawRate = yawRate;
            ratesInitialized = true;
        } else {
            double yawAccel = Math.abs((yawRate - lastYawRate) / safeDt);
            rBoostTheta = updateRBoost(rBoostTheta, yawAccel, JERK_THRESHOLD);
            lastYawRate = yawRate;
        }

        SimpleMatrix R = new SimpleMatrix(3, 3);
        R.set(0, 0, R_PIN_BASE * rBoostX);
        R.set(1, 1, R_PIN_BASE * rBoostY);
        R.set(2, 2, R_PIN_THETA_BASE * rBoostTheta);
        return R;
    }

    // ==================== 自适应 R_vision (与 AdaptiveUKF5DLocalizer 一致) ====================

    /**
     * 基于 MT1 各方向 stdDev 构建 3x3 对角 R 矩阵 (x, y, θ)。
     */
    private SimpleMatrix adaptVisionR() {
        double[] stdDevs = mt1.getStdDevs();  // {x, y, z, roll, pitch, yaw} (米, 度)

        double rX = mapStdToR(stdDevs[0] * M_TO_INCH);
        double rY = mapStdToR(stdDevs[1] * M_TO_INCH);
        double rTheta = mapStdToR(Math.toRadians(stdDevs[5]), STD_LOW_ANGLE, STD_HIGH_ANGLE);

        SimpleMatrix R = new SimpleMatrix(3, 3);
        R.set(0, 0, rX);
        R.set(1, 1, rY);
        R.set(2, 2, rTheta);
        return R;
    }

    // ==================== 工具函数 ====================

    private double updateRBoost(double current, double magnitude, double threshold) {
        if (magnitude > threshold) {
            return Math.min(R_BOOST_MAX, current * (1.0 + magnitude / threshold));
        } else {
            return Math.max(1.0, current * R_DECAY);
        }
    }

    private double mapStdToR(double std) {
        return mapStdToR(std, STD_LOW_INCH, STD_HIGH_INCH);
    }

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

    @Override
    public void setPose(Pose2d pose) {
        ukf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        odom.setPose(pose);
        lastTimestamp = getNow();
        rBoostX = 1.0;
        rBoostY = 1.0;
        rBoostTheta = 1.0;
        lastYawRate = 0;
        ratesInitialized = false;
    }

    @Override
    public Pose2d getPose() {
        double[] pose = ukf.getPose();
        return new Pose2d(pose[0], pose[1], pose[2]);
    }

    // ==================== 输出 / 调试 ====================

    /** @return 原始 UKF5D 实例 */
    public UKF5D getUKF() { return ukf; }

    /** @return MT1 视觉定位器 */
    public MT1Localizer getMT1() { return mt1; }

    /** @return 里程计定位器 (PinpointLocalizer) */
    public PinpointLocalizer getOdom() { return odom; }

    /** @return 外置加速度计封装 */
    public Rev9axisIMU getAccelerometer() { return accel; }

    /** @return 外置加速度计是否在线 */
    public boolean isAccelerometerReady() { return accel.isConnected(); }

    /** @return 世界坐标系速度 {Vx, Vy} (英寸/秒) */
    public double[] getVelocity() { return ukf.getVelocity(); }

    /** @return X 方向 R_pin boost 因子 */
    public double getRBoostX() { return rBoostX; }
    /** @return Y 方向 R_pin boost 因子 */
    public double getRBoostY() { return rBoostY; }
    /** @return θ 方向 R_pin boost 因子 */
    public double getRBoostTheta() { return rBoostTheta; }

    /** 重置定位到指定位姿。 */
    public void reset(Pose2d pose) {
        setPose(pose);
    }

    // ==================== 内部工具 ====================

    private double getNow() {
        return System.nanoTime() / 1e9;
    }
}