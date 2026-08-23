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
import org.firstinspires.ftc.teamcode.processors.Accelerometer.dfRobotAccelerometer;
import org.firstinspires.ftc.teamcode.processors.D3Localizer.PinpointD3Localizer;
import org.firstinspires.ftc.teamcode.processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.utility.filter.UKF.UKF5D;

/**
 * 5 维自适应融合定位器 (D3 模式) —— 外接加速度计 + Pinpoint 速度计 + Limelight + UKF5D。
 *
 * <p>本定位器实现 {@code Theory5D.md} 所述的升级方案，与
 * {@link AdaptiveEKF5DLocalizer} 结构一致，仅滤波器替换为无迹卡尔曼（UKF5D）：
 * <ol>
 *   <li><b>外接加速度计</b> ({@link dfRobotAccelerometer}) 作为 <b>控制输入</b>（重力已剔除）；</li>
 *   <li><b>Pinpoint 里程计</b> ({@link PinpointD3Localizer}) 作为 <b>速度/航向观测</b>（体坐标系）；</li>
 *   <li><b>Limelight MegaTag1</b> ({@link MT1Localizer}) 作为 <b>低频绝对位置观测</b>；</li>
 *   <li><b>自适应 R\_pinpoint</b>：按竖直加速度与角速度动态放大 Pinpoint 速度观测噪声。</li>
 * </ol>
 *
 * <p>UKF 通过 sigma 点无迹变换传播 5 维状态，每个 sigma 点拥有独立航向，
 * 加速度旋转逐点进行（Theory5D.md §10 注 4）。
 *
 * <p>只支持 <b>D3 模式</b>。
 */
@Config
public class AdaptiveUKF5DLocalizer implements Localizer {

    private final UKF5D ukf;
    private final PinpointD3Localizer odom;
    private final MT1Localizer mt1;
    private final IMU hubImu;
    private final dfRobotAccelerometer accel;
    private final boolean accelReady;

    private double lastTimestamp = 0;
    private PoseVelocity2d lastVel = new PoseVelocity2d(new Vector2d(0, 0), 0);

    private double rBoostX = 1.0;
    private double rBoostY = 1.0;
    private double rBoostTheta = 1.0;
    private double lastYawRate = 0;

    // ---- 可调参数 (Theory5D.md §7) ----
    public static final double GRAVITY_IN_S2 = 9.80665 * 39.37007874;

    public static double R_PIN_BASE = 0.1;
    public static double R_PIN_THETA_BASE = 0.05;

    public static double AZ_THRESHOLD = 2.0;              // in/s²
    public static double ANGULAR_VEL_THRESHOLD = 1.0;     // rad/s
    public static double JERK_THRESHOLD = 4.0;            // rad/s²
    public static double R_BOOST_MAX = 20.0;
    public static double R_DECAY = 0.85;

    public static double ZERO_VEL_ACCEL_THRESHOLD = 0.5;     // in/s²
    public static double ZERO_VEL_PINPOINT_THRESHOLD = 0.5;  // in/s

    // ---- R 自适应: MT1 stdDev ----
    public static double M_TO_INCH = 39.37007874;
    public static double STD_LOW_INCH = 2.0;
    public static double STD_HIGH_INCH = 6.0;
    public static double STD_LOW_ANGLE = 0.035;
    public static double STD_HIGH_ANGLE = 0.175;
    public static double R_MAX_SCALE = 20.0;

    // ==================== 构造 ====================

    public AdaptiveUKF5DLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                                  String imuDeviceName, String accelDeviceName,
                                  Pose2d initialPose) {
        this.ukf = new UKF5D(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble());
        this.odom = new PinpointD3Localizer(hardwareMap, 0.001999, imuDeviceName, initialPose);
        this.mt1 = new MT1Localizer(limelight);
        this.hubImu = hardwareMap.get(IMU.class, imuDeviceName);

        this.accel = new dfRobotAccelerometer(hardwareMap, accelDeviceName);
        this.accel.setRange(dfRobotAccelerometer.Range.RANGE_4G);
        this.accelReady = this.accel.initialize();

        this.lastTimestamp = getNow();
    }

    /** 简化构造：默认 IMU 名称 "imu"、加速度计名称 "accel"、初始位姿 (0,0,0)。 */
    public AdaptiveUKF5DLocalizer(HardwareMap hardwareMap, Limelight3A limelight) {
        this(hardwareMap, limelight, "imu", "accel", new Pose2d(0, 0, 0));
    }

    // ==================== 核心循环 ====================

    @Override
    public PoseVelocity2d update() {
        double now = getNow();
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        // ---- 1. 外接加速度计 (体坐标系, 含重力) ----
        double axRaw = 0, ayRaw = 0, azRaw = 0;
        if (accelReady) {
            accel.readAccelerometer();
            axRaw = accel.getXAcceleration();
            ayRaw = accel.getYAcceleration();
            azRaw = accel.getZAcceleration();
        }

        // ---- 2. Hub IMU 姿态 ----
        double pitch = 0, roll = 0;
        YawPitchRollAngles angles = hubImu.getRobotYawPitchRollAngles();
        if (angles != null) {
            pitch = angles.getPitch(AngleUnit.RADIANS);
            roll = angles.getRoll(AngleUnit.RADIANS);
        }

        // ---- 3. 重力剔除 ----
        // 约定: pitch θ 绕 Robot X(右) = RR 体轴 -Y；roll φ 绕 Robot Y(前) = RR 体轴 +X。
        // 加速度计静止时读到的重力比力分量（需从测量值中减去）:
        //   g_meas = [+G·sinθ, +G·cosθ·sinφ, +G·cosθ·cosφ]
        double sinP = Math.sin(pitch), cosP = Math.cos(pitch);
        double sinR = Math.sin(roll), cosR = Math.cos(roll);
        double axLin = axRaw - GRAVITY_IN_S2 * sinP;
        double ayLin = ayRaw - GRAVITY_IN_S2 * cosP * sinR;
        double azLin = azRaw - GRAVITY_IN_S2 * cosP * cosR;

        // ---- 4. Pinpoint 体坐标系水平速度 + 航向角速度 ----
        lastVel = odom.update();
        double vxBody = lastVel.linearVel.x;
        double vyBody = lastVel.linearVel.y;
        double omega = lastVel.angVel;

        // ---- 5. 自适应 R_pinpoint ----
        ukf.setOdomR(adaptOdomR(dt, azLin));

        // ---- 6. UKF5D 预测 ----
        ukf.predict(axLin, ayLin, azLin, pitch, roll, omega, now);

        // ---- 7. Pinpoint 速度/航向观测更新 ----
        double thetaOdom = odom.getPose().heading.toDouble();
        ukf.updateOdom(vxBody, vyBody, thetaOdom, now);

        // ---- 8. 零速检测 ----
        double linAccelMag = Math.hypot(axLin, ayLin);
        double pinSpeed = Math.hypot(vxBody, vyBody);
        if (linAccelMag < ZERO_VEL_ACCEL_THRESHOLD && pinSpeed < ZERO_VEL_PINPOINT_THRESHOLD) {
            ukf.zeroVelocity();
        }

        // ---- 9. MT1 视觉更新 ----
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

    private SimpleMatrix adaptOdomR(double dt, double azLin) {
        double safeDt = Math.max(dt, 1e-6);
        double azMag = Math.abs(azLin);

        double pitchRate = 0, rollRate = 0, yawRate = 0;
        if (hubImu != null) {
            AngularVelocity av = hubImu.getRobotAngularVelocity(AngleUnit.RADIANS);
            if (av != null) {
                pitchRate = av.xRotationRate;
                rollRate = av.yRotationRate;
                yawRate = av.zRotationRate;
            }
        }

        double magX = azMag / AZ_THRESHOLD + Math.abs(pitchRate) / ANGULAR_VEL_THRESHOLD;
        rBoostX = updateRBoost(rBoostX, magX, 1.0);

        double magY = azMag / AZ_THRESHOLD + Math.abs(rollRate) / ANGULAR_VEL_THRESHOLD;
        rBoostY = updateRBoost(rBoostY, magY, 1.0);

        double yawAccel = Math.abs((yawRate - lastYawRate) / safeDt);
        rBoostTheta = updateRBoost(rBoostTheta, yawAccel, JERK_THRESHOLD);
        lastYawRate = yawRate;

        SimpleMatrix R = new SimpleMatrix(3, 3);
        R.set(0, 0, R_PIN_BASE * rBoostX);
        R.set(1, 1, R_PIN_BASE * rBoostY);
        R.set(2, 2, R_PIN_THETA_BASE * rBoostTheta);
        return R;
    }

    // ==================== 自适应 R_vision ====================

    private SimpleMatrix adaptVisionR() {
        double[] stdDevs = mt1.getStdDevs();

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
    }

    @Override
    public Pose2d getPose() {
        double[] pose = ukf.getPose();
        return new Pose2d(pose[0], pose[1], pose[2]);
    }

    // ==================== 输出 / 调试 ====================

    /** @return 原始 UKF5D 实例 */
    public UKF5D getUKF() { return ukf; }

    public MT1Localizer getMT1() { return mt1; }

    public PinpointD3Localizer getOdom() { return odom; }

    public dfRobotAccelerometer getAccelerometer() { return accel; }

    public boolean isAccelerometerReady() { return accelReady; }

    public double[] getVelocity() { return ukf.getVelocity(); }

    public double getRBoostX() { return rBoostX; }
    public double getRBoostY() { return rBoostY; }
    public double getRBoostTheta() { return rBoostTheta; }

    public void reset(Pose2d pose) {
        setPose(pose);
    }

    // ==================== 内部工具 ====================

    private double getNow() {
        return System.nanoTime() / 1e9;
    }
}