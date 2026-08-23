package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.processors.D3Localizer.PinpointD3Localizer;
import org.firstinspires.ftc.teamcode.processors.FusionLocalizer.AdaptiveEKF5DLocalizer;
import org.firstinspires.ftc.teamcode.processors.FusionLocalizer.AdaptiveEKFLocalizer;
import org.firstinspires.ftc.teamcode.processors.FusionLocalizer.AdaptiveUKF5DLocalizer;
import org.firstinspires.ftc.teamcode.processors.FusionLocalizer.AdaptiveUKFLocalizer;
import org.firstinspires.ftc.teamcode.processors.VisionLocalizer.MT1Localizer;

import java.util.LinkedList;
import java.util.List;

/**
 * D3 模式多定位器对比测试 OpMode。
 *
 * <p>使用手柄操控机器人，在 FTC Dashboard 上同时绘制以下 D3 / 5D 定位器的轨迹和位姿：
 * <ul>
 *   <li>{@link PinpointD3Localizer} — 3D 斜坡补偿里程计 (基准)</li>
 *   <li>{@link MT1Localizer} — 视觉绝对位置 (基准)</li>
 *   <li>{@link AdaptiveEKFLocalizer} (D3 模式: {@code useD3=true})</li>
 *   <li>{@link AdaptiveUKFLocalizer} (D3 模式: {@code useD3=true})</li>
 *   <li>{@link AdaptiveEKF5DLocalizer} — 5D 加速度计驱动 EKF</li>
 *   <li>{@link AdaptiveUKF5DLocalizer} — 5D 加速度计驱动 UKF</li>
 * </ul>
 *
 * <h3>误差测量</h3>
 * <ol>
 *   <li>在 FTC Dashboard 中设置 {@code testX / testY / testHeading} 为目标位姿</li>
 *   <li>驾驶机器人到达 TestPose</li>
 *   <li>按下手柄 A 键 → 记录所有定位器的位姿并计算误差</li>
 *   <li>误差以 (Δx, Δy, Δθ°) 形式显示在 telemetry 中</li>
 * </ol>
 *
 * <p>颜色对应:
 * <ul>
 *   <li>绿色 — PinpointD3Localizer</li>
 *   <li>橙色 — MT1Localizer (视觉)</li>
 *   <li>紫色 — AdaptiveEKFLocalizer (D3)</li>
 *   <li>蓝色 — AdaptiveUKFLocalizer (D3)</li>
 *   <li>粉色 — AdaptiveEKF5DLocalizer</li>
 *   <li>青色 — AdaptiveUKF5DLocalizer</li>
 *   <li>红色虚线 — TestPose (目标点)</li>
 * </ul>
 */
@Config
@TeleOp(name = "Fusion D3 Test", group = "Fusion")
public class FusionD3TestOpmode extends LinearOpMode {

    private static final double IN_PER_TICK = 0.001999;

    // ---- TestPose (可在 Dashboard 动态调整) ----
    public static double testX = 63;
    public static double testY = 60.7;
    public static double testHeading = Math.PI / 2;  // 弧度

    // ---- 误差记录 ----
    private boolean prevAPressed = false;
    private boolean errorRecorded = false;

    private double errPinpointX, errPinpointY, errPinpointTheta;
    private double errMt1X, errMt1Y, errMt1Theta;
    private double errAekfX, errAekfY, errAekfTheta;
    private double errAukfX, errAukfY, errAukfTheta;
    private double errE5dX, errE5dY, errE5dTheta;
    private double errU5dX, errU5dY, errU5dTheta;

    // ---- 轨迹历史 (用于绘制) ----
    private final List<Pose2d> pinpointHistory   = new LinkedList<>();
    private final List<Pose2d> mt1History        = new LinkedList<>();
    private final List<Pose2d> aekfHistory       = new LinkedList<>();
    private final List<Pose2d> aukfHistory       = new LinkedList<>();
    private final List<Pose2d> ekf5dHistory      = new LinkedList<>();
    private final List<Pose2d> ukf5dHistory      = new LinkedList<>();
    private static final int MAX_HISTORY = 80;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- 初始化 Limelight ----
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        Pose2d initialPose = new Pose2d(0, 0, 0);

        // ---- 创建所有定位器 ----
        PinpointD3Localizer pinpoint = new PinpointD3Localizer(hardwareMap, IN_PER_TICK, "imu", initialPose);
        MT1Localizer mt1 = new MT1Localizer(limelight);
        AdaptiveEKFLocalizer adaptiveEkf = new AdaptiveEKFLocalizer(hardwareMap, limelight, "imu", initialPose, true);
        AdaptiveUKFLocalizer adaptiveUkf = new AdaptiveUKFLocalizer(hardwareMap, limelight, "imu", initialPose, true);
        AdaptiveEKF5DLocalizer ekf5d = new AdaptiveEKF5DLocalizer(hardwareMap, limelight, "imu", "accel", initialPose);
        AdaptiveUKF5DLocalizer ukf5d = new AdaptiveUKF5DLocalizer(hardwareMap, limelight, "imu", "accel", initialPose);

        // ---- 创建驱动 ----
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        // ---- 主循环 ----
        while (opModeIsActive()) {
            // === 手柄控制 ===
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            // === 更新所有定位器 ===
            drive.updatePoseEstimate();
            pinpoint.update();
            mt1.update();
            adaptiveEkf.update();
            adaptiveUkf.update();
            ekf5d.update();
            ukf5d.update();

            // === 获取位姿 ===
            Pose2d pinpointPose   = pinpoint.getPose();
            Pose2d mt1Pose        = mt1.getPose();
            Pose2d adaptiveEkfPose = adaptiveEkf.getPose();
            Pose2d adaptiveUkfPose = adaptiveUkf.getPose();
            Pose2d ekf5dPose      = ekf5d.getPose();
            Pose2d ukf5dPose      = ukf5d.getPose();

            // === 记录轨迹历史 ===
            addToHistory(pinpointHistory, pinpointPose);
            addToHistory(mt1History, mt1Pose);
            addToHistory(aekfHistory, adaptiveEkfPose);
            addToHistory(aukfHistory, adaptiveUkfPose);
            addToHistory(ekf5dHistory, ekf5dPose);
            addToHistory(ukf5dHistory, ukf5dPose);

            // === A 键触发误差记录 ===
            boolean aPressed = gamepad1.a;
            if (aPressed && !prevAPressed) {
                recordErrors(pinpointPose, mt1Pose, adaptiveEkfPose, adaptiveUkfPose, ekf5dPose, ukf5dPose);
            }
            prevAPressed = aPressed;

            // ============ Telemetry: 位姿 ============
            telemetry.addData("PinpointD3",  formatPose(pinpointPose));
            telemetry.addData("MT1",         formatPose(mt1Pose));
            telemetry.addData("AdaptiveEKF(D3)", formatPose(adaptiveEkfPose));
            telemetry.addData("AdaptiveUKF(D3)", formatPose(adaptiveUkfPose));
            telemetry.addData("EKF5D",       formatPose(ekf5dPose));
            telemetry.addData("UKF5D",       formatPose(ukf5dPose));

            // ============ Telemetry: TestPose ============
            Pose2d testPose = new Pose2d(testX, testY, testHeading);
            telemetry.addData("TestPose", formatPose(testPose));
            telemetry.addData("Press A to record error", "");

            // ============ Telemetry: 已记录误差 ============
            if (errorRecorded) {
                telemetry.addLine("=== Errors (Δx, Δy, Δθ°) ===");
                telemetry.addData("PinpointD3 err",
                        formatErr(errPinpointX, errPinpointY, errPinpointTheta));
                telemetry.addData("MT1 err",
                        formatErr(errMt1X, errMt1Y, errMt1Theta));
                telemetry.addData("AdaptiveEKF(D3) err",
                        formatErr(errAekfX, errAekfY, errAekfTheta));
                telemetry.addData("AdaptiveUKF(D3) err",
                        formatErr(errAukfX, errAukfY, errAukfTheta));
                telemetry.addData("EKF5D err",
                        formatErr(errE5dX, errE5dY, errE5dTheta));
                telemetry.addData("UKF5D err",
                        formatErr(errU5dX, errU5dY, errU5dTheta));
            }

            // ============ Telemetry: AdaptiveEKFLocalizer (D3) 可调参数 ============
            telemetry.addLine("--- AdaptiveEKFLocalizer (D3) ---");
            telemetry.addData("AEKF.qBase",           AdaptiveEKFLocalizer.qBase);
            telemetry.addData("AEKF.qBoostX",         AdaptiveEKFLocalizer.qBoostX);
            telemetry.addData("AEKF.qBoostY",         AdaptiveEKFLocalizer.qBoostY);
            telemetry.addData("AEKF.qBoostTheta",     AdaptiveEKFLocalizer.qBoostTheta);
            telemetry.addData("AEKF.angVelThresh",    AdaptiveEKFLocalizer.ANGULAR_VEL_THRESHOLD);
            telemetry.addData("AEKF.velBoostMax",     AdaptiveEKFLocalizer.VEL_BOOST_MAX);
            telemetry.addData("AEKF.jerkThresh",      AdaptiveEKFLocalizer.JERK_THRESHOLD);
            telemetry.addData("AEKF.qDecay",          AdaptiveEKFLocalizer.Q_DECAY);
            telemetry.addData("AEKF.stdLowInch",      AdaptiveEKFLocalizer.STD_LOW_INCH);
            telemetry.addData("AEKF.stdHighInch",     AdaptiveEKFLocalizer.STD_HIGH_INCH);
            telemetry.addData("AEKF.stdLowAngle",     AdaptiveEKFLocalizer.STD_LOW_ANGLE);
            telemetry.addData("AEKF.stdHighAngle",    AdaptiveEKFLocalizer.STD_HIGH_ANGLE);
            telemetry.addData("AEKF.rMaxScale",       AdaptiveEKFLocalizer.R_MAX_SCALE);

            // ============ Telemetry: AdaptiveUKFLocalizer (D3) 可调参数 ============
            telemetry.addLine("--- AdaptiveUKFLocalizer (D3) ---");
            telemetry.addData("AUKF.qBase",           AdaptiveUKFLocalizer.qBase);
            telemetry.addData("AUKF.qBoostX",         AdaptiveUKFLocalizer.qBoostX);
            telemetry.addData("AUKF.qBoostY",         AdaptiveUKFLocalizer.qBoostY);
            telemetry.addData("AUKF.qBoostTheta",     AdaptiveUKFLocalizer.qBoostTheta);
            telemetry.addData("AUKF.angVelThresh",    AdaptiveUKFLocalizer.ANGULAR_VEL_THRESHOLD);
            telemetry.addData("AUKF.velBoostMax",     AdaptiveUKFLocalizer.VEL_BOOST_MAX);
            telemetry.addData("AUKF.jerkThresh",      AdaptiveUKFLocalizer.JERK_THRESHOLD);
            telemetry.addData("AUKF.qDecay",          AdaptiveUKFLocalizer.Q_DECAY);
            telemetry.addData("AUKF.stdLowInch",      AdaptiveUKFLocalizer.STD_LOW_INCH);
            telemetry.addData("AUKF.stdHighInch",     AdaptiveUKFLocalizer.STD_HIGH_INCH);
            telemetry.addData("AUKF.stdLowAngle",     AdaptiveUKFLocalizer.STD_LOW_ANGLE);
            telemetry.addData("AUKF.stdHighAngle",    AdaptiveUKFLocalizer.STD_HIGH_ANGLE);
            telemetry.addData("AUKF.rMaxScale",       AdaptiveUKFLocalizer.R_MAX_SCALE);

            // ============ Telemetry: AdaptiveEKF5DLocalizer 可调参数 ============
            telemetry.addLine("--- AdaptiveEKF5DLocalizer ---");
            telemetry.addData("E5D.accelReady",       ekf5d.isAccelerometerReady());
            telemetry.addData("E5D.rPinBase",         AdaptiveEKF5DLocalizer.R_PIN_BASE);
            telemetry.addData("E5D.rPinThetaBase",    AdaptiveEKF5DLocalizer.R_PIN_THETA_BASE);
            telemetry.addData("E5D.azThresh",         AdaptiveEKF5DLocalizer.AZ_THRESHOLD);
            telemetry.addData("E5D.angVelThresh",     AdaptiveEKF5DLocalizer.ANGULAR_VEL_THRESHOLD);
            telemetry.addData("E5D.jerkThresh",       AdaptiveEKF5DLocalizer.JERK_THRESHOLD);
            telemetry.addData("E5D.rBoostMax",        AdaptiveEKF5DLocalizer.R_BOOST_MAX);
            telemetry.addData("E5D.rDecay",           AdaptiveEKF5DLocalizer.R_DECAY);
            telemetry.addData("E5D.rBoostX",          ekf5d.getRBoostX());
            telemetry.addData("E5D.rBoostY",          ekf5d.getRBoostY());
            telemetry.addData("E5D.rBoostTheta",      ekf5d.getRBoostTheta());
            telemetry.addData("E5D.stdLowInch",       AdaptiveEKF5DLocalizer.STD_LOW_INCH);
            telemetry.addData("E5D.stdHighInch",      AdaptiveEKF5DLocalizer.STD_HIGH_INCH);
            telemetry.addData("E5D.stdLowAngle",      AdaptiveEKF5DLocalizer.STD_LOW_ANGLE);
            telemetry.addData("E5D.stdHighAngle",     AdaptiveEKF5DLocalizer.STD_HIGH_ANGLE);
            telemetry.addData("E5D.rMaxScale",        AdaptiveEKF5DLocalizer.R_MAX_SCALE);
            double[] e5dVel = ekf5d.getVelocity();
            telemetry.addData("E5D.velocity (in/s)",
                    String.format("(%.2f, %.2f)", e5dVel[0], e5dVel[1]));

            // ============ Telemetry: AdaptiveUKF5DLocalizer 可调参数 ============
            telemetry.addLine("--- AdaptiveUKF5DLocalizer ---");
            telemetry.addData("U5D.accelReady",       ukf5d.isAccelerometerReady());
            telemetry.addData("U5D.rPinBase",         AdaptiveUKF5DLocalizer.R_PIN_BASE);
            telemetry.addData("U5D.rPinThetaBase",    AdaptiveUKF5DLocalizer.R_PIN_THETA_BASE);
            telemetry.addData("U5D.azThresh",         AdaptiveUKF5DLocalizer.AZ_THRESHOLD);
            telemetry.addData("U5D.angVelThresh",     AdaptiveUKF5DLocalizer.ANGULAR_VEL_THRESHOLD);
            telemetry.addData("U5D.jerkThresh",       AdaptiveUKF5DLocalizer.JERK_THRESHOLD);
            telemetry.addData("U5D.rBoostMax",        AdaptiveUKF5DLocalizer.R_BOOST_MAX);
            telemetry.addData("U5D.rDecay",           AdaptiveUKF5DLocalizer.R_DECAY);
            telemetry.addData("U5D.rBoostX",          ukf5d.getRBoostX());
            telemetry.addData("U5D.rBoostY",          ukf5d.getRBoostY());
            telemetry.addData("U5D.rBoostTheta",      ukf5d.getRBoostTheta());
            telemetry.addData("U5D.stdLowInch",       AdaptiveUKF5DLocalizer.STD_LOW_INCH);
            telemetry.addData("U5D.stdHighInch",      AdaptiveUKF5DLocalizer.STD_HIGH_INCH);
            telemetry.addData("U5D.stdLowAngle",      AdaptiveUKF5DLocalizer.STD_LOW_ANGLE);
            telemetry.addData("U5D.stdHighAngle",     AdaptiveUKF5DLocalizer.STD_HIGH_ANGLE);
            telemetry.addData("U5D.rMaxScale",        AdaptiveUKF5DLocalizer.R_MAX_SCALE);
            double[] u5dVel = ukf5d.getVelocity();
            telemetry.addData("U5D.velocity (in/s)",
                    String.format("(%.2f, %.2f)", u5dVel[0], u5dVel[1]));

            // ============ Telemetry: MT1Localizer 实时质量指标 ============
            telemetry.addLine("--- MT1Localizer ---");
            telemetry.addData("MT1.valid",               mt1.isValid());
            telemetry.addData("MT1.tagCount",            mt1.getTagCount());
            telemetry.addData("MT1.avgDist (m)",         mt1.getAvgDist());
            telemetry.addData("MT1.avgArea",             mt1.getAvgArea());
            telemetry.addData("MT1.span (m)",            mt1.getSpan());
            telemetry.addData("MT1.maxFiducialSkew",     mt1.getMaxFiducialSkew());
            telemetry.addData("MT1.ambiguity (m)",       mt1.getAmbiguity());
            telemetry.addData("MT1.angularAmb (rad)",    mt1.getAngularAmbiguity());
            telemetry.addData("MT1.captureLatency (s)",  mt1.getCaptureLatency());
            telemetry.addData("MT1.timestamp (s)",       mt1.getTimestamp());

            double[] std = mt1.getStdDevs();
            if (std != null && std.length >= 6) {
                telemetry.addData("MT1.stdX (m)",         std[0]);
                telemetry.addData("MT1.stdY (m)",         std[1]);
                telemetry.addData("MT1.stdZ (m)",         std[2]);
                telemetry.addData("MT1.stdRoll (deg)",    std[3]);
                telemetry.addData("MT1.stdPitch (deg)",   std[4]);
                telemetry.addData("MT1.stdYaw (deg)",     std[5]);
            }

            telemetry.update();

            // === Dashboard 图形绘制 ===
            TelemetryPacket packet = new TelemetryPacket();

            // TestPose — 红色
            packet.fieldOverlay().setStroke("#F44336");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), testPose);

            // 轨迹绘制
            drawTrail(packet.fieldOverlay(), pinpointHistory, "#4CAF50", 2);
            drawTrail(packet.fieldOverlay(), mt1History,      "#FF9800", 2);
            drawTrail(packet.fieldOverlay(), aekfHistory,     "#9C27B0", 2);
            drawTrail(packet.fieldOverlay(), aukfHistory,     "#2196F3", 2);
            drawTrail(packet.fieldOverlay(), ekf5dHistory,    "#E91E63", 2);
            drawTrail(packet.fieldOverlay(), ukf5dHistory,    "#00BCD4", 2);

            // 当前位姿绘制
            packet.fieldOverlay().setStroke("#4CAF50");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), pinpointPose);

            packet.fieldOverlay().setStroke("#FF9800");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), mt1Pose);

            packet.fieldOverlay().setStroke("#9C27B0");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), adaptiveEkfPose);

            packet.fieldOverlay().setStroke("#2196F3");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), adaptiveUkfPose);

            packet.fieldOverlay().setStroke("#E91E63");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), ekf5dPose);

            packet.fieldOverlay().setStroke("#00BCD4");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), ukf5dPose);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    // ==================== 误差计算 ====================

    private void recordErrors(Pose2d pp, Pose2d mt, Pose2d ae, Pose2d au, Pose2d e5, Pose2d u5) {
        Pose2d ref = new Pose2d(testX, testY, testHeading);

        errPinpointX     = pp.position.x - ref.position.x;
        errPinpointY     = pp.position.y - ref.position.y;
        errPinpointTheta = normalizeAngle(pp.heading.toDouble() - ref.heading.toDouble());

        errMt1X          = mt.position.x - ref.position.x;
        errMt1Y          = mt.position.y - ref.position.y;
        errMt1Theta      = normalizeAngle(mt.heading.toDouble() - ref.heading.toDouble());

        errAekfX         = ae.position.x - ref.position.x;
        errAekfY         = ae.position.y - ref.position.y;
        errAekfTheta     = normalizeAngle(ae.heading.toDouble() - ref.heading.toDouble());

        errAukfX         = au.position.x - ref.position.x;
        errAukfY         = au.position.y - ref.position.y;
        errAukfTheta     = normalizeAngle(au.heading.toDouble() - ref.heading.toDouble());

        errE5dX          = e5.position.x - ref.position.x;
        errE5dY          = e5.position.y - ref.position.y;
        errE5dTheta      = normalizeAngle(e5.heading.toDouble() - ref.heading.toDouble());

        errU5dX          = u5.position.x - ref.position.x;
        errU5dY          = u5.position.y - ref.position.y;
        errU5dTheta      = normalizeAngle(u5.heading.toDouble() - ref.heading.toDouble());

        errorRecorded = true;
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // ==================== 工具 ====================

    private static void addToHistory(List<Pose2d> history, Pose2d pose) {
        history.add(pose);
        if (history.size() > MAX_HISTORY) {
            history.remove(0);
        }
    }

    private static void drawTrail(com.acmerobotics.dashboard.canvas.Canvas c,
                                  List<Pose2d> history, String color, int width) {
        if (history.size() < 2) return;
        c.setStroke(color);
        c.setStrokeWidth(width);
        c.setFill(color);
        double[] xs = new double[history.size()];
        double[] ys = new double[history.size()];
        for (int i = 0; i < history.size(); i++) {
            xs[i] = history.get(i).position.x;
            ys[i] = history.get(i).position.y;
        }
        c.strokePolyline(xs, ys);
    }

    private static String formatPose(Pose2d pose) {
        return String.format("(%.2f, %.2f, %.1f°)",
                pose.position.x, pose.position.y,
                Math.toDegrees(pose.heading.toDouble()));
    }

    private static String formatErr(double dx, double dy, double dtheta) {
        return String.format("(%+.2f, %+.2f, %+.1f°)",
                dx, dy, Math.toDegrees(dtheta));
    }
}