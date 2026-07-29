package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.processors.FusionLocalizer.AdaptiveEKFLocalizer;
import org.firstinspires.ftc.teamcode.processors.FusionLocalizer.EKFLocalizer;
import org.firstinspires.ftc.teamcode.processors.VisionLocalizer.MT1Localizer;

import java.util.LinkedList;

/**
 * 融合定位测试 OpMode —— 同时绘制 Pinpoint / MT1 / EKFLocalizer / AdaptiveEKFLocalizer 轨迹。
 *
 * <p>使用手柄操控机器人，在 FTC Dashboard 上实时绘制四条定位轨迹。
 * 左摇杆控制平移，右摇杆 X 轴控制旋转。
 */
@TeleOp(name = "Fusion Localizer Test", group = "Fusion")
public class FusionTestOpMode extends LinearOpMode {

    private static final int MAX_HISTORY = 200;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- 初始化 Limelight ----
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        // ---- 构建四个定位器 ----
        Pose2d initialPose = new Pose2d(0, 0, 0);

        // 1. Pinpoint 原始定位
        PinpointLocalizer pinpointLocalizer = new PinpointLocalizer(hardwareMap, 0.001999, initialPose);

        // 2. MT1 视觉定位
        MT1Localizer mt1Localizer = new MT1Localizer(limelight);

        // 3. EKF 融合定位 (非自适应)
        EKFLocalizer ekfLocalizer = new EKFLocalizer(hardwareMap, limelight, initialPose);

        // 4. Adaptive EKF 融合定位
        AdaptiveEKFLocalizer adaptiveEKF = new AdaptiveEKFLocalizer(hardwareMap, limelight, "imu", initialPose);

        // ---- 构建 MecanumDrive (使用 Pinpoint 作为底层定位器) ----
        MecanumDrive drive = new MecanumDrive(hardwareMap, pinpointLocalizer);

        // ---- 轨迹历史 ----
        LinkedList<Pose2d> pinpointHistory = new LinkedList<>();
        LinkedList<Pose2d> mt1History = new LinkedList<>();
        LinkedList<Pose2d> ekfHistory = new LinkedList<>();
        LinkedList<Pose2d> adaptiveEKFHistory = new LinkedList<>();

        waitForStart();

        while (opModeIsActive()) {
            // ---- 手柄控制 ----
            double forward = -gamepad1.left_stick_y;
            double strafe   = -gamepad1.left_stick_x;
            double turn     = -gamepad1.right_stick_x;

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(forward, strafe),
                    turn
            ));

            // ---- 更新所有定位器 ----
            pinpointLocalizer.update();
            mt1Localizer.update();
            ekfLocalizer.update();
            adaptiveEKF.update();

            // ---- 记录轨迹 ----
            addToHistory(pinpointHistory, pinpointLocalizer.getPose());
            addToHistory(mt1History, mt1Localizer.getPose());
            addToHistory(ekfHistory, ekfLocalizer.getPose());
            addToHistory(adaptiveEKFHistory, adaptiveEKF.getPose());

            // ---- Telemetry 输出 ----
            Pose2d pinpointPose = pinpointLocalizer.getPose();
            Pose2d ekfPose = ekfLocalizer.getPose();
            Pose2d adaptivePose = adaptiveEKF.getPose();

            telemetry.addData("Pinpoint", "x=%.2f y=%.2f h=%.1f",
                    pinpointPose.position.x, pinpointPose.position.y,
                    Math.toDegrees(pinpointPose.heading.toDouble()));
            telemetry.addData("MT1 valid", mt1Localizer.isValid());
            telemetry.addData("EKF", "x=%.2f y=%.2f h=%.1f",
                    ekfPose.position.x, ekfPose.position.y,
                    Math.toDegrees(ekfPose.heading.toDouble()));
            telemetry.addData("AdaptiveEKF", "x=%.2f y=%.2f h=%.1f",
                    adaptivePose.position.x, adaptivePose.position.y,
                    Math.toDegrees(adaptivePose.heading.toDouble()));
            telemetry.addData("Q Boost", "x=%.1f y=%.1f θ=%.1f",
                    adaptiveEKF.getQBoostX(), adaptiveEKF.getQBoostY(), adaptiveEKF.getQBoostTheta());
            telemetry.update();

            // ---- Dashboard 绘制 ----
            TelemetryPacket packet = new TelemetryPacket();

            // 绘制四条轨迹 (不同颜色)
            drawTrajectory(packet, pinpointHistory, "#FF0000");   // 红 — Pinpoint
            drawTrajectory(packet, mt1History, "#00FF00");       // 绿 — MT1
            drawTrajectory(packet, ekfHistory, "#0000FF");       // 蓝 — EKF
            drawTrajectory(packet, adaptiveEKFHistory, "#FF00FF"); // 紫 — AdaptiveEKF

            // 绘制当前机器人位姿 (使用 AdaptiveEKF 位姿)
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), adaptiveEKF.getPose());

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        limelight.stop();
    }

    // ==================== 工具 ====================

    private void addToHistory(LinkedList<Pose2d> history, Pose2d pose) {
        history.add(pose);
        while (history.size() > MAX_HISTORY) {
            history.removeFirst();
        }
    }

    private void drawTrajectory(TelemetryPacket packet, LinkedList<Pose2d> history, String color) {
        if (history.size() < 2) return;

        double[] xPoints = new double[history.size()];
        double[] yPoints = new double[history.size()];
        int i = 0;
        for (Pose2d p : history) {
            xPoints[i] = p.position.x;
            yPoints[i] = p.position.y;
            i++;
        }

        packet.fieldOverlay().setStroke(color);
        packet.fieldOverlay().setStrokeWidth(1);
        packet.fieldOverlay().strokePolyline(xPoints, yPoints);
    }
}