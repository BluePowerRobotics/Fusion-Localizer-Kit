package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.FusionLocalizer.FusionConfig;
import org.firstinspires.ftc.teamcode.controllers.FusionLocalizer.FusionLocalizer;
import org.firstinspires.ftc.teamcode.controllers.VisionLocalizer.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.controllers.VisionLocalizer.TagLayout;
import org.firstinspires.ftc.teamcode.controllers.VisionLocalizer.VisionLocalizer;

/**
 * 融合定位系统参数调优OpMode
 */
@TeleOp(name = "Tune Fusion Localizer", group = "Fusion Localizer")
public class TuningFusionOpMode extends LinearOpMode {
    private TagLayout tagLayout;
    private AprilTagPipeline aprilTagPipeline;
    private VisionLocalizer visionLocalizer;
    private FusionLocalizer fusionLocalizer;

    @Override
    public void runOpMode() {
        // TODO: 初始化标记点布局
        tagLayout = new TagLayout();

        // TODO: 初始化AprilTag检测流水线
        aprilTagPipeline = new AprilTagPipeline(hardwareMap, "Webcam 1");
        aprilTagPipeline.initialize();

        // TODO: 初始化视觉定位器
        visionLocalizer = new VisionLocalizer(tagLayout, aprilTagPipeline);

        // TODO: 初始化融合定位器
        Pose2d initialPose = new Pose2d(0, 0, 0);
        fusionLocalizer = new FusionLocalizer(hardwareMap, visionLocalizer, initialPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // TODO: 更新融合定位器
            PoseVelocity2d velocity = fusionLocalizer.update();
            Pose2d pose = fusionLocalizer.getPose();

            // TODO: 添加遥测信息
            telemetry.addData("=== Fusion Pose ===", "");
            telemetry.addData("X", "%.2f", pose.position.x);
            telemetry.addData("Y", "%.2f", pose.position.y);
            telemetry.addData("Heading", "%.2f", Math.toDegrees(pose.heading.log()));

            telemetry.addData("=== Config ===", "");
            telemetry.addData("ACCEL_LOW_THRESHOLD", "%.1f", FusionConfig.ACCEL_LOW_THRESHOLD);
            telemetry.addData("ACCEL_HIGH_THRESHOLD", "%.1f", FusionConfig.ACCEL_HIGH_THRESHOLD);
            telemetry.addData("NOISE_SCALE_NORMAL", "%.1f", FusionConfig.NOISE_SCALE_NORMAL);
            telemetry.addData("NOISE_SCALE_LIGHT", "%.1f", FusionConfig.NOISE_SCALE_LIGHT);
            telemetry.addData("NOISE_SCALE_HEAVY", "%.1f", FusionConfig.NOISE_SCALE_HEAVY);
            telemetry.addData("INNOVATION_THRESHOLD", "%.3f", FusionConfig.INNOVATION_THRESHOLD);

            telemetry.addData("=== Control ===", "");
            telemetry.addData("Press A", "Reset Pose to (0,0,0)");
            telemetry.addData("Use Dashboard", "Tune Parameters");

            // 重置位姿
            if (gamepad1.a) {
                fusionLocalizer.setPose(new Pose2d(0, 0, 0));
                telemetry.addData("Status", "Pose Reset!");
            }

            telemetry.update();
        }

        // TODO: 停止相机
        aprilTagPipeline.stop();
    }
}
