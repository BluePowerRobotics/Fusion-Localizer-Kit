package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.controllers.FusionLocalizer.FusionLocalizer;
import org.firstinspires.ftc.teamcode.controllers.VisionLocalizer.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.controllers.VisionLocalizer.TagLayout;
import org.firstinspires.ftc.teamcode.controllers.VisionLocalizer.VisionLocalizer;

/**
 * 融合定位系统测试OpMode
 */
@TeleOp(name = "Fusion Localizer Test", group = "Fusion Localizer")
public class FusionTestOpMode extends LinearOpMode {
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


            // 视觉信息
            Pose2d visualPose = visionLocalizer.getLastValidPose();
            if (visualPose != null) {
                telemetry.addData("Visual Pose X", "%.2f", visualPose.position.x);
                telemetry.addData("Visual Pose Y", "%.2f", visualPose.position.y);
                telemetry.addData("Visual Pose Heading", "%.2f", Math.toDegrees(visualPose.heading.log()));
            }
            telemetry.addData("Visible Tags", visionLocalizer.getVisibleTagCount());

            telemetry.update();
        }

        // TODO: 停止相机
        aprilTagPipeline.stop();
    }
}
