package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.VisionLocalizer.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.controllers.VisionLocalizer.TagLayout;
import org.firstinspires.ftc.teamcode.controllers.VisionLocalizer.VisionLocalizer;

/**
 * AprilTag视觉定位测试OpMode
 */
@TeleOp(name = "AprilTag Test", group = "Fusion Localizer")
public class AprilTagTestOpMode extends LinearOpMode {
    private TagLayout tagLayout;
    private AprilTagPipeline aprilTagPipeline;
    private VisionLocalizer visionLocalizer;

    @Override
    public void runOpMode() {
        // TODO: 初始化标记点布局
        tagLayout = new TagLayout();
        // 示例：添加几个标记点
        // tagLayout.addTag(0, new Vector2d(0, 0));
        // tagLayout.addTag(1, new Vector2d(3, 0));
        // tagLayout.addTag(2, new Vector2d(0, 3));

        // TODO: 初始化AprilTag检测流水线
        aprilTagPipeline = new AprilTagPipeline(hardwareMap, "Webcam 1");
        aprilTagPipeline.initialize();

        // TODO: 初始化视觉定位器
        visionLocalizer = new VisionLocalizer(tagLayout, aprilTagPipeline);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // TODO: 更新视觉定位器
            Pose2d pose = visionLocalizer.update();

            // TODO: 添加遥测信息
            if (pose != null) {
                telemetry.addData("Pose X", "%.2f", pose.position.x);
                telemetry.addData("Pose Y", "%.2f", pose.position.y);
                telemetry.addData("Pose Heading", "%.2f", Math.toDegrees(pose.heading.log()));
            }
            telemetry.addData("Visible Tags", visionLocalizer.getVisibleTagCount());
            telemetry.addData("Valid Pose", visionLocalizer.hasValidPose());
            aprilTagPipeline.addTelemetry(telemetry);

            telemetry.update();
        }

        // TODO: 停止相机
        aprilTagPipeline.stop();
    }
}
