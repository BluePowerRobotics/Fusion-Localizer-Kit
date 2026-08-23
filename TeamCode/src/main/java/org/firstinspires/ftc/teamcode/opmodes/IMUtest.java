package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "IMU Test", group = "Fusion")
public class IMUtest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        IMU hubImu = hardwareMap.get(IMU.class, "imu");
        hubImu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        )));

        telemetry.addLine("IMU initialized");
        telemetry.addLine("Initial pose: (24, 24, PI/2)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            YawPitchRollAngles angles = hubImu.getRobotYawPitchRollAngles();
            AngularVelocity angVel = hubImu.getRobotAngularVelocity(AngleUnit.RADIANS);

            double yaw   = angles.getYaw(AngleUnit.DEGREES);
            double pitch = angles.getPitch(AngleUnit.DEGREES);
            double roll  = angles.getRoll(AngleUnit.DEGREES);

            telemetry.addData("Yaw",   "%.2f°", yaw);
            telemetry.addData("Pitch", "%.2f°", pitch);
            telemetry.addData("Roll",  "%.2f°", roll);

            telemetry.addLine();
            telemetry.addData("Yaw (rad)",   "%.4f", Math.toRadians(yaw));
            telemetry.addData("Pitch (rad)", "%.4f", Math.toRadians(pitch));
            telemetry.addData("Roll (rad)",  "%.4f", Math.toRadians(roll));

            telemetry.addLine();
            if (angVel != null) {
                // IMU 体坐标系 (X=右, Y=前, Z=上) 与 RR 体坐标系 (X=前, Y=左, Z=上) 的映射:
                //   Robot X (右) → RR -Y (右) → pitch 轴 (前后倾斜)
                //   Robot Y (前) → RR X (前) → roll 轴 (左右倾斜)
                telemetry.addData("xRotRate (pitch)", "%.4f rad/s", angVel.xRotationRate);
                telemetry.addData("yRotRate (roll)",  "%.4f rad/s", angVel.yRotationRate);
                telemetry.addData("zRotRate (yaw)",   "%.4f rad/s", angVel.zRotationRate);
            }

            telemetry.update();
        }
    }
}
