package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.processors.Accelerometer.Rev9axisIMU;
import org.firstinspires.ftc.teamcode.processors.FusionLocalizer.AdaptiveEKF5DLocalizer;
import org.firstinspires.ftc.teamcode.processors.FusionLocalizer.AdaptiveUKF5DLocalizer;

/**
 * Rev9axisIMU (BNO055) 加速度测量功能测试。
 *
 * <p>设备名 {@code accel}（配置为 BNO055IMU 类型）。BNO055 以融合模式 (NDOF) 运行，
 * {@code getAcceleration()} 返回<b>已剔除重力</b>的线性加速度，可直接观察
 * 静止时三个轴应近似为 0，缓慢移动/倾斜时对应轴出现读数。
 *
 * <p><b>用法</b>：
 * <ul>
 *   <li>启动即读取 {@code xFacing / yFacing}（Dashboard 可调）配置朝向并初始化；</li>
 *   <li>按 <b>B</b> 键重新应用朝向并重新初始化（修改 Dashboard 朝向参数后生效）；</li>
 *   <li>静置时三轴 ≈ 0，沿 +X 前进加速时 X 为正，沿 +Y 左移时 Y 为正。</li>
 * </ul>
 *
 * <p>注意：+Z 轴方向由 x / y 按右手系自动推算（Z = X × Y），无需配置。
 */
@Config
@TeleOp(name = "Accel Test (BNO055)", group = "Fusion")
public class AccelTest extends LinearOpMode {

    // ---- Dashboard 可调：本次测试的传感器朝向（仅 x / y，z 自动推算）----
    public static Rev9axisIMU.FacingDirection xFacing = Rev9axisIMU.FacingDirection.FORWARD;
    public static Rev9axisIMU.FacingDirection yFacing = Rev9axisIMU.FacingDirection.LEFT;

    private Rev9axisIMU accel;
    private boolean accelReady = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // === 初始化（设备名 "accel"，配置为 BNO055IMU 类型）===
        accel = new Rev9axisIMU(hardwareMap, "accel");
        accel.setOrientationXY(xFacing, yFacing);
        accelReady = accel.initialize();

        telemetry.addLine("Accel Test (BNO055) Ready");
        telemetry.addLine("Dashboard 可调: xFacing, yFacing (按 B 键生效)");
        telemetry.update();

        waitForStart();

        boolean prevBPressed = false;

        while (opModeIsActive()) {
            // === 重新应用朝向并重新初始化 (B 键) ===
            boolean bPressed = gamepad1.b;
            if (bPressed && !prevBPressed) {
                accel.setOrientationXY(xFacing, yFacing);
                accelReady = accel.initialize();
            }
            prevBPressed = bPressed;

            // === 读取（失败时保留上一次的值）===
            if (accelReady) {
                accel.readAccelerometer();
            }

            // === Telemetry ===
            telemetry.addLine("--- 状态 ---");
            telemetry.addData("Initialized", accelReady);
            telemetry.addData("Connected", accel.isConnected());
            telemetry.addData("Press B to re-apply orientation & re-init", "");

            telemetry.addLine();
            telemetry.addLine("--- 线性加速度 (剔除重力, inch/s²) ---");
            telemetry.addData("X (前)", "%.4f", accel.getXAcceleration());
            telemetry.addData("Y (左)", "%.4f", accel.getYAcceleration());
            telemetry.addData("Z (上)", "%.4f", accel.getZAcceleration());

            telemetry.addLine();
            telemetry.addLine("--- 线性加速度 (剔除重力, m/s²) ---");
            telemetry.addData("X (前)", "%.4f", accel.getXAccelerationMps2());
            telemetry.addData("Y (左)", "%.4f", accel.getYAccelerationMps2());
            telemetry.addData("Z (上)", "%.4f", accel.getZAccelerationMps2());

            telemetry.addLine();
            telemetry.addLine("--- 本测试朝向 (x/y, z 自动推算) ---");
            telemetry.addData("xFacing", xFacing);
            telemetry.addData("yFacing", yFacing);

            telemetry.update();
        }
    }
}
