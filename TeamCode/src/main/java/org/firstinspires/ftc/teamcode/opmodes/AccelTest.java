package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.processors.Accelerometer.dfRobotAccelerometer;

/**
 * DFRobot LIS2DW12 加速度计测试 OpMode。
 *
 * <p>读取并显示三轴加速度（英制 inch/s² / 公制 m/s²）、原始寄存器值、
 * 传感器温度、数据就绪状态与当前量程 / 速率。
 *
 * <h3>使用说明</h3>
 * <ol>
 *   <li>将传感器接入 Hub 的 I2C 端口，设备名配置为 {@code accel}（默认地址 0x19）</li>
 *   <li>在 FTC Dashboard 中可调整 {@code rangeG}（2/4/8/16）</li>
 *   <li>按手柄 B 键重新初始化传感器（重新软复位并校验 WHO_AM_I）</li>
 *   <li>静止时 Z 轴应约等于 1g（≈386 inch/s² 或 ≈9.81 m/s²）作为自检基准</li>
 * </ol>
 */
@Config
@TeleOp(name = "Accel Test", group = "Fusion")
public class AccelTest extends LinearOpMode {

    /** 加速度计量程 (g)，支持 2 / 4 / 8 / 16 */
    public static double rangeG = 4;

    private dfRobotAccelerometer accel;
    private boolean accelReady = false;

    private boolean prevBPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        accel = new dfRobotAccelerometer(hardwareMap, "accel");

        // === 朝向配置（Facing 方式，与官方 Hub IMU 风格一致）===
        // 实际安装: 传感器 +Y 朝前、+Z 朝右、+X 朝下
        accel.setOrientation(
                dfRobotAccelerometer.FacingDirection.DOWN,
                dfRobotAccelerometer.FacingDirection.FORWARD,
                dfRobotAccelerometer.FacingDirection.RIGHT);
        
        accel.setRange(dfRobotAccelerometer.Range.RANGE_4G);
        accelReady = accel.initialize();

        waitForStart();

        while (opModeIsActive()) {
            // === 量程动态调整 ===
            int requestedRange = (int) rangeG;
            if (requestedRange != rangeToG(accel.getRange()) && isValidRange(requestedRange)) {
                accel.setRange(requestedRange);
            }

            // === 重新初始化 (B 键) ===
            boolean bPressed = gamepad1.b;
            if (bPressed && !prevBPressed) {
                if (isValidRange(requestedRange)) {
                    accel.setRange(requestedRange);
                }
                accelReady = accel.initialize();
            }
            prevBPressed = bPressed;

            // === 读取 ===
            if (accelReady) {
                accel.readAccelerometer();

                double ax = accel.getXAcceleration();
                double ay = accel.getYAcceleration();
                double az = accel.getZAcceleration();
                double mag = Math.sqrt(ax * ax + ay * ay + az * az);

                telemetry.addLine("--- Acceleration (inch/s²) ---");
                telemetry.addData("X", String.format("%+.2f", ax));
                telemetry.addData("Y", String.format("%+.2f", ay));
                telemetry.addData("Z", String.format("%+.2f", az));
                telemetry.addData("Mag", String.format("%.2f", mag));

                telemetry.addLine("--- Acceleration (m/s²) ---");
                telemetry.addData("X", String.format("%+.3f", accel.getXAccelerationMps2()));
                telemetry.addData("Y", String.format("%+.3f", accel.getYAccelerationMps2()));
                telemetry.addData("Z", String.format("%+.3f", accel.getZAccelerationMps2()));

                telemetry.addLine("--- Raw / Status ---");
                telemetry.addData("RawX", accel.getRawX());
                telemetry.addData("RawY", accel.getRawY());
                telemetry.addData("RawZ", accel.getRawZ());
                telemetry.addData("DataReady", accel.isDataReady());
                telemetry.addData("Temperature (°C)", String.format("%.1f", accel.getTemperatureCelsius()));
            }

            telemetry.addLine("--- Config ---");
            telemetry.addData("Initialized", accelReady);
            telemetry.addData("Connected", accel.isConnected());
            telemetry.addData("Range (g)", accel.getRange());
            telemetry.addData("DataRate", accel.getDataRate());
            telemetry.addData("Press B to re-init", "");

            telemetry.update();
        }
    }

    private static int rangeToG(dfRobotAccelerometer.Range range) {
        switch (range) {
            case RANGE_2G:  return 2;
            case RANGE_4G:  return 4;
            case RANGE_8G:  return 8;
            case RANGE_16G: return 16;
            default:        return 4;
        }
    }

    private static boolean isValidRange(int g) {
        return g == 2 || g == 4 || g == 8 || g == 16;
    }
}