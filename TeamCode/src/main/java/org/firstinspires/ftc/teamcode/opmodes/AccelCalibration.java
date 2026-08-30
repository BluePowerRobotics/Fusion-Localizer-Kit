package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.processors.Accelerometer.Rev9axisIMU;

import java.io.File;
import java.util.Locale;

/**
 * BNO055 (Rev9axisIMU) 校准 OpMode，功能参照官方示例
 * {@code SensorBNO055IMUCalibration}。
 *
 * <p>通过 {@link Rev9axisIMU} 封装访问设备名 {@code accel} 的 BNO055：
 * 使用融合模式 (NDOF) 初始化，因此磁力计、陀螺仪、加速度计均可校准。
 *
 * <p><b>校准动作</b>（Bosch BNO055 数据手册 §3.11，官方注释摘要）：
 * <ul>
 *   <li><b>GYR</b>：静置平放几秒；</li>
 *   <li><b>ACC</b>：缓慢旋转到不同姿态，每 45° 停几秒，保证至少一次让设备
 *       垂直于 x、y、z 各轴摆放；</li>
 *   <li><b>MAG</b>：在空中缓慢画「8」字，直到 MAG 达到 3；</li>
 *   <li><b>SYS</b>：其余三项到 3 后通常自动到 3。</li>
 * </ul>
 *
 * <p>校准达标后按 <b>A</b> 键，把校准数据序列化保存到
 * {@code BNO055IMUCalibration.json}（文件名必须与
 * {@link Rev9axisIMU#initialize()} 中 {@code calibrationDataFile} 一致，
 * 这样后续每次初始化自动加载，缩短自校准时间）。
 */
@TeleOp(name = "Accel Calibration (BNO055)", group = "Fusion")
public class AccelCalibration extends LinearOpMode {

    /** 设备名，与 5D 定位器中 Rev9axisIMU 的用法一致 */
    private static final String ACCEL_DEVICE = "accel";

    /** 校准数据保存文件名，必须与 Rev9axisIMU.initialize() 的 calibrationDataFile 一致 */
    private static final String CALIBRATION_FILE = "BNO055IMUCalibration.json";

    private Rev9axisIMU accel;
    private Orientation angles;

    @Override
    public void runOpMode() {
        telemetry.log().setCapacity(12);
        telemetry.log().add("请参照官方 SensorBNO055IMUCalibration 的校准动作：");
        telemetry.log().add("GYR 静置 / ACC 多姿态 / MAG 画8字");
        telemetry.log().add("校准达标后按 A 键保存校准数据到文件");
        telemetry.log().add("");

        // 用 Rev9axisIMU 封装初始化（NDOF 融合模式；若已有校准文件会自动加载）
        accel = new Rev9axisIMU(hardwareMap, ACCEL_DEVICE);
        boolean ready = accel.initialize();
        if (!ready) {
            telemetry.log().add("警告: BNO055 初始化失败, 请检查设备 'accel' 配置与接线");
        }

        composeTelemetry();

        while (!isStarted()) {
            telemetry.update();
            idle();
        }

        while (opModeIsActive()) {

            if (gamepad1.a) {

                // 读取当前校准数据
                BNO055IMU.CalibrationData calibrationData = accel.getAccelerometer().readCalibrationData();

                // 序列化并保存到设置目录，文件名与 Rev9axisIMU.initialize() 一致
                File file = AppUtil.getInstance().getSettingsFile(CALIBRATION_FILE);
                ReadWriteFile.writeFile(file, calibrationData.serialize());
                telemetry.log().add("已保存校准数据到 '%s'", CALIBRATION_FILE);

                // 等待按键释放，避免一次按压重复保存
                while (gamepad1.a) {
                    telemetry.update();
                    idle();
                }
            }

            telemetry.update();
        }
    }

    void composeTelemetry() {

        // 每次遥测更新前抓取一次姿态（较耗时，避免在多个 addData 中重复获取）
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = accel.getAccelerometer()
                        .getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return accel.getAccelerometer().getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        // SYS/ACC/MAG/GYR，各 0~3，3 表示完全校准
                        return accel.getAccelerometer().getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        // 附加：线性加速度读数（剔除重力），便于确认加速计轴符号/数值
        telemetry.addLine()
                .addData("linAccel", new Func<String>() {
                    @Override
                    public String value() {
                        accel.readAccelerometer();
                        return String.format(Locale.getDefault(),
                                "X %.2f Y %.2f Z %.2f inch/s²",
                                accel.getXAcceleration(), accel.getYAcceleration(), accel.getZAcceleration());
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // 格式化
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
