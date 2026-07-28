package org.firstinspires.ftc.teamcode.controllers.Chassis;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.HypParams;

/**
 * 基于 RoadRunner MecanumDrive 的麦轮底盘控制器。
 *
 * 封装了 RoadRunner 的驱动与定位能力，对外提供与之前一致的
 * {@link #update(double, double, double)} 接口 (归一化 -1~1 输入)。
 */
public class Chassis {

    /** RoadRunner 底层驱动 */
    public final MecanumDrive mecanumDrive;

    /** 最大线速度 (inch/s) */
    private double maxV;
    /** 最大角速度 (rad/s) */
    private double maxOmega;

    /**
     * @param hardwareMap 硬件映射
     * @param initialPose 初始位姿 (x, y, heading)
     */
    public Chassis(HardwareMap hardwareMap, Pose2d initialPose) {
        this.mecanumDrive = new MecanumDrive(hardwareMap, initialPose);
    }

    /**
     * @param hardwareMap 硬件映射
     */
    public Chassis(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose2d(0, 0, 0));
    }

    // ==================== 驱动控制 ====================

    /**
     * 每帧更新底盘速度。
     *
     * @param vx    机器人坐标系 x 方向速度 (前进, -1~1)
     * @param vy    机器人坐标系 y 方向速度 (左移, -1~1)
     * @param omega 角速度 (逆时针, -1~1)
     */
    public void update(double vx, double vy, double omega) {
        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(vx * maxV, vy * maxV),
                omega * maxOmega
        ));
    }

    /**
     * 使用 Gamepad 控制底盘。
     */
    public void update(Gamepad gamepad) {
        double vx = -gamepad.left_stick_y;
        double vy = -gamepad.left_stick_x;
        double omega = -gamepad.right_stick_x;
        update(vx, vy, omega);
    }

    /**
     * 停止所有电机。
     */
    public void stop() {
        mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    // ==================== 定位 ====================

    /**
     * 更新定位器并返回当前速度估计。
     * 每帧应调用一次以保持定位器状态同步。
     *
     * @return 当前速度估计 (in/s, in/s, rad/s)
     */
    public PoseVelocity2d updatePoseEstimate() {
        return mecanumDrive.updatePoseEstimate();
    }

    /**
     * @return 当前融合位姿 {x, y, heading} (英寸, 英寸, 弧度)
     */
    public Pose2d getPose() {
        return mecanumDrive.localizer.getPose();
    }

    /**
     * 设置当前位姿 (用于重置定位)。
     */
    public void setPose(Pose2d pose) {
        mecanumDrive.localizer.setPose(pose);
    }

    // ==================== 参数 ====================

    public double getMaxV() { return maxV; }
    public void setMaxV(double maxV) { this.maxV = maxV; }
    public double getMaxOmega() { return maxOmega; }
    public void setMaxOmega(double maxOmega) { this.maxOmega = maxOmega; }
}