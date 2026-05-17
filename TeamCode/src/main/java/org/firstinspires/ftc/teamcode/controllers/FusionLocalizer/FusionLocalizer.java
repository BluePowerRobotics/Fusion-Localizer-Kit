package org.firstinspires.ftc.teamcode.controllers.FusionLocalizer;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.controllers.VisionLocalizer.VisionLocalizer;

/**
 * 实现RoadRunner Localizer接口的融合定位器
 */
public class FusionLocalizer implements Localizer {
    /**
     * 融合位姿估计器
     */
    private final FusionPoseEstimator poseEstimator;

    /**
     * 视觉定位器
     */
    private final VisionLocalizer visionLocalizer;

    /**
     * Pinpoint驱动
     */
    private final GoBildaPinpointDriver pinpointDriver;

    /**
     * 构造函数
     *
     * @param hardwareMap 硬件映射
     * @param visionLocalizer 视觉定位器
     * @param initialPose 初始位姿
     */
    public FusionLocalizer(HardwareMap hardwareMap, VisionLocalizer visionLocalizer, Pose2d initialPose) {
        this.visionLocalizer = visionLocalizer;
        this.poseEstimator = new FusionPoseEstimator(initialPose);
        // TODO: 初始化Pinpoint驱动
        this.pinpointDriver = null;
    }

    /**
     * 设置当前位姿
     *
     * @param pose 要设置的位姿
     */
    @Override
    public void setPose(Pose2d pose) {
        // TODO: 设置当前位姿
    }

    /**
     * 获取当前位姿估计
     *
     * @return 当前位姿估计
     */
    @Override
    public Pose2d getPose() {
        // TODO: 返回当前位姿估计
        return null;
    }

    /**
     * 更新定位器的位姿估计
     *
     * @return 当前速度估计
     */
    @Override
    public PoseVelocity2d update() {
        // TODO: 更新定位器的位姿估计
        return null;
    }
}
