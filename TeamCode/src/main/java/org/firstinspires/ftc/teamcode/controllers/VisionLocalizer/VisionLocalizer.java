package org.firstinspires.ftc.teamcode.controllers.VisionLocalizer;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utility.Geometry.P3PSolver;

/**
 * 实现视觉位姿解算的类
 */
public class VisionLocalizer {
    /**
     * 标记点布局
     */
    private final TagLayout tagLayout;

    /**
     * AprilTag检测流水线
     */
    private final AprilTagPipeline aprilTagPipeline;

    /**
     * P3P求解器
     */
    private final P3PSolver p3pSolver;

    /**
     * 上一次有效的视觉位姿
     */
    private Pose2d lastValidPose;

    /**
     * 构造函数
     *
     * @param tagLayout 标记点布局
     * @param aprilTagPipeline AprilTag检测流水线
     */
    public VisionLocalizer(TagLayout tagLayout, AprilTagPipeline aprilTagPipeline) {
        this.tagLayout = tagLayout;
        this.aprilTagPipeline = aprilTagPipeline;
        this.p3pSolver = new P3PSolver();
    }

    /**
     * 更新视觉位姿估计
     *
     * @return 视觉位姿估计，若无效则返回null
     */
    public Pose2d update() {
        // TODO: 更新视觉位姿估计
        return null;
    }

    /**
     * 获取上一次有效的视觉位姿
     *
     * @return 上一次有效的视觉位姿
     */
    public Pose2d getLastValidPose() {
        // TODO: 返回上一次有效的视觉位姿
        return lastValidPose;
    }

    /**
     * 获取当前可见的标记点数量
     *
     * @return 可见标记点数量
     */
    public int getVisibleTagCount() {
        // TODO: 返回当前可见的标记点数量
        return 0;
    }

    /**
     * 检查是否有有效的视觉位姿
     *
     * @return 是否有有效的视觉位姿
     */
    public boolean hasValidPose() {
        // TODO: 检查是否有有效的视觉位姿
        return false;
    }
}
