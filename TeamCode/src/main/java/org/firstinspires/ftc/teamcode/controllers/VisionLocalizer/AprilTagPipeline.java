package org.firstinspires.ftc.teamcode.controllers.VisionLocalizer;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

/**
 * 封装相机与AprilTag检测的类
 */
public class AprilTagPipeline {
    /**
     * 检测到的AprilTag信息
     */
    public static class DetectedTag {
        public int id;
        public double bearing; // 方位角（弧度）
        public double distance; // 距离（米）
    }

    /**
     * OpenCV相机实例
     */
    private OpenCvCamera camera;

    /**
     * 检测到的标记点列表
     */
    private List<DetectedTag> detectedTags;

    /**
     * 构造函数
     *
     * @param hardwareMap 硬件映射
     * @param cameraName 相机名称
     */
    public AprilTagPipeline(HardwareMap hardwareMap, String cameraName) {
        // TODO: 初始化相机
    }

    /**
     * 初始化相机并开始捕获
     */
    public void initialize() {
        // TODO: 初始化相机并开始捕获
    }

    /**
     * 停止相机捕获
     */
    public void stop() {
        // TODO: 停止相机捕获
    }

    /**
     * 获取最新检测到的标记点列表
     *
     * @return 检测到的标记点列表
     */
    public List<DetectedTag> getDetectedTags() {
        // TODO: 返回最新检测到的标记点列表
        return null;
    }

    /**
     * 检查是否有有效的检测结果
     *
     * @return 是否有有效的检测结果
     */
    public boolean hasValidDetection() {
        // TODO: 检查是否有有效的检测结果
        return false;
    }

    /**
     * 将遥测信息添加到界面
     *
     * @param telemetry 遥测对象
     */
    public void addTelemetry(Telemetry telemetry) {
        // TODO: 添加遥测信息
    }
}
