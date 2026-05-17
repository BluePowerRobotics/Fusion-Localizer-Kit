package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.IOException;

/**
 * 即时写入定位器实现
 * 包装了 PinpointLocalizer，并在每次更新时将位姿写入文件
 * 用于实时记录机器人位姿数据
 */
public class WriteInstantlyLocalizer implements Localizer{
    /**
     * 内部使用的定位器，初始化为 PinpointLocalizer
     */
    Localizer localizer;
    
    /**
     * 当前机器人位姿
     */
    Pose2d pose;
    
    /**
     * 每编码器 tick 对应的英寸数
     */
    double inPerTick;
    
    /**
     * 硬件映射对象，用于获取硬件设备
     */
    HardwareMap hardwareMap;
    
    /**
     * AprilTag 状态标志
     */
    boolean AprilTagStatus = false;
    
    /**
     * 构造函数
     * 
     * @param hardwareMap 硬件映射对象，用于获取硬件设备
     * @param inPerTick 每编码器 tick 对应的英寸数
     * @param initialPose 初始位姿
     */
    public WriteInstantlyLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose){
        this.hardwareMap=hardwareMap;
        this.inPerTick=inPerTick;
        // 初始化内部定位器为 PinpointLocalizer
        this.localizer =new PinpointLocalizer(hardwareMap,inPerTick,initialPose);
        // 获取初始位姿
        pose=localizer.getPose();
    }
    
    @Override
    public void setPose(Pose2d pose) {
        // 委托给内部定位器设置位姿
        localizer.setPose(pose);
    }

    @Override
    public Pose2d getPose() {
        // 返回当前位姿的副本
        return new Pose2d(pose.position,pose.heading);
    }

    @Override
    public PoseVelocity2d update() {
        // 更新内部定位器
        PoseVelocity2d poseVelocity2d = localizer.update();
        // 获取最新位姿
        Pose2d pose = localizer.getPose();
        
        // 检查位姿是否有效（非 NaN）
        if(Double.isNaN(pose.position.x)||Double.isNaN(pose.position.y)||Double.isNaN(pose.heading.toDouble())){
            // 如果位姿无效，重新初始化定位器
            localizer = new PinpointLocalizer(hardwareMap,inPerTick,this.pose);
            // 重新更新
            poseVelocity2d = localizer.update();
            pose = localizer.getPose();
        }
        
        // 更新当前位姿
        this.pose = pose;
        
        // 将位姿写入文件
        try (java.io.FileWriter writer = new java.io.FileWriter("/sdcard/FIRST/pose.txt")) {
            writer.write(pose.position.x + "," +
                    pose.position.y + "," +
                    pose.heading.toDouble());
        } catch (IOException e) {
            // 处理文件写入异常
            throw new RuntimeException(e);
        }
        
        // 返回速度估计
        return poseVelocity2d;
    }


}
