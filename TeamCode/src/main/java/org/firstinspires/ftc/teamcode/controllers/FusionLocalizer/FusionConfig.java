package org.firstinspires.ftc.teamcode.controllers.FusionLocalizer;

import com.acmerobotics.dashboard.config.Config;

/**
 * 融合定位系统的可调参数配置类
 */
@Config
public class FusionConfig {
    // IMU冲击检测阈值
    public static double ACCEL_LOW_THRESHOLD = 5.0; // m/s²
    public static double ACCEL_HIGH_THRESHOLD = 12.0; // m/s²

    // 噪声放大系数
    public static double NOISE_SCALE_NORMAL = 1.0;
    public static double NOISE_SCALE_LIGHT = 10.0;
    public static double NOISE_SCALE_HEAVY = 50.0;

    // 新息阈值
    public static double INNOVATION_THRESHOLD = 0.5; // 米

    // 基础过程噪声协方差
    public static double PROCESS_NOISE_X = 0.01;
    public static double PROCESS_NOISE_Y = 0.01;
    public static double PROCESS_NOISE_THETA = 0.01;

    // 观测噪声协方差（与标记点数量相关）
    public static double OBSERVATION_NOISE_BASE = 0.1;
    public static double OBSERVATION_NOISE_PER_TAG = 0.02;

    // 最大迭代次数
    public static int MAX_ITERATIONS = 2;
    public static int MIN_TAGS_FOR_SECOND_ITERATION = 5;
}
