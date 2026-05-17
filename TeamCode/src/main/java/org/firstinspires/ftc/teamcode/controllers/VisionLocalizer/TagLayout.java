package org.firstinspires.ftc.teamcode.controllers.VisionLocalizer;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.HashMap;
import java.util.Map;

/**
 * 管理AprilTag标记点世界坐标的类
 */
public class TagLayout {
    /**
     * 存储标记点ID到世界坐标的映射
     */
    private final Map<Integer, Vector2d> tagPositions = new HashMap<>();

    /**
     * 添加一个标记点的世界坐标
     *
     * @param tagId 标记点ID
     * @param position 标记点在世界坐标系中的位置
     */
    public void addTag(int tagId, Vector2d position) {
        // TODO: 添加标记点到映射中
    }

    /**
     * 获取指定ID标记点的世界坐标
     *
     * @param tagId 标记点ID
     * @return 标记点的世界坐标，若不存在则返回null
     */
    public Vector2d getTagPosition(int tagId) {
        // TODO: 从映射中获取标记点位置
        return null;
    }

    /**
     * 获取所有已注册的标记点ID
     *
     * @return 标记点ID的集合
     */
    public Iterable<Integer> getAllTagIds() {
        // TODO: 返回所有标记点ID
        return null;
    }

    /**
     * 检查指定ID的标记点是否存在
     *
     * @param tagId 标记点ID
     * @return 标记点是否存在
     */
    public boolean hasTag(int tagId) {
        // TODO: 检查标记点是否存在
        return false;
    }

    /**
     * 移除指定ID的标记点
     *
     * @param tagId 标记点ID
     */
    public void removeTag(int tagId) {
        // TODO: 从映射中移除标记点
    }

    /**
     * 清空所有标记点
     */
    public void clear() {
        // TODO: 清空所有标记点
    }
}
