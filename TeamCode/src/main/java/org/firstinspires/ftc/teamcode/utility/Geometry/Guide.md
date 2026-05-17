# Geometry 几何计算工具

## 概述

本目录包含几何计算相关的工具类，主要用于处理凸多边形和2D几何运算。

## 文件列表

| 文件 | 功能说明 |
|------|----------|
| `ConvexPolygon.java` | 凸多边形类，支持碰撞检测、坐标转换等 |

## ConvexPolygon 使用说明

### 功能特性

- 凸多边形的创建和验证
- 坐标转换（相对坐标↔绝对坐标）
- 点是否在多边形内的检测
- 多边形碰撞检测（包含、相交）
- 多边形交集计算
- 最近点计算

### 使用示例

```java
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

// 创建三角形
Vector2D v1 = new Vector2D(0, 0);
Vector2D v2 = new Vector2D(10, 0);
Vector2D v3 = new Vector2D(5, 10);
ConvexPolygon triangle = new ConvexPolygon(v1, v2, v3);

// 创建四边形
ConvexPolygon rectangle = new ConvexPolygon(
    new Vector2D(0, 0),
    new Vector2D(20, 0),
    new Vector2D(20, 15),
    new Vector2D(0, 15)
);

// 检测点是否在多边形内
Vector2D point = new Vector2D(5, 5);
boolean contains = triangle.Contains(point);

// 检测两个多边形是否相交
boolean intersects = triangle.IsIntersected(rectangle);

// 计算最近点向量（从点到多边形边界）
Vector2D nearest = rectangle.NearestVectorFrom(25, 10);

// 坐标转换：相对坐标转绝对坐标
ConvexPolygon absolutePolygon = rectangle.inAbsolute(100, 200, Math.toRadians(45));
```

### 方法签名

| 方法 | 说明 | 参数 |
|------|------|------|
| `ConvexPolygon(vertices...)` | 构造函数，接受3个或更多顶点 | `Vector2D...` 顶点数组 |
| `Contains(x, y)` | 检测点是否在多边形内 | `double x, double y` |
| `Contains(Vector2D)` | 检测点是否在多边形内 | `Vector2D point` |
| `Contains(ConvexPolygon)` | 检测多边形是否完全包含另一个 | `ConvexPolygon other` |
| `IsIntersected(ConvexPolygon)` | 检测两个多边形是否相交 | `ConvexPolygon other` |
| `IntersectWith(ConvexPolygon)` | 计算两个多边形的交集 | `ConvexPolygon clip` |
| `NearestVectorFrom(x, y)` | 计算点到多边形边界的最近向量 | `double x, double y` |
| `inRelative(x, y, theta)` | 将多边形转换为相对坐标 | `double x, double y, double theta` |
| `inAbsolute(x, y, theta)` | 将多边形转换为绝对坐标 | `double x, double y, double theta` |
| `getVertices()` | 获取所有顶点 | 返回 `Vector2D[]` |

### 注意事项

1. 构造函数会自动验证顶点是否构成凸多边形，非凸多边形会抛出异常
2. 顶点会自动按逆时针方向排序
3. 交集计算要求两个多边形必须相交，否则抛出异常
4. 使用 `Pose2d` 进行坐标转换需要导入 Road Runner 库