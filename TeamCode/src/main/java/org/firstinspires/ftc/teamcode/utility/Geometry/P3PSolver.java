package org.firstinspires.ftc.teamcode.utility.Geometry;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;

import org.firstinspires.ftc.teamcode.utility.Algebra.ComplexNumber;
import org.firstinspires.ftc.teamcode.utility.Algebra.EquationSolver;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

import java.util.ArrayList;
import java.util.List;

/**
 * P3P（Perspective-3-Point）位姿求解器 - Kneip 算法实现
 */
public class P3PSolver {
    private static final double EPSILON = 1e-10;
    private static final double PI = Math.PI;

    /**
     * P3P求解结果
     */
    public static class P3PResult {
        public Pose2d pose;
        public double confidence;
    }

    /**
     * 使用3个标记点求解位姿（Kneip算法）
     *
     * @param worldPoints 标记点世界坐标列表（至少3个）
     * @param bearings 标记点方位角列表（弧度，相对于机器人前方）
     * @param distances 标记点距离列表（米）
     * @return 求解结果，若失败则返回null
     */
    public P3PResult solve3Point(List<Vector2D> worldPoints, List<Double> bearings, List<Double> distances) {
        if (worldPoints.size() != 3 || bearings.size() != 3 || distances.size() != 3) {
            return null;
        }

        // 检查退化情况
        if (isDegenerate(worldPoints)) {
            return null;
        }

        // 获取三个标记点
        Vector2D P1 = worldPoints.get(0);
        Vector2D P2 = worldPoints.get(1);
        Vector2D P3 = worldPoints.get(2);

        double alpha1 = bearings.get(0);
        double alpha2 = bearings.get(1);
        double alpha3 = bearings.get(2);

        // double d1 = distances.get(0); // 实际不使用，d12/d13由世界坐标计算
        double d12 = Vector2D.getDistance(P1, P2);
        double d13 = Vector2D.getDistance(P1, P3);
        double d23 = Vector2D.getDistance(P2, P3);

        // 步骤1：构建观测方向向量（在相机坐标系下）
        // 相机光轴朝向机器人前方（y轴正方向）
        // f = (sin(alpha), cos(alpha), 0) 归一化后
        double[] f1 = normalize(new double[]{Math.sin(alpha1), Math.cos(alpha1), 0});
        double[] f2 = normalize(new double[]{Math.sin(alpha2), Math.cos(alpha2), 0});
        double[] f3 = normalize(new double[]{Math.sin(alpha3), Math.cos(alpha3), 0});

        // 步骤2：构建新的相机坐标系 (C, Tx, Ty, Tz)
        // Tz 沿第一个观测方向
        double[] Tz = f1;
        
        // Tx 垂直于 Tz，在 f2 和 Tz 构成的平面内
        double[] f2_proj_Tz = multiply(dot(f2, Tz), Tz);
        double[] Tx_candidate = subtract(f2, f2_proj_Tz);
        double Tx_len = norm(Tx_candidate);
        
        if (Tx_len < EPSILON) {
            // f2 接近平行于 f1，退化，使用 f3 构造 Tx
            double[] f3_proj_Tz = multiply(dot(f3, Tz), Tz);
            Tx_candidate = subtract(f3, f3_proj_Tz);
            Tx_len = norm(Tx_candidate);
            if (Tx_len < EPSILON) {
                return null; // 无法构造有效坐标系
            }
        }
        double[] Tx = divide(Tx_candidate, Tx_len);
        
        // Ty 垂直于 Tz 和 Tx
        double[] Ty = cross(Tz, Tx);

        // 步骤3：构建新的世界坐标系 (P1, Nx, Ny, Nz)
        // Nz 沿 P1->P2 方向
        double[] P2_minus_P1 = {P2.getX() - P1.getX(), P2.getY() - P1.getY(), 0};
        double[] Nz = normalize(P2_minus_P1);
        
        // Nx 垂直于 Nz，在 P3-P1 和 Nz 构成的平面内
        double[] P3_minus_P1 = {P3.getX() - P1.getX(), P3.getY() - P1.getY(), 0};
        double[] temp = subtract(P3_minus_P1, multiply(dot(P3_minus_P1, Nz), Nz));
        double temp_len = norm(temp);
        if (temp_len < EPSILON) {
            return null; // P3 接近在 P1-P2 连线上，退化
        }
        double[] Nx = divide(temp, temp_len);
        
        // Ny 垂直于 Nz 和 Nx
        double[] Ny = cross(Nz, Nx);

        // 步骤4：计算中间变量
        // 观测向量在新相机坐标系下的坐标
        double m2 = dot(f2, Tx);
        double n2 = dot(f2, Ty);
        double m3 = dot(f3, Tx);
        double n3 = dot(f3, Ty);

        // 检查 n2 和 n3 是否过小（会导致方程退化）
        if (Math.abs(n2) < EPSILON || Math.abs(n3) < EPSILON) {
            return null; // 需要特殊处理，本简化版本不支持
        }

        // 构建四次方程系数（基于 Kneip 公式）
        // 注意：这里使用 n12=d12, n13=d13（距离）
        double n12_sq = d12 * d12;
        double n13_sq = d13 * d13;
        double n2_sq = n2 * n2;
        double n3_sq = n3 * n3;
        double m2_sq = m2 * m2;
        double m3_sq = m3 * m3;

        double a4 = n2_sq * n3_sq;
        double a3 = -2 * n2_sq * n3_sq;
        double a2 = n2_sq * n3_sq + 2 * m2_sq * n3_sq + n2_sq * m3_sq - n12_sq * n3_sq - n13_sq * n2_sq;
        double a1 = -2 * m2_sq * n3_sq - 2 * n2_sq * m3_sq + 2 * n12_sq * n3_sq;
        double a0 = m2_sq * n3_sq + n2_sq * m3_sq - n12_sq * n3_sq;

        // 步骤5：使用复数版本求解四次方程
        ComplexNumber[] complexRoots = EquationSolver.solve4Complex(a4, a3, a2, a1, a0);
        double[] roots = EquationSolver.filterRealRoots(complexRoots);
        
        if (roots.length == 0) {
            return null;
        }

        // 步骤6：验证解并计算位姿
        List<Pose2d> candidates = new ArrayList<>();
        for (double root : roots) {
            // root 对应 kneip 算法中的参数比值 u/v
            if (root <= 0) continue;
            
            // 计算分母：denom = 1 + root^2 - 2*root*m2
            double denom = 1 + root * root - 2 * root * m2;
            if (denom <= EPSILON) {
                continue; // 分母非正，跳过
            }
            
            // 计算 v = d12 / sqrt(denom)
            double v = d12 / Math.sqrt(denom);
            if (v <= 0 || Double.isNaN(v) || Double.isInfinite(v)) {
                continue;
            }
            
            // 计算 u = root * v
            double u = root * v;
            
            // 计算相机在新世界坐标系下的坐标 (C_new)
            // C_new = [u*m2 - v, u*n2, u*dot(f2,Tz)]
            double C_new_x = u * m2 - v;
            double C_new_y = u * n2;
            double C_new_z = u * dot(f2, Tz);
            
            // 构建从新相机坐标系到新世界坐标系的旋转矩阵 R
            // R 的列为 [Tx, Ty, Tz]（即 R[i][j] = coordinate_j of axis_i）
            // 但更准确地说，我们需要构建从世界到相机的旋转矩阵 R_cw
            // R_cw 的行是相机轴在世界坐标系中的表示
            // R_cw = [Tx^T; Ty^T; Tz^T]
            
            // 将 C_new 从新世界坐标系转换到原世界坐标系
            // C_world = P1 + N * C_new，其中 N 的列为 [Nx, Ny, Nz]
            double[] C_world = {
                P1.getX() + Nx[0] * C_new_x + Ny[0] * C_new_y + Nz[0] * C_new_z,
                P1.getY() + Nx[1] * C_new_x + Ny[1] * C_new_y + Nz[1] * C_new_z,
                Nx[2] * C_new_x + Ny[2] * C_new_y + Nz[2] * C_new_z
            };
            
            // 构建从世界到相机的新旋转矩阵 R_cw
            // 相机坐标系轴在世界坐标系中的表示：
            // 相机 x 轴（Tx）世界坐标系表示：R_cw^T * Tx = [Tx·Nx, Tx·Ny, Tx·Nz]^T
            // 但根据 Kneip 定义，R_cw 的行是：
            // R_cw[0] = Tx^T（Tx 在世界坐标系中的坐标）
            // R_cw[1] = Ty^T
            // R_cw[2] = Tz^T
            
            double[][] R_cw = new double[3][3];
            R_cw[0] = new double[]{dot(Tx, Nx), dot(Tx, Ny), dot(Tx, Nz)};
            R_cw[1] = new double[]{dot(Ty, Nx), dot(Ty, Ny), dot(Ty, Nz)};
            R_cw[2] = new double[]{dot(Tz, Nx), dot(Tz, Ny), dot(Tz, Nz)};
            
            // 相机到世界的旋转矩阵 R_wc = R_cw^T
            // 相机光轴方向（y轴）在世界坐标系中为 R_wc * (0,1,0) = R_cw^T 的第二列
            double forward_x = R_cw[0][1];  // 第二列第0行
            double forward_y = R_cw[1][1];  // 第二列第1行
            
            // 航向角：相机前方（y轴正方向）在世界坐标系中的角度
            // 标准 atan2(y, x) 返回从 x 轴正方向到向量 (x,y) 的角度
            double theta = Math.atan2(forward_y, forward_x);
            
            Pose2d candidate = new Pose2d(C_world[0], C_world[1], theta);
            
            // 验证距离约束
            if (validateCandidate(candidate, worldPoints, distances)) {
                candidates.add(candidate);
            }
        }

        if (candidates.isEmpty()) {
            return null;
        }

        // 如果有多个候选解，选择距离约束误差最小的
        Pose2d bestPose = candidates.get(0);
        if (candidates.size() > 1) {
            double bestScore = Double.MAX_VALUE;
            for (Pose2d candidate : candidates) {
                double score = computeDistanceError(candidate, worldPoints, distances);
                if (score < bestScore) {
                    bestScore = score;
                    bestPose = candidate;
                }
            }
        }

        P3PResult result = new P3PResult();
        result.pose = bestPose;
        result.confidence = 1.0 / Math.max(1, candidates.size());
        
        return result;
    }

    /**
     * 验证候选解是否满足距离约束
     */
    private boolean validateCandidate(Pose2d candidate, List<Vector2D> worldPoints, List<Double> distances) {
        for (int i = 0; i < worldPoints.size(); i++) {
            Vector2D P = worldPoints.get(i);
            double dx = P.getX() - candidate.position.x;
            double dy = P.getY() - candidate.position.y;
            double actualDist = Math.hypot(dx, dy);
            double expectedDist = distances.get(i);
            
            // 固定误差阈值 0.2m
            if (Math.abs(actualDist - expectedDist) > 0.2) {
                return false;
            }
        }
        return true;
    }

    /**
     * 计算候选解的距离误差
     */
    private double computeDistanceError(Pose2d candidate, List<Vector2D> worldPoints, List<Double> distances) {
        double error = 0;
        for (int i = 0; i < worldPoints.size(); i++) {
            Vector2D P = worldPoints.get(i);
            double dx = P.getX() - candidate.position.x;
            double dy = P.getY() - candidate.position.y;
            double actualDist = Math.hypot(dx, dy);
            double expectedDist = distances.get(i);
            error += Math.abs(actualDist - expectedDist);
        }
        return error;
    }

    /**
     * 使用4个或更多标记点求解位姿（最小二乘）
     *
     * @param worldPoints 标记点世界坐标列表
     * @param bearings 标记点方位角列表（弧度）
     * @param distances 标记点距离列表（米）
     * @return 求解结果，若失败则返回null
     */
    public P3PResult solveMultiPoint(List<Vector2D> worldPoints, List<Double> bearings, List<Double> distances) {
        if (worldPoints.size() < 4) {
            return solve3Point(worldPoints, bearings, distances);
        }

        // 使用最小二乘法融合多个P3P解
        List<Pose2d> allCandidates = new ArrayList<>();
        
        // 采样多组3点组合求解
        int n = worldPoints.size();
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                for (int k = j + 1; k < n; k++) {
                    List<Vector2D> subsetPoints = new ArrayList<>();
                    List<Double> subsetBearings = new ArrayList<>();
                    List<Double> subsetDistances = new ArrayList<>();
                    
                    subsetPoints.add(worldPoints.get(i));
                    subsetPoints.add(worldPoints.get(j));
                    subsetPoints.add(worldPoints.get(k));
                    subsetBearings.add(bearings.get(i));
                    subsetBearings.add(bearings.get(j));
                    subsetBearings.add(bearings.get(k));
                    subsetDistances.add(distances.get(i));
                    subsetDistances.add(distances.get(j));
                    subsetDistances.add(distances.get(k));
                    
                    P3PResult result = solve3Point(subsetPoints, subsetBearings, subsetDistances);
                    if (result != null && result.pose != null) {
                        allCandidates.add(result.pose);
                    }
                }
            }
        }

        if (allCandidates.isEmpty()) {
            return null;
        }

        // 选择距离误差最小的解
        Pose2d bestPose = null;
        double bestScore = Double.MAX_VALUE;
        
        for (Pose2d candidate : allCandidates) {
            double score = computeDistanceError(candidate, worldPoints, distances);
            if (score < bestScore) {
                bestScore = score;
                bestPose = candidate;
            }
        }

        if (bestPose == null) {
            return null;
        }

        P3PResult result = new P3PResult();
        result.pose = bestPose;
        result.confidence = Math.max(0, 1.0 - bestScore / worldPoints.size());
        
        return result;
    }

    /**
     * 使用历史位姿选择合理的候选解
     *
     * @param candidates 候选解列表
     * @param historyPose 历史位姿
     * @return 选择的最佳解
     */
    public Pose2d selectBestCandidate(List<Pose2d> candidates, Pose2d historyPose) {
        if (candidates == null || candidates.isEmpty()) {
            return null;
        }

        if (historyPose == null) {
            return candidates.get(0);
        }

        Pose2d bestCandidate = null;
        double minDistance = Double.MAX_VALUE;

        for (Pose2d candidate : candidates) {
            double dx = candidate.position.x - historyPose.position.x;
            double dy = candidate.position.y - historyPose.position.y;
            double dTheta = Math.abs(normalizeAngle(candidate.heading.log() - historyPose.heading.log()));

            // 综合距离和角度差异
            double distance = Math.sqrt(dx * dx + dy * dy) + dTheta;

            if (distance < minDistance) {
                minDistance = distance;
                bestCandidate = candidate;
            }
        }

        return bestCandidate;
    }

    /**
     * 角度归一化到 [-PI, PI]
     */
    private double normalizeAngle(double angle) {
        while (angle > PI) angle -= 2 * PI;
        while (angle < -PI) angle += 2 * PI;
        return angle;
    }

    /**
     * 检查是否存在退化情况
     *
     * @param worldPoints 标记点世界坐标列表
     * @return 是否存在退化情况
     */
    public boolean isDegenerate(List<Vector2D> worldPoints) {
        if (worldPoints.size() < 3) {
            return true;
        }

        Vector2D P1 = worldPoints.get(0);
        Vector2D P2 = worldPoints.get(1);
        Vector2D P3 = worldPoints.get(2);

        // 检查三点是否共线（三角形面积接近0）
        double area = Math.abs(
            (P2.getX() - P1.getX()) * (P3.getY() - P1.getY()) -
            (P3.getX() - P1.getX()) * (P2.getY() - P1.getY())
        );
        
        if (area < 1e-6) {
            return true; // 共线
        }

        // 检查是否形成钝角三角形（可能导致危险圆问题）
        double d12 = Vector2D.getDistance(P1, P2);
        double d13 = Vector2D.getDistance(P1, P3);
        double d23 = Vector2D.getDistance(P2, P3);
        
        double maxDist = Math.max(d12, Math.max(d13, d23));
        double sumSquares = d12 * d12 + d13 * d13 + d23 * d23;
        
        // 如果最大边的平方大于其他两边平方和，则是钝角三角形
        if (maxDist * maxDist > sumSquares - maxDist * maxDist) {
            return true; // 钝角三角形，可能存在危险圆问题
        }

        return false;
    }

    // ==================== 向量运算辅助函数 ====================
    
    private double[] normalize(double[] v) {
        double n = norm(v);
        if (n < EPSILON) {
            return new double[]{0, 0, 0};
        }
        return new double[]{v[0] / n, v[1] / n, v[2] / n};
    }

    private double norm(double[] v) {
        return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    private double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private double[] cross(double[] a, double[] b) {
        return new double[]{
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        };
    }

    private double[] add(double[] a, double[] b) {
        return new double[]{a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    private double[] subtract(double[] a, double[] b) {
        return new double[]{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private double[] multiply(double scalar, double[] v) {
        return new double[]{scalar * v[0], scalar * v[1], scalar * v[2]};
    }

    private double[] divide(double[] v, double scalar) {
        if (Math.abs(scalar) < EPSILON) {
            throw new ArithmeticException("Division by zero in vector divide");
        }
        return new double[]{v[0] / scalar, v[1] / scalar, v[2] / scalar};
    }
}
