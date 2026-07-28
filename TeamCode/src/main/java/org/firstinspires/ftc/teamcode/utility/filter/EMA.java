package org.firstinspires.ftc.teamcode.utility.filter;

/**
 * EMA（指数移动平均）滤波器
 * 公式：filteredValue = alpha * x + (1 - alpha) * filteredValue
 * alpha越接近1，平滑效果越弱，响应越快；alpha越接近0，平滑效果越强，响应越慢
 */
public class EMA {
    private double alpha;
    private double filteredValue;
    private boolean hasInitialValue;

    /**
     * 构造EMA滤波器
     *
     * @param alpha 平滑系数，范围(0, 1]
     *              alpha = 2 / (n + 1) 对应等效窗口大小为n的移动平均
     */
    public EMA(double alpha) {
        if (alpha <= 0 || alpha > 1) {
            throw new IllegalArgumentException("alpha must be in range (0, 1]");
        }
        this.alpha = alpha;
        this.filteredValue = 0.0;
        this.hasInitialValue = false;
    }

    /**
     * 设置新的alpha值
     *
     * @param alpha 平滑系数，范围(0, 1]
     */
    public void setAlpha(double alpha) {
        if (alpha <= 0 || alpha > 1) {
            throw new IllegalArgumentException("alpha must be in range (0, 1]");
        }
        this.alpha = alpha;
    }

    /**
     * 根据等效窗口大小创建EMA滤波器
     * alpha = 2.0 / (n + 1)，使得EMA的时间常数与n阶移动平均相当
     *
     * @param n 等效窗口大小，必须大于0
     * @return EMA滤波器实例
     */
    public static EMA fromWindow(int n) {
        if (n <= 0) {
            throw new IllegalArgumentException("window size must be > 0");
        }
        return new EMA(2.0 / (n + 1));
    }

    /**
     * 输入原始值x，返回滤波后的值
     * 首次调用时直接返回x作为初始值
     *
     * @param x 原始输入值
     * @return 滤波后的值
     */
    public double update(double x) {
        if (!Double.isNaN(x) && Double.isFinite(x)) {
            if (!hasInitialValue) {
                filteredValue = x;
                hasInitialValue = true;
            } else {
                filteredValue = alpha * x + (1 - alpha) * filteredValue;
            }
        }
        return filteredValue;
    }

    /**
     * 重置滤波器，清空历史状态
     */
    public void reset() {
        filteredValue = 0.0;
        hasInitialValue = false;
    }

    /**
     * 获取当前滤波后的值（不添加新样本）
     *
     * @return 当前滤波值
     */
    public double getFilteredValue() {
        return filteredValue;
    }

    /**
     * 获取当前alpha值
     *
     * @return 平滑系数alpha
     */
    public double getAlpha() {
        return alpha;
    }

    /**
     * 判断是否已有初始值
     *
     * @return true表示已接收过至少一个有效样本
     */
    public boolean hasInitialValue() {
        return hasInitialValue;
    }
}