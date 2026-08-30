package org.firstinspires.ftc.teamcode.processors.Accelerometer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * REV 9-axis IMU (BNO055) 三轴加速度计封装。
 *
 * <p>功能参照 {@link dfRobotAccelerometer}：把 BNO055 封装为机器人相对坐标系
 * 三轴加速度计，提供统一的朝向配置（{@link FacingDirection} / {@link Parameters}）、
 * 初始化检测与英制/公制加速度输出。
 *
 * <p><b>与 dfRobot (LIS2DW12) 的关键差异</b>：
 * <ul>
 *   <li><b>线性加速度</b>：设置 {@link JustLoggingAccelerationIntegrator} 后 BNO055
 *       进入融合模式 (NDOF)，{@code getAcceleration()} 返回<b>已剔除重力</b>的线性加速度，
 *       可直接作为惯导/死推的控制输入（对应 {@code Theory5D.md} 对加速度计的要求），
 *       而 LIS2DW12 返回的是包含重力的总加速度；</li>
 *   <li><b>量程 / 数据速率</b>：BNO055 内部固定（融合模式约 100 Hz、±8 g），
 *       无法经标准 API 配置，故不提供 {@code setRange / setDataRate}；</li>
 *   <li><b>原始 LSB / 温度</b>：标准 API 未暴露原始寄存器值与温度，故不提供
 *       {@code getRawX/Y/Z / getTemperatureCelsius}；</li>
 *   <li><b>I2C 地址</b>：BNO055 默认 0x28；本封装在 {@link #initialize()} 中显式指定为
 *       0x29（对应 ADD/ADR 引脚接高的 REV 9-axis IMU）。</li>
 * </ul>
 *
 * <p><b>朝向配置</b>：默认采用机器人相对坐标系 {X 前 / Y 左 / Z 上}（对应 BNO055
 * 三个物理轴按该朝向安装）。若实际安装方向不符，可调用
 * {@link #setOrientationXY(FacingDirection, FacingDirection)} 或
 * {@link #setOrientation(Parameters)} 仅指定传感器 +X / +Y 物理轴在机器人上
 * 指向的方向，+Z 轴方向按右手系自动推算（命名风格与官方 Hub IMU 的 logo/usb
 * 朝向一致）。<b>加速度轴符号依赖实际安装朝向，需实车静置/倾斜标定确认。</b>
 *
 * <p>典型用法：
 * <pre>{@code
 *   Rev9axisIMU accel = new Rev9axisIMU(hardwareMap, "accel");
 *   // 例如「Y 前 / Z 右 / X 下」安装时：只需给出 +X 与 +Y，+Z 自动推算为 RIGHT
 *   accel.setOrientationXY(
 *           Rev9axisIMU.FacingDirection.DOWN,
 *           Rev9axisIMU.FacingDirection.FORWARD);
 *   boolean ready = accel.initialize();   // BNO055 初始化 + 启动融合模式
 *   // 循环中:
 *   accel.readAccelerometer();
 *   double ax = accel.getXAcceleration(); // 单位: inch/s² (剔除重力)
 * }</pre>
 */
@Config
public class Rev9axisIMU {

    // ==================== 常量 ====================

    /** 1 米对应的英寸数 */
    private static final double INCH_PER_METER = 39.37007874;

    // ==================== 朝向 (坐标系映射) ====================

    /**
     * 机器人坐标系方向，与官方 Hub IMU 的
     * {@code RevHubOrientationOnRobot.LogoFacingDirection} / {@code UsbFacingDirection}
     * 命名风格一致，用于描述传感器某个物理轴在机器人上指向的方向。
     */
    public enum FacingDirection {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        UP,
        DOWN;

        /** @return 该方向的相反方向 */
        public FacingDirection opposite() {
            switch (this) {
                case FORWARD:  return BACKWARD;
                case BACKWARD: return FORWARD;
                case LEFT:     return RIGHT;
                case RIGHT:    return LEFT;
                case UP:       return DOWN;
                default:       return UP;
            }
        }
    }

    /**
     * 加速度计安装朝向参数，用法类似 {@code MecanumDrive.Params} 中 IMU 的朝向字段。
     *
     * <p>仅需指定传感器 +X / +Y 物理轴在机器人坐标系中指向的方向，+Z 轴按右手系
     * 自动推算。默认值对应「X 前 / Y 左 / Z 上」的标准安装。
     */
    public static class Parameters {
        /** 传感器 +X 物理轴指向的机器人方向 */
        public FacingDirection xFacing = FacingDirection.FORWARD;
        /** 传感器 +Y 物理轴指向的机器人方向 */
        public FacingDirection yFacing = FacingDirection.LEFT;
    }

    /** 全局朝向参数，可通过 FTC Dashboard (@Config) 实时调整 */
    public static Parameters PARAMS = new Parameters();

    // ==================== 字段 ====================

    private final BNO055IMU imu;

    /** 是否初始化成功（BNO055 在线且已进入融合模式） */
    private boolean ready = false;

    // ----- 朝向映射 (默认: X 前 / Y 左 / Z 上，即恒等映射) -----
    /** 机器人 X (前进) 对应的传感器轴索引 (0=X, 1=Y, 2=Z) */
    private int xSrc = 0;
    private int xSign = 1;
    /** 机器人 Y (左) 对应的传感器轴索引 */
    private int ySrc = 1;
    private int ySign = 1;
    /** 机器人 Z (上) 对应的传感器轴索引 */
    private int zSrc = 2;
    private int zSign = 1;

    /** 最近一次读取的线性加速度 (机器人坐标系, inch/s², 已剔除重力) */
    private double axInch;
    private double ayInch;
    private double azInch;

    // ==================== 构造 ====================

    /**
     * 从 HardwareMap 按设备名获取 BNO055 (配置为 BNO055IMU 类型)。
     *
     * @param hardwareMap 硬件映射
     * @param deviceName  BNO055 在机器人控制器配置中的名称
     */
    public Rev9axisIMU(HardwareMap hardwareMap, String deviceName) {
        this.imu = hardwareMap.get(BNO055IMU.class, deviceName);
        setOrientation(PARAMS);
    }

    /**
     * 从 HardwareMap 按设备名获取 BNO055，并指定安装朝向。
     *
     * @param hardwareMap 硬件映射
     * @param deviceName  BNO055 在机器人控制器配置中的名称
     * @param params      安装朝向参数
     */
    public Rev9axisIMU(HardwareMap hardwareMap, String deviceName, Parameters params) {
        this.imu = hardwareMap.get(BNO055IMU.class, deviceName);
        setOrientation(params);
    }

    // ==================== 初始化 ====================

    /**
     * 初始化 BNO055 并启动融合模式 (NDOF)。
     *
     * <p>参照官方 SensorBNO055IMU 示例：设置 {@link JustLoggingAccelerationIntegrator}
     * 会启用融合模式，使 {@code getAcceleration()} 返回剔除重力后的线性加速度。
     * 应在 OpMode 的 {@code init()} 阶段调用一次。
     *
     * @return true 表示初始化成功
     */
    public boolean initialize() {
        try {
            BNO055IMU.Parameters params = new BNO055IMU.Parameters();
            // BNO055 默认扫描 I2C 地址 0x28；本设备 ADD/ADR 引脚接高，地址为 0x29，需显式指定
            params.i2cAddr = I2cAddr.create8bit(0x29);
            params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            // 使用米制加速度；读取后在机器人坐标系换算为英制
            params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            params.calibrationDataFile = "BNO055IMUCalibration.json"; // 参见校准示例
            params.loggingEnabled = true;
            params.loggingTag = "AccelIMU";
            // 启用融合模式(NDOF)，使 getAcceleration() 返回剔除重力后的线性加速度
            params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            // 校验 BNO055 实际初始化结果，避免初始化失败仍标记为就绪
            boolean initializedOk = imu.initialize(params);
            if (initializedOk) {
                imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                ready = true;
            } else {
                ready = false;
            }
        } catch (Exception e) {
            ready = false;
        }
        return ready;
    }

    /**
     * 检查加速度计是否在线且已初始化成功。
     *
     * @return true 表示可用
     */
    public boolean isConnected() {
        return ready;
    }

    // ==================== 朝向配置 ====================

    /**
     * 按机器人坐标系方向设置传感器安装朝向。
     *
     * <p>仅需指定传感器 +X / +Y 物理轴在机器人上指向的方向，+Z 轴方向按右手系
     * 自动推算（Z = X × Y）。
     *
     * @param xFacing 传感器 +X 物理轴指向的机器人方向
     * @param yFacing 传感器 +Y 物理轴指向的机器人方向
     */
    public void setOrientationXY(FacingDirection xFacing, FacingDirection yFacing) {
        applyOrientation(xFacing, yFacing, computeZFacing(xFacing, yFacing));
    }

    /**
     * 按 {@link Parameters} 设置传感器安装朝向（仅 x / y，z 自动推算）。
     */
    public void setOrientation(Parameters params) {
        setOrientationXY(params.xFacing, params.yFacing);
    }

    /** 核心映射：记录三轴各自对应的传感器轴索引与符号。 */
    private void applyOrientation(FacingDirection xFacing, FacingDirection yFacing, FacingDirection zFacing) {
        xSrc  = facingIndex(xFacing, yFacing, zFacing, FacingDirection.FORWARD);
        xSign = facingSign(xFacing, yFacing, zFacing, FacingDirection.FORWARD);
        ySrc  = facingIndex(xFacing, yFacing, zFacing, FacingDirection.LEFT);
        ySign = facingSign(xFacing, yFacing, zFacing, FacingDirection.LEFT);
        zSrc  = facingIndex(xFacing, yFacing, zFacing, FacingDirection.UP);
        zSign = facingSign(xFacing, yFacing, zFacing, FacingDirection.UP);
    }

    /**
     * 由 x / y 轴朝向按右手系推算 z 轴朝向：Z = X × Y。
     *
     * @throws IllegalArgumentException x 与 y 轴平行（叉积为零），无法构成右手坐标系
     */
    private static FacingDirection computeZFacing(FacingDirection xFacing, FacingDirection yFacing) {
        int[] vx = directionVector(xFacing);
        int[] vy = directionVector(yFacing);
        int[] vz = {
                vx[1] * vy[2] - vx[2] * vy[1],
                vx[2] * vy[0] - vx[0] * vy[2],
                vx[0] * vy[1] - vx[1] * vy[0]
        };
        if (vz[0] == 0 && vz[1] == 0 && vz[2] == 0) {
            throw new IllegalArgumentException("x 与 y 轴朝向平行, 无法构成右手坐标系");
        }
        return facingFromVector(vz);
    }

    /** 机器人坐标系方向 → 单位向量。 */
    private static int[] directionVector(FacingDirection dir) {
        switch (dir) {
            case FORWARD:  return new int[]{ 1,  0,  0};
            case BACKWARD: return new int[]{-1,  0,  0};
            case LEFT:     return new int[]{ 0,  1,  0};
            case RIGHT:    return new int[]{ 0, -1,  0};
            case UP:       return new int[]{ 0,  0,  1};
            default:       return new int[]{ 0,  0, -1}; // DOWN
        }
    }

    /** 单位向量 → 机器人坐标系方向。 */
    private static FacingDirection facingFromVector(int[] v) {
        if (v[0] == 1) return FacingDirection.FORWARD;
        if (v[0] == -1) return FacingDirection.BACKWARD;
        if (v[1] == 1) return FacingDirection.LEFT;
        if (v[1] == -1) return FacingDirection.RIGHT;
        if (v[2] == 1) return FacingDirection.UP;
        return FacingDirection.DOWN;
    }

    /** 在三个传感器轴中，找出指向目标机器人方向 {@code dir}（或其相反方向）的轴索引。 */
    private static int facingIndex(FacingDirection xf, FacingDirection yf, FacingDirection zf,
                                   FacingDirection dir) {
        if (xf == dir || xf == dir.opposite()) return 0;
        if (yf == dir || yf == dir.opposite()) return 1;
        return 2;
    }

    /** 返回朝向目标机器人方向 {@code dir} 时的符号：正方向 +1，相反方向 -1。 */
    private static int facingSign(FacingDirection xf, FacingDirection yf, FacingDirection zf,
                                  FacingDirection dir) {
        if (xf == dir) return 1;
        if (xf == dir.opposite()) return -1;
        if (yf == dir) return 1;
        if (yf == dir.opposite()) return -1;
        if (zf == dir) return 1;
        return -1;
    }

    // ==================== 数据读取 ====================

    /**
     * 从 BNO055 读取融合线性加速度 (剔除重力)，并按朝向映射到机器人坐标系。
     * 每帧循环中调用一次，随后通过 getter 获取换算后的加速度。
     * 若读取失败则保留上一次的值。
     */
    public void readAccelerometer() {
        Acceleration a = imu.getAcceleration();
        if (a == null) {
            return;
        }
        double sx = a.xAccel; // 传感器坐标系线性加速度 (m/s²)
        double sy = a.yAccel;
        double sz = a.zAccel;
        axInch = axisComponent(sx, sy, sz, xSrc, xSign) * INCH_PER_METER;
        ayInch = axisComponent(sx, sy, sz, ySrc, ySign) * INCH_PER_METER;
        azInch = axisComponent(sx, sy, sz, zSrc, zSign) * INCH_PER_METER;
    }

    /** 取指定传感器轴分量并乘以符号。 */
    private static double axisComponent(double sx, double sy, double sz, int srcIndex, int sign) {
        double v;
        switch (srcIndex) {
            case 0:  v = sx; break;
            case 1:  v = sy; break;
            default: v = sz; break;
        }
        return sign * v;
    }

    // ==================== 加速度输出 (机器人相对坐标系, 默认单位: inch/s²) ====================

    /** @return 机器人 X (前进) 轴线性加速度 (剔除重力)，单位 inch/s² */
    public double getXAcceleration() {
        return axInch;
    }

    /** @return 机器人 Y (左) 轴线性加速度 (剔除重力)，单位 inch/s² */
    public double getYAcceleration() {
        return ayInch;
    }

    /** @return 机器人 Z (上) 轴线性加速度 (剔除重力)，单位 inch/s² */
    public double getZAcceleration() {
        return azInch;
    }

    // ==================== 加速度输出 (机器人相对坐标系, 单位: m/s²) ====================

    /** @return 机器人 X (前进) 轴线性加速度 (剔除重力)，单位 m/s² */
    public double getXAccelerationMps2() {
        return axInch / INCH_PER_METER;
    }

    /** @return 机器人 Y (左) 轴线性加速度 (剔除重力)，单位 m/s² */
    public double getYAccelerationMps2() {
        return ayInch / INCH_PER_METER;
    }

    /** @return 机器人 Z (上) 轴线性加速度 (剔除重力)，单位 m/s² */
    public double getZAccelerationMps2() {
        return azInch / INCH_PER_METER;
    }

    // ==================== 输出 / 调试 ====================

    /** @return 底层 BNO055IMU 实例（用于底层访问/诊断） */
    public BNO055IMU getAccelerometer() {
        return imu;
    }
}
