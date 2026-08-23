package org.firstinspires.ftc.teamcode.processors.Accelerometer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * DFRobot Gravity LIS2DW12 三轴加速度传感器 (I2C 版) 驱动。
 *
 * <p>该驱动封装了对 {@link I2cDeviceSynch} 的底层寄存器读写，
 * 参考了 ST 官方 {@code LIS2DW12 datasheet} 与 DFRobot Arduino 库的实现思路。
 *
 * <p><b>硬件连接</b>：将传感器的 VCC / GND / SCL / SDA 四根线接到
 * REV Control / Expansion Hub 的 I2C 端口。Gravity 版本使用标准 4 针
 * JST PH 2.0 连接器，供电支持 3.3V ~ 5V。
 *
 * <p><b>I2C 地址</b>：默认 7-bit 地址 <b>0x19</b>，可通过板载拨钮开关切换为
 * <b>0x18</b>。若修改了开关，请使用带地址参数的构造器。
 *
 * <p><b>注意</b>：Gravity 版本未引出可编程中断引脚 (INT1/INT2)，因此本驱动
 * 仅支持轮询读取 (polling)，无法使用自由落体 / 点击等硬件中断。如需这些
 * 功能请购买 Breakout 版本。
 *
 * <p><b>朝向配置</b>：默认采用机器人相对坐标系 {X 前 / Y 左 / Z 上}。若传感器
 * 实际安装方向与之不符，可调用
 * {@link #setOrientation(FacingDirection, FacingDirection, FacingDirection)} 或
 * {@link #setOrientation(Parameters)} 指定传感器 +X / +Y / +Z 各物理轴在机器人上
 * 指向的方向（命名风格与官方 Hub IMU 的 logo/usb 朝向一致），之后
 * getX/Y/ZAcceleration 将以机器人相对坐标系输出。
 *
 * <p>典型用法：
 * <pre>{@code
 *   dfRobotAccelerometer accel = new dfRobotAccelerometer(hardwareMap, "accel");
 *   // 例如「Y 前 / Z 右 / X 下」安装时：
 *   accel.setOrientation(
 *           dfRobotAccelerometer.FacingDirection.DOWN,
 *           dfRobotAccelerometer.FacingDirection.FORWARD,
 *           dfRobotAccelerometer.FacingDirection.RIGHT);
 *   if (accel.initialize()) {
 *       accel.setRange(dfRobotAccelerometer.Range.RANGE_4G);
 *       accel.setDataRate(dfRobotAccelerometer.DataRate.RATE_100_HZ);
 *   }
 *   // 循环中:
 *   accel.readAccelerometer();
 *   double ax = accel.getXAcceleration();   // 单位: inch/s²
 * }</pre>
 */
@Config
public class dfRobotAccelerometer {

    // ==================== I2C 地址 ====================

    /** 默认 I2C 地址 (7-bit) */
    public static final int I2C_ADDRESS_DEFAULT = 0x19;
    /** 备用 I2C 地址 (7-bit)，拨钮开关切换后使用 */
    public static final int I2C_ADDRESS_ALT = 0x18;

    // ==================== 寄存器地址 ====================

    /** 芯片 ID 寄存器 */
    public static final int REG_WHO_AM_I = 0x0F;
    /** 控制寄存器 1: ODR / 工作模式 / 低功耗模式 */
    public static final int REG_CTRL1 = 0x20;
    /** 控制寄存器 2: BOOT / 软复位 / BDU 等 */
    public static final int REG_CTRL2 = 0x21;
    /** 控制寄存器 6: 带宽滤波 / 量程 FS / 低噪声 */
    public static final int REG_CTRL6 = 0x25;
    /** 8-bit 温度输出 (补齐后度数 = 原始值 + 25) */
    public static final int REG_OUT_T = 0x26;
    /** 状态寄存器 */
    public static final int REG_STATUS = 0x27;
    /** X 轴输出低字节 (连续读取 6 字节得到 X/Y/Z) */
    public static final int REG_OUT_X_L = 0x28;

    /** LIS2DW12 的 WHO_AM_I 期望值 */
    public static final int CHIP_ID = 0x44;

    // ==================== CTRL2 位掩码 ====================

    /** 块数据更新 (Block Data Update)：保证高/低字节属于同一次采样 */
    private static final int CTRL2_BDU = 0x08;
    /** 多字节读取时地址自动递增 */
    private static final int CTRL2_IF_ADD_INC = 0x04;
    /** 软复位 */
    private static final int CTRL2_SOFT_RESET = 0x40;

    // ==================== STATUS 位掩码 ====================

    /** 数据就绪标志 (Data Ready) */
    private static final int STATUS_DRDY = 0x01;

    // ==================== 量程枚举 ====================

    /**
     * 传感器量程 (满量程)。对应 CTRL6 的 FS[1:0] 位编码以及
     * 14-bit 分辨率下的灵敏度 (mg/LSB)。
     */
    public enum Range {
        /** ±2 g，灵敏度 0.244 mg/LSB */
        RANGE_2G(0x00, 0.244),
        /** ±4 g，灵敏度 0.488 mg/LSB */
        RANGE_4G(0x10, 0.488),
        /** ±8 g，灵敏度 0.976 mg/LSB */
        RANGE_8G(0x20, 0.976),
        /** ±16 g，灵敏度 1.952 mg/LSB */
        RANGE_16G(0x30, 1.952);

        final int ctrl6Value;
        final double sensitivityMg;

        Range(int ctrl6Value, double sensitivityMg) {
            this.ctrl6Value = ctrl6Value;
            this.sensitivityMg = sensitivityMg;
        }
    }

    // ==================== 输出数据速率枚举 ====================

    /**
     * 输出数据速率 (ODR)。{@code ctrl1Value} 为高性能模式
     * (MODE=01, LP_MODE=00) 下写入 CTRL1 的完整字节。
     */
    public enum DataRate {
        RATE_12_5_HZ(0x14, 12.5),
        RATE_25_HZ(0x34, 25.0),
        RATE_50_HZ(0x44, 50.0),
        RATE_100_HZ(0x54, 100.0),
        RATE_200_HZ(0x64, 200.0),
        RATE_400_HZ(0x74, 400.0),
        RATE_800_HZ(0x84, 800.0),
        RATE_1600_HZ(0xB4, 1600.0);

        final int ctrl1Value;
        final double hz;

        DataRate(int ctrl1Value, double hz) {
            this.ctrl1Value = ctrl1Value;
            this.hz = hz;
        }
    }

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
     * <p>三个字段分别描述传感器 +X / +Y / +Z 物理轴在机器人坐标系中指向的方向：
     * <ul>
     *   <li>{@code xFacing}: 传感器 +X 轴指向的方向（默认 {@link FacingDirection#FORWARD}）</li>
     *   <li>{@code yFacing}: 传感器 +Y 轴指向的方向（默认 {@link FacingDirection#LEFT}）</li>
     *   <li>{@code zFacing}: 传感器 +Z 轴指向的方向（默认 {@link FacingDirection#UP}）</li>
     * </ul>
     *
     * <p>默认值对应「X 前 / Y 左 / Z 上」的标准安装。例如「Y 前 / Z 左 / X 下」安装时，
     * 应设为 {@code yFacing = FORWARD, zFacing = LEFT, xFacing = DOWN}。
     */
    public static class Parameters {
        /** 传感器 +X 物理轴指向的机器人方向 */
        public FacingDirection xFacing = FacingDirection.FORWARD;
        /** 传感器 +Y 物理轴指向的机器人方向 */
        public FacingDirection yFacing = FacingDirection.LEFT;
        /** 传感器 +Z 物理轴指向的机器人方向 */
        public FacingDirection zFacing = FacingDirection.UP;
    }

    /** 全局朝向参数，可通过 FTC Dashboard (@Config) 实时调整 */
    public static Parameters PARAMS = new Parameters();

    // ==================== 常量 ====================

    /** 标准重力加速度 (m/s²)，用于将 g 换算为 m/s² */
    private static final double GRAVITY_MPS2 = 9.80665;
    /** 1 米对应的英寸数 */
    private static final double INCH_PER_METER = 39.37007874;
    /** 1 g 对应的加速度 (inch/s²)，用于将 g 换算为英制单位 */
    private static final double INCH_PER_SEC2_PER_G = GRAVITY_MPS2 * INCH_PER_METER;

    // ==================== 字段 ====================

    private I2cDeviceSynch deviceClient;

    /** 当前量程 */
    private Range range = Range.RANGE_2G;
    /** 当前数据速率 */
    private DataRate dataRate = DataRate.RATE_100_HZ;
    /** 当前灵敏度 (mg/LSB, 14-bit) */
    private double sensitivityMg = Range.RANGE_2G.sensitivityMg;

    /** 最近一次读取的原始数据 (16-bit 左对齐) */
    private short rawX;
    private short rawY;
    private short rawZ;

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

    // ==================== 构造 ====================

    /**
     * 使用默认 I2C 地址 (0x19) 构造。
     *
     * @param deviceClient 已由 HardwareMap 获取的 I2C 同步设备
     */
    public dfRobotAccelerometer(I2cDeviceSynch deviceClient) {
        this(deviceClient, I2C_ADDRESS_DEFAULT);
    }

    /**
     * @param deviceClient     已由 HardwareMap 获取的 I2C 同步设备
     * @param i2cAddress7Bit   7-bit I2C 地址 (0x19 或 0x18)
     */
    public dfRobotAccelerometer(I2cDeviceSynch deviceClient, int i2cAddress7Bit) {
        this.deviceClient = deviceClient;
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(i2cAddress7Bit));
        setOrientation(PARAMS);
    }

    /**
     * 从 HardwareMap 按设备名获取 I2C 设备，使用默认地址 0x19。
     *
     * @param hardwareMap 硬件映射
     * @param deviceName  I2C 设备在机器人控制器配置中的名称
     */
    public dfRobotAccelerometer(HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap.i2cDeviceSynch.get(deviceName));
    }

    /**
     * 从 HardwareMap 按设备名获取 I2C 设备，使用指定地址。
     *
     * @param hardwareMap     硬件映射
     * @param deviceName      I2C 设备在机器人控制器配置中的名称
     * @param i2cAddress7Bit  7-bit I2C 地址 (0x19 或 0x18)
     */
    public dfRobotAccelerometer(HardwareMap hardwareMap, String deviceName, int i2cAddress7Bit) {
        this(hardwareMap.i2cDeviceSynch.get(deviceName), i2cAddress7Bit);
    }

    // ==================== 初始化 ====================

    /**
     * 初始化传感器：软复位 → 校验 WHO_AM_I → 配置默认量程 / 速率 / BDU。
     * 应在 OpMode 的 {@code init()} 阶段调用一次。
     *
     * @return true 表示芯片 ID 校验通过，通信正常
     */
    public boolean initialize() {
        softReset();

        // 校验 WHO_AM_I
        if (readRegister(REG_WHO_AM_I) != CHIP_ID) {
            return false;
        }

        // BDU + 地址自动递增，保证高/低字节一致性
        writeRegister(REG_CTRL2, CTRL2_BDU | CTRL2_IF_ADD_INC);

        // 应用默认配置：高性能模式 + 量程 + 速率
        setRange(range);
        setDataRate(dataRate);
        return true;
    }

    /**
     * 检查传感器是否在线 (WHO_AM_I 是否匹配)。
     *
     * @return true 表示通信正常
     */
    public boolean isConnected() {
        return readRegister(REG_WHO_AM_I) == CHIP_ID;
    }

    /**
     * 软复位设备。内部会等待约 10 ms 让器件稳定。
     */
    public void softReset() {
        writeRegister(REG_CTRL2, CTRL2_SOFT_RESET);
        delayMs(10);
    }

    // ==================== 底层读写 ====================

    /**
     * 写单个寄存器。
     *
     * @param reg  寄存器地址
     * @param data 要写入的字节 (0~255)
     */
    public void writeRegister(int reg, int data) {
        deviceClient.write8(reg, data);
    }

    /**
     * 读单个寄存器。
     *
     * @param reg 寄存器地址
     * @return 寄存器值 (0~255)
     */
    public int readRegister(int reg) {
        return deviceClient.read8(reg);
    }

    /**
     * 从指定寄存器开始连续读取多个字节 (地址自动递增需已开启)。
     *
     * @param reg   起始寄存器地址
     * @param count 读取字节数
     * @return 读取到的原始字节数组
     */
    public byte[] readRegisters(int reg, int count) {
        return deviceClient.read(reg, count);
    }

    // ==================== 配置 ====================

    /**
     * 设置量程，并同步更新灵敏度。
     *
     * @param range 量程枚举值
     */
    public void setRange(Range range) {
        this.range = range;
        this.sensitivityMg = range.sensitivityMg;
        writeRegister(REG_CTRL6, range.ctrl6Value);
    }

    /**
     * 按 g 数值设置量程 (仅支持 2 / 4 / 8 / 16)。
     *
     * @param rangeG 量程 (g)，如 2、4、8、16
     */
    public void setRange(int rangeG) {
        switch (rangeG) {
            case 2:
                setRange(Range.RANGE_2G);
                break;
            case 4:
                setRange(Range.RANGE_4G);
                break;
            case 8:
                setRange(Range.RANGE_8G);
                break;
            case 16:
                setRange(Range.RANGE_16G);
                break;
            default:
                throw new IllegalArgumentException("Unsupported range: " + rangeG + " g (2/4/8/16 only)");
        }
    }

    /**
     * 设置输出数据速率 (高性能模式)。
     *
     * @param dataRate 速率枚举值
     */
    public void setDataRate(DataRate dataRate) {
        this.dataRate = dataRate;
        writeRegister(REG_CTRL1, dataRate.ctrl1Value);
    }

    /** @return 当前量程 */
    public Range getRange() {
        return range;
    }

    /** @return 当前输出数据速率 */
    public DataRate getDataRate() {
        return dataRate;
    }

    // ==================== 朝向配置 ====================

    /**
     * 按机器人坐标系方向设置传感器安装朝向。
     *
     * @param xFacing 传感器 +X 物理轴指向的机器人方向
     * @param yFacing 传感器 +Y 物理轴指向的机器人方向
     * @param zFacing 传感器 +Z 物理轴指向的机器人方向
     */
    public void setOrientation(FacingDirection xFacing, FacingDirection yFacing, FacingDirection zFacing) {
        xSrc  = facingIndex(xFacing, yFacing, zFacing, FacingDirection.FORWARD);
        xSign = facingSign(xFacing, yFacing, zFacing, FacingDirection.FORWARD);
        ySrc  = facingIndex(xFacing, yFacing, zFacing, FacingDirection.LEFT);
        ySign = facingSign(xFacing, yFacing, zFacing, FacingDirection.LEFT);
        zSrc  = facingIndex(xFacing, yFacing, zFacing, FacingDirection.UP);
        zSign = facingSign(xFacing, yFacing, zFacing, FacingDirection.UP);
    }

    /**
     * 按 {@link Parameters} 设置传感器安装朝向。
     */
    public void setOrientation(Parameters params) {
        setOrientation(params.xFacing, params.yFacing, params.zFacing);
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
     * 从输出寄存器读取原始 X/Y/Z 数据 (一次读取 6 字节)。
     * 每帧循环中调用一次，随后通过 getter 获取换算后的加速度。
     */
    public void readAccelerometer() {
        byte[] data = readRegisters(REG_OUT_X_L, 6);
        rawX = composeInt16(data, 0);
        rawY = composeInt16(data, 2);
        rawZ = composeInt16(data, 4);
    }

    /**
     * 检查是否有新的加速度数据就绪 (STATUS 寄存器 DRDY 位)。
     *
     * @return true 表示新数据已就绪
     */
    public boolean isDataReady() {
        return (readRegister(REG_STATUS) & STATUS_DRDY) != 0;
    }

    // ==================== 加速度输出 (机器人相对坐标系, 默认单位: inch/s²) ====================

    /** @return 机器人 X (前进) 轴加速度，单位 inch/s² */
    public double getXAcceleration() {
        return toRobotG(xSrc, xSign) * INCH_PER_SEC2_PER_G;
    }

    /** @return 机器人 Y (左) 轴加速度，单位 inch/s² */
    public double getYAcceleration() {
        return toRobotG(ySrc, ySign) * INCH_PER_SEC2_PER_G;
    }

    /** @return 机器人 Z (上) 轴加速度，单位 inch/s² */
    public double getZAcceleration() {
        return toRobotG(zSrc, zSign) * INCH_PER_SEC2_PER_G;
    }

    // ==================== 加速度输出 (机器人相对坐标系, 单位: m/s²) ====================

    /** @return 机器人 X (前进) 轴加速度，单位 m/s² */
    public double getXAccelerationMps2() {
        return toRobotG(xSrc, xSign) * GRAVITY_MPS2;
    }

    /** @return 机器人 Y (左) 轴加速度，单位 m/s² */
    public double getYAccelerationMps2() {
        return toRobotG(ySrc, ySign) * GRAVITY_MPS2;
    }

    /** @return 机器人 Z (上) 轴加速度，单位 m/s² */
    public double getZAccelerationMps2() {
        return toRobotG(zSrc, zSign) * GRAVITY_MPS2;
    }

    // ==================== 原始数据 ====================

    /** @return X 轴原始 16-bit 数据 (左对齐) */
    public short getRawX() {
        return rawX;
    }

    /** @return Y 轴原始 16-bit 数据 (左对齐) */
    public short getRawY() {
        return rawY;
    }

    /** @return Z 轴原始 16-bit 数据 (左对齐) */
    public short getRawZ() {
        return rawZ;
    }

    // ==================== 温度 ====================

    /**
     * 读取片上温度传感器。
     *
     * @return 温度，单位 °C
     */
    public double getTemperatureCelsius() {
        byte t = (byte) readRegister(REG_OUT_T);
        return t + 25.0;
    }

    // ==================== 内部工具 ====================

    /**
     * 将 16-bit 左对齐原始值换算为 g。
     *
     * <p>LIS2DW12 为 14-bit 分辨率的左对齐输出，16-bit 寄存器的低 2 bit
     * 恒为 0，因此先算术右移 2 位得到 14-bit 有符号值，再乘以灵敏度 (mg/LSB)
     * 并除以 1000 换算为 g。
     */
    private double toG(short raw) {
        int value14 = (raw >> 2);
        return value14 * sensitivityMg / 1000.0;
    }

    /**
     * 按朝向映射，将指定索引的传感器轴换算为机器人坐标系下的 g 值。
     *
     * @param srcIndex 传感器轴索引 (0=X, 1=Y, 2=Z)
     * @param sign     方向符号 (+1 / -1)
     */
    private double toRobotG(int srcIndex, int sign) {
        switch (srcIndex) {
            case 0:
                return sign * toG(rawX);
            case 1:
                return sign * toG(rawY);
            default:
                return sign * toG(rawZ);
        }
    }

    /**
     * 从字节数组中按小端序组合出一个有符号 16-bit 值。
     *
     * @param data   字节数组 (低字节在前)
     * @param offset 低字节所在下标
     */
    private static short composeInt16(byte[] data, int offset) {
        return (short) ((data[offset] & 0xFF) | (data[offset + 1] << 8));
    }

    /** 简单的毫秒延时，仅在初始化阶段使用。 */
    private static void delayMs(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}