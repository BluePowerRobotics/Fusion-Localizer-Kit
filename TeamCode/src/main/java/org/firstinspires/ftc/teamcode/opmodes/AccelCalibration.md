# AccelCalibration —— BNO055 (Rev9axisIMU) 校准说明

本文档详细说明 [AccelCalibration.java](AccelCalibration.java) 的校准方法、校准文件内容与保存位置。
该 OpMode 参照官方示例 `SensorBNO055IMUCalibration`，通过 `Rev9axisIMU` 封装对设备名 `accel` 的 BNO055 进行手动校准并持久化保存。

---

## 1. 概述

- **设备**：BNO055（9 轴 IMU），在机器人控制器配置中命名为 `accel`，类型为 `BNO055IMU`。
- **运行模式**：融合模式（NDOF，由 `JustLoggingAccelerationIntegrator` 触发）。
  因此 `getAcceleration()` 返回**剔除重力**的线性加速度；同时 NDOF 使用磁力计，磁力计也可被校准。
- **作用**：校准数据保存后，`Rev9axisIMU.initialize()` 每次上电自动加载，缩短自校准时间并提高航向/姿态精度。
- **参考**：官方 [SensorBNO055IMUCalibration.java](../../../../../../../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorBNO055IMUCalibration.java)。

> **重要结论（官方说明）**：手动校准**并非必须**——陀螺仪与加速度计开机后会内部自校准。
> 手动校准 + 保存的价值在于：① 缩短每次上电的自校准时间；② 校准磁力计（本项目 NDOF 模式会用到）。

---

## 2. 前置条件

| 项 | 要求 |
|---|---|
| 设备配置 | 机器人控制器配置中必须有名为 `accel`、类型 `BNO055IMU` 的设备 |
| 接线 | BNO055 正常连接（I2C），上电后驱动板检测到设备 |
| 校准环境 | 远离强磁场（大功率电机、电调、磁铁），金属桌面也可能干扰磁力计 |
| 操作人员 | 准备一个手柄，用于按 A 键保存 |

---

## 3. 使用步骤

1. **运行** `Accel Calibration (BNO055)`（group=Fusion）。
2. **初始化阶段**：OpMode 自动执行 `new Rev9axisIMU(hardwareMap, "accel")` + `initialize()`。
   若初始化失败，日志区会显示“BNO055 初始化失败”的警告。
3. **按 START** 进入循环，观察 telemetry：
   - `status`：系统状态（如 `RUNNING_FUSION`）
   - `calib`：`SYS/ACC/MAG/GYR` 四项校准状态（每项 0~3）
   - `heading / roll / pitch`：姿态角
   - `linAccel`：剔除重力的线性加速度（便于确认轴符号）
4. **按 `calib` 显示的动作执行校准**（见第 4 节），直到四项均达到 **3**。
5. **按手柄 A 键**：读取当前校准数据并序列化保存到文件，日志区显示“已保存校准数据到 'BNO055IMUCalibration.json'”。
   按键释放前不会重复保存。
6. 校准完成，停止 OpMode。

---

## 4. 校准动作详解（Bosch BNO055 数据手册 §3.11）

校准状态寄存器返回四个子项的校准状态，各取值 **0~3**：

| 值 | 含义 |
|---|---|
| 0 | 未校准 |
| 1 | 校准不足 |
| 2 | 可接受 |
| 3 | 完全校准（最佳） |

| 子项 | 含义 | 校准动作 |
|---|---|---|
| **GYR** | 陀螺仪 | 让设备**静置平放几秒**即可（通常开机后自动完成） |
| **ACC** | 加速度计 | 缓慢旋转到**各种姿态**：每 45° 停几秒再继续；至少一次让设备分别**垂直于 x、y、z 各轴**摆放（例如正放、倒放、立放、侧立各保持几秒），6 次以上移动 |
| **MAG** | 磁力计 | 在空中**缓慢画“8”字**，直到 MAG 达到 3 |
| **SYS** | 系统 | 其余三项到 3 后通常自动到 3；若未到，继续缓慢绕各轴移动设备直至达标 |

> 提示：MAG 是通常最慢的一项。请慢速、匀速画 8 字，并让设备在运动中尽量旋转各个朝向。

---

## 5. 校准文件内容

保存的是 `BNO055IMU.CalibrationData` 序列化后的 **JSON 文本**，字段如下：

| 字段 | 含义 |
|---|---|
| `calibrationTime` | 校准耗时/时间戳 |
| `dx, dy, dz` | 加速度计校准偏移相关 |
| `radius` | 加速度计校准半径 |
| `accelOffset_x / _y / _z` | 加速度计各轴偏移 |
| `gyroOffset_x / _y / _z` | 陀螺仪各轴偏移 |
| `magOffset_x / _y / _z` | 磁力计各轴偏移 |
| `magRadius` | 磁力计校准半径 |

示例（示意，实际数值因设备而异）：

```json
{
  "calibrationTime": 12.34,
  "dx": 0.01, "dy": -0.02, "dz": 0.03,
  "radius": 1.01,
  "accelOffset_x": 24.0, "accelOffset_y": -16.0, "accelOffset_z": 8.0,
  "gyroOffset_x": 0.5, "gyroOffset_y": -0.3, "gyroOffset_z": 0.7,
  "magOffset_x": 150.0, "magOffset_y": -80.0, "magOffset_z": 40.0,
  "magRadius": 650.0
}
```

> 该文件即 BNO055 内部的校准寄存器快照。BNO055 的校准寄存器**断电即丢失**，因此需要持久化为文件。

---

## 6. 保存位置

由 [AccelCalibration.java:82](AccelCalibration.java#L82) 的
`AppUtil.getInstance().getSettingsFile(CALIBRATION_FILE)` 决定，即**机器人控制器 App 的内部 settings 目录**：

- 目录：`AppUtil.getSettingsDirectory()`（Robot Controller App 私有存储，约
  `/data/data/org.firstinspires.ftc.robotcontroller/files/settings/`，具体以 SDK 实现为准）
- 完整文件：`.../settings/BNO055IMUCalibration.json`
- **普通文件管理器不可见**（不在 `/sdcard` 公开目录下）

**查看 / 备份 / 迁移**：
- Android Studio → Device File Explorer（连接 RC 后浏览 App 私有目录）
- 或 `adb shell` + `run-as org.firstinspires.ftc.robotcontroller`（需可调试）
- 建议把校准文件备份到电脑；更换 RC 手机/平板后重新校准（每台设备的磁力计偏移不同）。

---

## 7. 自动加载机制

`Rev9axisIMU.initialize()` 中设置：

```java
params.calibrationDataFile = "BNO055IMUCalibration.json";
```

因此每次 OpMode 初始化 BNO055 时，若 `settings` 目录下存在该文件，SDK 会自动
`deserialize` 并写入 BNO055 校准寄存器，**跳过漫长的自校准等待**。

> **文件名一致性**：保存文件名（`BNO055IMUCalibration.json`）必须与
> `Rev9axisIMU.initialize()` 的 `calibrationDataFile` 完全一致，否则保存了也不会被加载。
> 官方示例默认保存为 `AdafruitIMUCalibration.json`，本项目为自洽已统一为 `BNO055IMUCalibration.json`。

---

## 8. 注意事项 / FAQ

- **每台 RC 校准一次**：校准文件存在 RC 手机/平板上，不随项目代码分发；换设备需重新校准。
- **磁干扰**：磁力计校准最易受干扰。校准和比赛时都要远离电机、电调、钕磁铁；若机器人在比赛中挪动方位或磁场环境变化，航向可能漂移。
- **无需频繁重校**：校准数据写入 BNO055 非易失性寄存器前只是临时值，**文件保存后即可长期使用**；仅在更换安装位置或设备损坏时重校。
- **“未做校准也能用”**：即使不校准，BNO055 也会自动校准陀螺/加速度计；但 NDOF 航向依赖磁力计，未校准磁力计可能导致航向缓慢漂移。
- **多 IMU**：若机器人有多个 BNO055，需为每个设备用不同文件名（本项目仅 `accel` 一个）。
