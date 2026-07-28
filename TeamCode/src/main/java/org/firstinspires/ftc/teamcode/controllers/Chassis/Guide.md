# Chassis 底盘控制器使用指南

## 概述

`Controllers/Chassis` 目录提供简单的麦轮底盘控制器及测试工具。底盘通过 `update(vx, vy, omega)` 手动控制速度，不依赖RoadRunner。

---

## 类说明

### 1. Chassis

简单的麦轮（Mecanum）底盘控制器，直接控制四个电机。

```java
// 创建底盘（默认 maxV=1.0, maxOmega=PI）
Chassis chassis = new Chassis(hardwareMap);

// 或指定最大速度
Chassis chassis = new Chassis(hardwareMap, maxV, maxOmega);

// 每帧更新底盘速度
chassis.update(vx, vy, omega);

// 停止所有电机
chassis.stop();
```

**电机配置：**
- `fL`: 左前电机
- `fR`: 右前电机
- `bL`: 左后电机
- `bR`: 右后电机

**参数说明：**
- `vx`: 前进速度（-1~1），正值前进
- `vy`: 横向速度（-1~1），正值左移
- `omega`: 角速度（-1~1），正值逆时针旋转
- `maxV`: 最大线速度（m/s），默认1.0
- `maxOmega`: 最大角速度（rad/s），默认PI

**内置功率归一化：** 当任一电机功率超过1时，自动等比例缩放所有电机功率。

### 2. ChassisTester

底盘测试OpMode，用于手动测试底盘驱动功能。

```java
@TeleOp(name = "ChassisTester", group = "Tests")
public class ChassisTester extends LinearOpMode { ... }
```

**操作方式：**
- 左摇杆：控制前后左右移动
  - 上/下：前进/后退
  - 左/右：左移/右移
- 右摇杆左右：控制旋转

**Telemetry显示：**
- vx, vy, omega 输入值
- 四个电机的实际功率

### 3. ChassisTuner

底盘调谐OpMode，用于测试单个电机和编码器。

```java
@TeleOp(name = "ChassisTuner", group = "Tests")
public class ChassisTuner extends LinearOpMode { ... }
```

**操作方式：**
- 按A键：所有电机前进2000 ticks（编码器位置模式）
- 按B键：所有电机以0.3功率持续转动

**Telemetry显示：**
- 四个电机的编码器位置
- 运动完成状态

---

## 典型用法

### 在TeleOp中使用

```java
@TeleOp(name = "MyTeleOp", group = "Main")
public class MyTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Chassis chassis = new Chassis(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            // 摇杆控制
            double vx = -gamepad1.left_stick_y;   // 前进/后退
            double vy = -gamepad1.left_stick_x;    // 左移/右移
            double omega = -gamepad1.right_stick_x; // 旋转

            chassis.update(vx, vy, omega);
        }
    }
}
```

### 在Autonomous中使用

```java
// 前进
chassis.update(0.5, 0, 0);
sleep(1000);
chassis.stop();

// 左移
chassis.update(0, 0.5, 0);
sleep(1000);
chassis.stop();

// 旋转
chassis.update(0, 0, 0.5);
sleep(1000);
chassis.stop();
```