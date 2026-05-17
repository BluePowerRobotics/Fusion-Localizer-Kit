# Algebra 代数计算工具

## 概述

本目录包含代数计算相关的工具类，主要用于方程求解和单位转换。

## 文件列表

| 文件 | 功能说明 |
|------|----------|
| `EquationSolver.java` | 多项式方程求解器，支持1-4阶方程求解 |

## EquationSolver 使用说明

### 功能特性

- 支持一元一次方程求解 (`solve1`)
- 支持一元二次方程求解 (`solve2`)
- 支持一元三次方程求解 (`solve3`)
- 支持一元四次方程求解 (`solve4`)
- 提供单位转换方法（毫米↔英寸）
- 提供平均值计算方法

### 使用示例

```java
import org.firstinspires.ftc.teamcode.utility.Algebra.EquationSolver;

// 求解二次方程: x^2 - 5x + 6 = 0
double[] roots = EquationSolver.solve2(1, -5, 6);
// 返回: [3.0, 2.0]

// 求解三次方程: x^3 - 6x^2 + 11x - 6 = 0
double[] cubicRoots = EquationSolver.solve3(1, -6, 11, -6);
// 返回: [1.0, 2.0, 3.0]

// 单位转换
double inches = EquationSolver.toInch(100);  // 100mm -> 英寸
double mm = EquationSolver.toMM(4);          // 4英寸 -> mm

// 计算平均值
double avg = EquationSolver.avg(1.0, 2.0, 3.0, 4.0);
```

### 方法签名

| 方法 | 说明 | 返回值 |
|------|------|--------|
| `solve1(a, b)` | 求解 ax + b = 0 | `double[]` 根数组 |
| `solve2(a, b, c)` | 求解 ax² + bx + c = 0 | `double[]` 根数组 |
| `solve3(a, b, c, d)` | 求解 ax³ + bx² + cx + d = 0 | `double[]` 根数组 |
| `solve4(a, b, c, d, e)` | 求解 ax⁴ + bx³ + cx² + dx + e = 0 | `double[]` 根数组 |
| `toInch(mm)` | 毫米转英寸 | `double` |
| `toMM(inch)` | 英寸转毫米 | `double` |
| `avg(doubles...)` | 计算平均值 | `double` |

### 注意事项

1. 返回的根数组可能为空（无实数解）
2. 高阶方程求解使用数值方法，可能存在精度误差
3. 系数为0时会自动降级到低阶方程求解