package org.firstinspires.ftc.teamcode.utility.Algebra;

public class EquationSolver {
    private static final double EPSILON = 1e-10;
    private static final double IMAG_THRESHOLD = 1e-6;
    private static final double PI = Math.PI;

    public static double sgn(double n) {
        return Math.signum(n);
    }

    public static double[] solve4(double a, double b, double c, double d, double e) {
        if (Math.abs(a) < EPSILON) {
            return solve3(b, c, d, e);
        }

        double D = 3 * b * b - 8 * a * c;
        double E = -b * b * b + 4 * a * b * c - 8 * a * a * d;
        double F = 3 * Math.pow(b, 4) + 16 * a * a * c * c - 16 * a * b * b * c + 16 * a * a * b * d - 64 * Math.pow(a, 3) * e;
        double A = D * D - 3 * F;
        double B = D * F - 9 * E * E;
        double C = F * F - 3 * D * E * E;
        double delta = B * B - 4 * A * C;

        if (Math.abs(D) < EPSILON && Math.abs(E) < EPSILON && Math.abs(F) < EPSILON) {
            return new double[]{-b / (4 * a)};
        }

        if (Math.abs(A) < EPSILON && Math.abs(B) < EPSILON && Math.abs(C) < EPSILON && (Math.abs(D) >= EPSILON || Math.abs(E) >= EPSILON || Math.abs(F) >= EPSILON)) {
            double x1 = (-b * D + 9 * E) / (4 * a * D);
            double x2 = (-b * D - 3 * E) / (4 * a * D);
            return new double[]{x1, x2};
        }

        if (Math.abs(E) < EPSILON && Math.abs(F) < EPSILON && Math.abs(D) >= EPSILON) {
            if (D > 0) {
                double sqrtD = Math.sqrt(D);
                return new double[]{(-b + sqrtD) / (4 * a), (-b - sqrtD) / (4 * a)};
            }
            return new double[]{};
        }

        if (Math.abs(delta) < EPSILON && A * B > 0) {
            double x1 = (-b + 2 * A * E / B + Math.sqrt(2 * B / A)) / (4 * a);
            double x2 = (-b + 2 * A * E / B - Math.sqrt(2 * B / A)) / (4 * a);
            double x3 = (-b - 2 * A * E / B) / (4 * a);
            return new double[]{x1, x2, x3};
        }

        if (delta > 0) {
            double sqrtDelta = Math.sqrt(delta);
            double z1 = A * D + 3 * ((-B + sqrtDelta) / 2);
            double z2 = A * D + 3 * ((-B - sqrtDelta) / 2);
            double cbrtZ1 = Math.cbrt(z1);
            double cbrtZ2 = Math.cbrt(z2);
            double z = D * D - D * (cbrtZ1 + cbrtZ2) + (cbrtZ1 + cbrtZ2) * (cbrtZ1 + cbrtZ2) - 3 * A;

            double signE = sgn(E);
            double realPart = (-b + signE * Math.sqrt(Math.max((D + cbrtZ1 + cbrtZ2) / 3, 0))) / (4 * a);
            double imagPart = Math.sqrt((2 * D - (cbrtZ1 + cbrtZ2) + 2 * Math.sqrt(z)) / 3) / (4 * a);

            double x1 = realPart + imagPart;
            double x2 = realPart - imagPart;
            return new double[]{x1, x2};
        }

        if (delta < 0 && D > 0 && F > 0) {
            double theta = Math.acos((3 * B - 2 * A * D) / (2 * A * Math.sqrt(A)));
            double y1 = (D - 2 * Math.sqrt(A) * Math.cos(theta / 3)) / 3;
            double y2 = (D + Math.sqrt(A) * (Math.cos(theta / 3) + Math.sqrt(3) * Math.sin(theta / 3))) / 3;
            double y3 = (D + Math.sqrt(A) * (Math.cos(theta / 3) - Math.sqrt(3) * Math.sin(theta / 3))) / 3;

            if (Math.abs(E) < EPSILON) {
                return new double[]{
                        (-b + Math.sqrt(D + 2 * Math.sqrt(F))) / (4 * a),
                        (-b - Math.sqrt(D + 2 * Math.sqrt(F))) / (4 * a),
                        (-b + Math.sqrt(D - 2 * Math.sqrt(F))) / (4 * a),
                        (-b - Math.sqrt(D - 2 * Math.sqrt(F))) / (4 * a)
                };
            } else {
                double signE = sgn(E);
                return new double[]{
                        (-b + signE * Math.sqrt(y1) + Math.sqrt(y2) + Math.sqrt(y3)) / (4 * a),
                        (-b + signE * Math.sqrt(y1) - Math.sqrt(y2) - Math.sqrt(y3)) / (4 * a),
                        (-b - signE * Math.sqrt(y1) + Math.sqrt(y2) - Math.sqrt(y3)) / (4 * a),
                        (-b - signE * Math.sqrt(y1) - Math.sqrt(y2) + Math.sqrt(y3)) / (4 * a)
                };
            }
        }
        return new double[]{};
    }

    public static double[] solve3(double a, double b, double c, double d) {
        if (Math.abs(a) < EPSILON) {
            return solve2(b, c, d);
        }
        double p = (3 * a * c - b * b) / (3 * a * a);
        double q = (2 * b * b * b - 9 * a * b * c + 27 * a * a * d) / (27 * a * a * a);

        double discriminant = (q * q) / 4 + (p * p * p) / 27;
        if (discriminant > 0) {
            double u = Math.cbrt(-q / 2 + Math.sqrt(discriminant));
            double v = Math.cbrt(-q / 2 - Math.sqrt(discriminant));
            double root = u + v - b / (3 * a);
            return new double[]{root};
        } else if (Math.abs(discriminant) < EPSILON) {
            if(Math.abs(q)<EPSILON){
                double root = -b / (3 * a);
                return new double[]{root};
            }else{
                double root1 = -2 * Math.cbrt(q / 2) - b / (3 * a);
                double root2 = Math.cbrt(q / 2) - b / (3 * a);
                return new double[]{root1, root2};
            }
        } else {
            double r = Math.sqrt(-p / 3);
            double theta = Math.acos(Math.max(-1, Math.min(1,-q / (2 * r * r * r))));
            double root1 = 2 * r * Math.cos(theta / 3) - b / (3 * a);
            double root2 = 2 * r * Math.cos((theta + 2 * Math.PI) / 3) - b / (3 * a);
            double root3 = 2 * r * Math.cos((theta + 4 * Math.PI) / 3) - b / (3 * a);
            return new double[]{root1, root2, root3};
        }
    }

    public static double[] solve2(double a, double b, double c) {
        if (Math.abs(a) < EPSILON) {
            return solve1(b, c);
        }
        double discriminant = b * b - 4 * a * c;
        if (discriminant > 0) {
            double root1 = (-b + Math.sqrt(discriminant)) / (2 * a);
            double root2 = (-b - Math.sqrt(discriminant)) / (2 * a);
            return new double[]{root1, root2};
        } else if (Math.abs(discriminant) < EPSILON) {
            double root = -b / (2 * a);
            return new double[]{root};
        } else {
            return new double[]{};
        }
    }

    public static double[] solve1(double a, double b) {
        if (Math.abs(a) < EPSILON) {
            throw new IllegalArgumentException("系数不能为0");
        }
        double root = -b / a;
        return new double[]{root};
    }

    public static double toInch(double MM) {
        return MM * 0.0394;
    }

    public static double toMM(double Inch) {
        return Inch * 25.4;
    }

    public static double avg(double... doubles) {
        if (doubles.length == 0) return 0.0;
        double sum = 0;
        for (double aDouble : doubles) {
            sum += aDouble;
        }
        return sum / doubles.length;
    }

    public static double avg(Number... numbers) {
        if (numbers.length == 0) return 0.0;
        double sum = 0;
        for (Number aDouble : numbers) {
            sum = sum + (double) aDouble;
        }
        return sum / numbers.length;
    }

    /**
     * 求解复数四次方程 a*x^4 + b*x^3 + c*x^2 + d*x + e = 0
     * 返回所有4个复数根（可能有重根）
     */
    public static ComplexNumber[] solve4Complex(double a, double b, double c, double d, double e) {
        if (Math.abs(a) < EPSILON) {
            return solve3Complex(b, c, d, e);
        }

        // 标准化：x = y - b/(4a)，消去三次项
        double p = (8 * a * c - 3 * b * b) / (8 * a * a);
        double q = (b * b * b - 4 * a * b * c + 8 * a * a * d) / (8 * a * a * a);
        double r = (-3 * b * b * b * b + 256 * a * a * a * e - 64 * a * a * b * d + 16 * a * b * b * c) / (256 * a * a * a * a);

        // 求解三次方程：z^3 + 2*p*z^2 + (p^2 - 4*r)*z - q^2 = 0
        ComplexNumber[] cubicRoots = solve3Complex(1, 2 * p, p * p - 4 * r, -q * q);
        
        // 从三次方程的根中选择一个非零根
        ComplexNumber z0 = null;
        for (ComplexNumber root : cubicRoots) {
            if (root.magnitude() > EPSILON) {
                z0 = root;
                break;
            }
        }
        
        if (z0 == null) {
            // 特殊情况：三次方程所有根都为零
            if (Math.abs(q) < EPSILON) {
                double root = -b / (4 * a);
                return new ComplexNumber[]{new ComplexNumber(root, 0), new ComplexNumber(root, 0), 
                                          new ComplexNumber(root, 0), new ComplexNumber(root, 0)};
            }
            return new ComplexNumber[]{};
        }

        // 计算四次方程的四个根
        ComplexNumber sqrtZ0 = z0.sqrt();
        ComplexNumber sqrtTerm = new ComplexNumber(p, 0).add(z0).subtract(new ComplexNumber(q, 0).divide(sqrtZ0));
        ComplexNumber sqrtTerm2 = new ComplexNumber(p, 0).add(z0).add(new ComplexNumber(q, 0).divide(sqrtZ0));
        
        ComplexNumber[] quadRoots1 = solve2Complex(1, sqrtZ0, sqrtTerm);
        ComplexNumber[] quadRoots2 = solve2Complex(1, sqrtZ0.negate(), sqrtTerm2);

        // 收集所有根并转换回原变量 x = y - b/(4a)
        ComplexNumber[] result = new ComplexNumber[4];
        int idx = 0;
        for (ComplexNumber root : quadRoots1) {
            if (idx < 4) {
                result[idx++] = root.subtract(new ComplexNumber(b / (4 * a), 0));
            }
        }
        for (ComplexNumber root : quadRoots2) {
            if (idx < 4) {
                result[idx++] = root.subtract(new ComplexNumber(b / (4 * a), 0));
            }
        }
        
        return result;
    }

    /**
     * 求解复数三次方程 a*x^3 + b*x^2 + c*x + d = 0
     */
    private static ComplexNumber[] solve3Complex(double a, double b, double c, double d) {
        if (Math.abs(a) < EPSILON) {
            return solve2Complex(b, c, d);
        }

        // 标准化：x = y - b/(3a)，消去二次项
        double p = (3 * a * c - b * b) / (3 * a * a);
        double q = (2 * b * b * b - 9 * a * b * c + 27 * a * a * d) / (27 * a * a * a);

        ComplexNumber[] roots = new ComplexNumber[3];
        
        // 计算判别式
        ComplexNumber discriminant = new ComplexNumber(q * q / 4, 0).add(new ComplexNumber(p * p * p / 27, 0));
        ComplexNumber sqrtD = discriminant.sqrt();
        
        // 计算两个立方根
        ComplexNumber u = new ComplexNumber(-q / 2, 0).add(sqrtD).cbrt();
        ComplexNumber v = new ComplexNumber(-q / 2, 0).subtract(sqrtD).cbrt();
        
        // 三个根（考虑单位根）
        ComplexNumber omega = new ComplexNumber(-0.5, Math.sqrt(3) / 2);  // 120度旋转
        ComplexNumber omega2 = new ComplexNumber(-0.5, -Math.sqrt(3) / 2); // 240度旋转
        
        for (int i = 0; i < 3; i++) {
            ComplexNumber ui = (i == 0) ? u : (i == 1) ? u.multiply(omega) : u.multiply(omega2);
            ComplexNumber vi = (i == 0) ? v : (i == 1) ? v.multiply(omega2) : v.multiply(omega);
            roots[i] = ui.add(vi).subtract(new ComplexNumber(b / (3 * a), 0));
        }
        
        return roots;
    }

    /**
     * 求解复数二次方程 a*x^2 + b*x + c = 0
     */
    private static ComplexNumber[] solve2Complex(double a, double b, double c) {
        if (Math.abs(a) < EPSILON) {
            return solve1Complex(b, c);
        }
        
        ComplexNumber discriminant = new ComplexNumber(b * b - 4 * a * c, 0);
        ComplexNumber sqrtD = discriminant.sqrt();
        
        return new ComplexNumber[]{
            sqrtD.subtract(new ComplexNumber(b, 0)).divide(new ComplexNumber(2 * a, 0)),
            sqrtD.negate().subtract(new ComplexNumber(b, 0)).divide(new ComplexNumber(2 * a, 0))
        };
    }

    /**
     * 求解复数一次方程 a*x + b = 0
     */
    private static ComplexNumber[] solve1Complex(double a, double b) {
        if (Math.abs(a) < EPSILON) {
            throw new IllegalArgumentException("系数不能为0");
        }
        return new ComplexNumber[]{new ComplexNumber(-b / a, 0)};
    }

    /**
     * 从复数根中过滤出虚部小于阈值的实数根
     */
    public static double[] filterRealRoots(ComplexNumber[] complexRoots) {
        java.util.ArrayList<Double> realRoots = new java.util.ArrayList<>();
        for (ComplexNumber root : complexRoots) {
            if (Math.abs(root.imag) < IMAG_THRESHOLD) {
                realRoots.add(root.real);
            }
        }
        double[] result = new double[realRoots.size()];
        for (int i = 0; i < realRoots.size(); i++) {
            result[i] = realRoots.get(i);
        }
        return result;
    }
}
