package org.firstinspires.ftc.teamcode.utility.Algebra;

/**
 * 复数运算类
 */
public class ComplexNumber {
    public double real;
    public double imag;

    public ComplexNumber(double real, double imag) {
        this.real = real;
        this.imag = imag;
    }

    public ComplexNumber add(ComplexNumber other) {
        return new ComplexNumber(real + other.real, imag + other.imag);
    }

    public ComplexNumber subtract(ComplexNumber other) {
        return new ComplexNumber(real - other.real, imag - other.imag);
    }

    public ComplexNumber multiply(ComplexNumber other) {
        double r = real * other.real - imag * other.imag;
        double i = real * other.imag + imag * other.real;
        return new ComplexNumber(r, i);
    }

    public ComplexNumber divide(ComplexNumber other) {
        double denominator = other.real * other.real + other.imag * other.imag;
        double r = (real * other.real + imag * other.imag) / denominator;
        double i = (imag * other.real - real * other.imag) / denominator;
        return new ComplexNumber(r, i);
    }

    public ComplexNumber multiply(double scalar) {
        return new ComplexNumber(real * scalar, imag * scalar);
    }

    public double magnitude() {
        return Math.sqrt(real * real + imag * imag);
    }

    public ComplexNumber sqrt() {
        double r = Math.sqrt(magnitude());
        double theta = Math.atan2(imag, real) / 2;
        return new ComplexNumber(r * Math.cos(theta), r * Math.sin(theta));
    }

    public ComplexNumber cbrt() {
        double r = Math.cbrt(magnitude());
        double theta = Math.atan2(imag, real) / 3;
        return new ComplexNumber(r * Math.cos(theta), r * Math.sin(theta));
    }

    public ComplexNumber negate() {
        return new ComplexNumber(-real, -imag);
    }

    public boolean isReal() {
        return Math.abs(imag) < 1e-10;
    }

    public double toReal() {
        return real;
    }
}
