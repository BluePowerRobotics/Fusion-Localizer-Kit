package org.firstinspires.ftc.teamcode.utility.Geometry;

import org.firstinspires.ftc.teamcode.utility.Vector3D;

public class EulerAngle {
    private double roll;
    private double pitch;
    private double yaw;

    public EulerAngle(double roll, double pitch, double yaw) {
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
    }

    public double getRoll() {
        return roll;
    }

    public void setRoll(double roll) {
        this.roll = roll;
    }

    public double getPitch() {
        return pitch;
    }

    public void setPitch(double pitch) {
        this.pitch = pitch;
    }

    public double getYaw() {
        return yaw;
    }

    public void setYaw(double yaw) {
        this.yaw = yaw;
    }

    public Vector3D reform(Vector3D vec) {
        double x = vec.getX();
        double y = vec.getY();
        double z = vec.getZ();

        double cosYaw = Math.cos(yaw);
        double sinYaw = Math.sin(yaw);
        double cosPitch = Math.cos(pitch);
        double sinPitch = Math.sin(pitch);
        double cosRoll = Math.cos(roll);
        double sinRoll = Math.sin(roll);

        double v2x = x * cosPitch + z * sinPitch;
        double v2y = y;
        double v2z = -x * sinPitch + z * cosPitch;

        double v1x = v2x * cosYaw - v2y * sinYaw;
        double v1y = v2x * sinYaw + v2y * cosYaw;
        double v1z = v2z;

        double resultX = v1x;
        double resultY = v1y * cosRoll - v1z * sinRoll;
        double resultZ = v1y * sinRoll + v1z * cosRoll;

        return new Vector3D(resultX, resultY, resultZ);
    }

    public Vector3D transform(Vector3D vec) {
        double x = vec.getX();
        double y = vec.getY();
        double z = vec.getZ();

        double cosYaw = Math.cos(yaw);
        double sinYaw = Math.sin(yaw);
        double cosPitch = Math.cos(pitch);
        double sinPitch = Math.sin(pitch);
        double cosRoll = Math.cos(roll);
        double sinRoll = Math.sin(roll);

        double v1x = x;
        double v1y = y * cosRoll + z * sinRoll;
        double v1z = -y * sinRoll + z * cosRoll;

        double v2x = v1x * cosYaw + v1y * sinYaw;
        double v2y = -v1x * sinYaw + v1y * cosYaw;
        double v2z = v1z;

        double resultX = v2x * cosPitch - v2z * sinPitch;
        double resultY = v2y;
        double resultZ = v2x * sinPitch + v2z * cosPitch;

        return new Vector3D(resultX, resultY, resultZ);
    }

    @Override
    public String toString() {
        return "EulerAngle{" +
                "roll=" + roll +
                ", pitch=" + pitch +
                ", yaw=" + yaw +
                '}';
    }
}
