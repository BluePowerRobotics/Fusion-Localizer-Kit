package org.firstinspires.ftc.teamcode.utility.Geometry;

import org.firstinspires.ftc.teamcode.utility.Vector3D;

public class Projector {
    private EulerAngle pose;
    private double l0;

    public Projector(EulerAngle pose, double l0) {
        this.pose = pose;
        this.l0 = l0;
    }

    public EulerAngle getPose() {
        return pose;
    }

    public void setPose(EulerAngle pose) {
        this.pose = pose;
    }

    public double getL0() {
        return l0;
    }

    public void setL0(double l0) {
        this.l0 = l0;
    }

    public Vector3D project(double x_p, double y_p, double k) {
        double d = 1.0 / k;
        double A = 1.0 / (k * l0);

        Vector3D cameraCoord = new Vector3D(d, -A * x_p, A * y_p);

        return pose.reform(cameraCoord);
    }

    @Override
    public String toString() {
        return "Projector{" +
                "pose=" + pose +
                ", l0=" + l0 +
                '}';
    }
}
