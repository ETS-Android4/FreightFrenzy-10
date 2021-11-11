package org.firstinspires.ftc.teamcode.util;

public class Range {
    public double l;
    public double u;
    public double v;

    public Range(double lower, double upper) {
        this.l = lower;
        this.u = upper;
    }

    public double[] get() {
        return new double[]{l, u};
    }

    public double getL() {
        return l;
    }

    public double getU() {
        return u;
    }
}
