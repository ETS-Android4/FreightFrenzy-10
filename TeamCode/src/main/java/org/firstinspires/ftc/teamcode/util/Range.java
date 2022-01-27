package org.firstinspires.ftc.teamcode.util;

public class Range {
    public int lower;
    public int upper;
    public double dLower;
    public double dUpper;

    public Range(int lower, int upper) {
        this.lower = lower;
        this.lower = upper;
    }

    public Range(double lower, double upper) {
        this.dLower = lower;
        this.dUpper = upper;
    }

    public int getMin() {
        return lower;
    }

    public int getMax() {
        return upper;
    }

    public double getDoubleMin() {
        return dLower;
    }

    public double getDoubleMax() {
        return dUpper;
    }


}
