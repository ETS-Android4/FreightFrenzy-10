package org.firstinspires.ftc.teamcode.util;

public class MathUtil {
    public static int clamp(int x, int lowerBound, int upperBound) {
        return Math.min(Math.max(lowerBound, x), upperBound);
    }

    public static double clamp(double x, double lowerBound, double upperBound) {
        return Math.min(Math.max(lowerBound, x), upperBound);
    }
}
