package org.firstinspires.ftc.teamcode.util;

import android.graphics.Point;

public class MathUtil {
    public static int clamp(int x, int lowerBound, int upperBound) {
        return Math.min(Math.max(lowerBound, x), upperBound);
    }

    public static double clamp(double x, double lowerBound, double upperBound) {
        return Math.min(Math.max(lowerBound, x), upperBound);
    }

    public static boolean about(int num1, int num2) {
        return Math.abs(Math.abs(num1) - Math.abs(num2)) < 10;
    }

    public static double getDistanceBetweenTwoPoints (Point start, Point end) {
        float o = end.y - start.y;
        float a = end.x - start.x;
        return Math.sqrt(Math.pow(o, 2) + Math.pow(a, 2));
    }

    public static double getAngleBetweenTwoPoints(Point start, Point end) {
        float o = end.y - start.y;
        float a = end.x - start.x;

        double inRads = Math.atan2(o, a);
        return  (inRads >= 0 ? inRads : inRads + (2 * Math.PI)) * 180 / Math.PI;
    }

    public static float piTo2Pi(float angle) {
        return (angle + 360) % 360;
    }

    public static boolean isInRange2pi(float angle, float target, float window) {
        float min = piTo2Pi(target - window);
        float max = piTo2Pi(target + window);
        angle = piTo2Pi(angle);

        return angle > min && angle < max;
    }
}
