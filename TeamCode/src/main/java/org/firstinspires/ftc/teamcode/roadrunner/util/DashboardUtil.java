package org.firstinspires.ftc.teamcode.roadrunner.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.opencv.core.Size;

import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 1.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 6; // in
    private static final Size ROBOT_SIZE = new Size(12.5,18);


    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d pose = path.get(displacement);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        Vector2d h = pose.headingVec().times(ROBOT_SIZE.height);
        Vector2d w = pose.headingVec().times(ROBOT_SIZE.width).rotated(Math.toRadians(90));
        Vector2d fl = h.plus(w);
        Vector2d fr = h.plus(w.rotated(Math.toRadians(180)));
        Vector2d bl = h.rotated(Math.toRadians(180)).plus(w);
        Vector2d br = h.rotated(Math.toRadians(180)).plus(w.rotated(Math.toRadians(180)));
        // draw rectangle
        double[] xPoints = {pose.getX() + fl.getX(), pose.getX() + fr.getX(), pose.getX() + br.getX(), pose.getX() + bl.getX()};
        double[] yPoints = {pose.getY() + fl.getY(), pose.getY() + fr.getY(), pose.getY() + br.getY(), pose.getY() + bl.getY()};
        canvas.strokePolygon(xPoints, yPoints);

        // draw line
        double x5 = pose.getX() + h.getX() / 2, y5 = pose.getY() + h.getY() / 2;
        double x6 = pose.getX() + h.getX(), y6 = pose.getY() + h.getY();
        canvas.strokeLine(x5, y5, x6, y6);

//        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
//        Vector2d v = pose.headingVec().times(ROBOT_RADIUS);
//        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
//        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
//        canvas.strokeLine(x1, y1, x2, y2);
    }
}
