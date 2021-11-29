package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Collections;
import java.util.List;

public class OpenCVUtil {

    /** Draw a dot on the given Mat at the given point.
     * @param img The {@link Mat} into which to draw.
     * @param point The {@link Point} at which to draw.
     * @param color The {@link Scalar}, in BGR color format, to use.
     */
    public static void drawPoint(Mat img, Point point, Scalar color) {
        Imgproc.circle(img, point, 3, color,  -1);
    }

    /** Finds the center (centroid) of the given contour.
     * @param contour The contour.
     * @return A {@link Point} representing the center of the given contour.
     */
    public static Point getCenterOfContour(MatOfPoint contour) {
        Moments moments = Imgproc.moments(contour);
        return new Point(moments.m10 / moments.m00, moments.m01/ moments.m00);
    }

    /** Gets the point representing the bottom left corner of the contour's bounding box.
     * @param contour The contour.
     * @return A {@link Point} representing the bottom left corner of the contour's bounding box.
     */
    public static Point getBottomLeftOfContour(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return new Point(boundingRect.x, boundingRect.y+boundingRect.height);
    }

    /** Gets the point representing the bottom left corner of the contour's bounding box.
     * @param contour The contour.
     * @return A {@link Point} representing the bottom left corner of the contour's bounding box.
     */    public static Point getBottomRightOfContour(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return new Point(
                boundingRect.x+boundingRect.width,
                boundingRect.y+boundingRect.height);
    }

    /** Draws tightly fitted line around the contour.
     * @param img The {@link Mat} into which to draw.
     * @param contour The contour.
     * @param color The {@link Scalar}, in BGR color format, to use.
     */
    public static void drawContour(Mat img, MatOfPoint contour, Scalar color) {
        Imgproc.drawContours(
                img,
                Collections.singletonList(contour),
                0,
                color,
                2);
    }

    /** Draws a line representing the convex hull of the given contour.
     * @param img The {@link Mat} into which to draw.
     * @param contour The contour.
     * @param color The {@link Scalar}, in BGR color format, to use.
     */
    public static void drawConvexHull(Mat img, MatOfPoint contour, Scalar color) {
        drawConvexHull(img, contour, color, 2);
    }

    /** Draws a line representing the convex hull of the given contour.
     * @param img The {@link Mat} into which to draw.
     * @param contour The contour.
     * @param color The {@link Scalar}, in BGR color format, to use.
     * @param stroke the thickness of the line to draw.
     */
    public static void drawConvexHull(Mat img, MatOfPoint contour, Scalar color, int stroke) {
        MatOfInt hull =  new MatOfInt();
        Imgproc.convexHull(contour, hull);
        Imgproc.drawContours(
                img,
                Collections.singletonList(indexesToPoints(contour, hull)),
                0,
                color,
                stroke);
    }

    /** Returns the largest contour (by area) in the list of contours.
     * @param contours The list of contours.
     * @return The contour from the list whose area is largest.
     */
    public static MatOfPoint getLargestContour(List<MatOfPoint> contours) {
        if (contours.size() == 0) {
            return null;
        }

        return getLargestContours(contours, 1).get(0);
    }

    /** Returns the N largest contours (by area) in the list of contours.
     * @param contours The list of contours.
     * @param numContours The maximum number of contours to return.
     * @return The contour from the list whose area is largest.
     */
    public static List<MatOfPoint> getLargestContours(List<MatOfPoint> contours, int numContours) {
        Collections.sort(
                contours,
                (a, b) -> (int) Imgproc.contourArea(b) - (int) Imgproc.contourArea(a));

        return contours.subList(0, Math.min(numContours, contours.size()));
    }

    private static MatOfPoint indexesToPoints(MatOfPoint contour, MatOfInt indexes) {
        int[] arrIndex = indexes.toArray();
        Point[] arrContour = contour.toArray();
        Point[] arrPoints = new Point[arrIndex.length];

        for (int i=0;i<arrIndex.length;i++) {
            arrPoints[i] = arrContour[arrIndex[i]];
        }

        MatOfPoint hull = new MatOfPoint();
        hull.fromArray(arrPoints);

        return hull;
    }

    public static class Constants {
        // CV Color Constants
        public static Scalar RED = new Scalar(255, 0, 0);
        public static Scalar GREEN = new Scalar(0, 255, 0);
        public static Scalar BLUE = new Scalar(0, 0, 255);
        public static Scalar WHITE = new Scalar(255, 255, 255);
        public static Scalar GRAY = new Scalar(80, 80, 80);
        public static Scalar BLACK = new Scalar(0, 0, 0);
        public static Scalar ORANGE = new Scalar(255, 165, 0);
        public static Scalar YELLOW = new Scalar(255, 255, 0);
        public static Scalar PURPLE = new Scalar(128, 0, 128);

        // CV Structuring Constants
        public static final Mat STRUCTURING_ELEMENT =
                Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        public static final Point ANCHOR =
                new Point((STRUCTURING_ELEMENT.cols() / 2f), STRUCTURING_ELEMENT.rows() / 2f);
        public static final int ERODE_DILATE_ITERATIONS = 2;
        public static final Size BLUR_SIZE = new Size(7, 7);

        // CV Camera Constants
        public static final int WEBCAM_WIDTH = 320;
        public static final int WEBCAM_HEIGHT = 240;
        public static final OpenCvCameraRotation WEBCAM_ROTATION = OpenCvCameraRotation.UPRIGHT;

        // CV Invalid Detection Constants
        public static final Point INVALID_POINT = new Point(Double.MIN_VALUE, Double.MIN_VALUE);
        public static final double INVALID_AREA = -1;
        public static final Detection INVALID_DETECTION =
                new Detection(new Size(0, 0), 0);
    }
}