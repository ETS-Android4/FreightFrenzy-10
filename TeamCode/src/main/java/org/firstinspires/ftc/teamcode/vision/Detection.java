package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import static org.firstinspires.ftc.teamcode.vision.OpenCVUtil.Constants.*;
import static org.firstinspires.ftc.teamcode.vision.OpenCVUtil.*;

import lombok.Getter;

public class Detection {
    protected double minAreaPx;
    protected double maxAreaPx;
    protected final Size maxSizePx;
    protected double areaPx =  INVALID_AREA;
    @Getter protected Point centerPx = INVALID_POINT;
    @Getter protected Point bottomLeftPx = INVALID_POINT;
    @Getter protected Point bottomRightPx = INVALID_POINT;
    @Getter protected MatOfPoint contour;

    public Detection(Size frameSize, double minAreaFactor) {
        this.maxSizePx = frameSize;
        this.minAreaPx = frameSize.area() * minAreaFactor;
        this.maxAreaPx = frameSize.area();
    }

    public Detection(Size frameSize, double minAreaFactor, double maxAreaFactor) {
        this.maxSizePx = frameSize;
        this.minAreaPx = frameSize.area() * minAreaFactor;
        this.maxAreaPx = frameSize.area() * maxAreaFactor;
    }

    /** Sets the minimum area for a contour to be considered a valid detection.
     * @param minAreaFactor The ratio of detection area to total image area below which contours
     *                      should be ignored.
     */
    public void setMinArea(double minAreaFactor) {
        this.minAreaPx = maxSizePx.area() * minAreaFactor;
    }


    /** Sets the maximum area for a contour to be considered a valid detection.
     * @param maxAreaFactor The ratio of detection area to total image area above which contours
     *                      should be ignored.
     */
    public void setMaxArea(double maxAreaFactor) {
        this.minAreaPx = maxSizePx.area() * maxAreaFactor;
    }

    /** Draws a line around the convex hull of the detection to the given Mat.
     * @param img The {@link Mat} into which to draw.
     * @param color The {@link Scalar}, in BGR color format, to use.
     */
    public void draw(Mat img, Scalar color) {
        if (isValid()) {
            drawConvexHull(img, contour, color);
            drawPoint(img, centerPx, GREEN);
        }
    }

    /** Draws the filled convex hull of the detection to the given Mat.
     * @param img The {@link Mat} into which to draw.
     * @param color The {@link Scalar}, in BGR color format, to use.
     */
    public void fill(Mat img, Scalar color) {
        if (isValid()) {
            drawConvexHull(img, contour, color, -1);
            drawPoint(img, centerPx, GREEN);
        }
    }

    /** Get whether the detection has a non-null contour whose center and area can be calculated.
     * @return True if the contour is non-null, and its center and area can be calculated.
     */
    public boolean isValid() {
        return this.contour != null
                && this.centerPx != INVALID_POINT
                && this.areaPx != INVALID_AREA;
    }

    /** Updates this Detection with the given contour.
     * @param contour The contour.
     */
    public void setContour(MatOfPoint contour) {
        this.contour = contour;

        double area;
        if (contour != null
                && (area = Imgproc.contourArea(contour)) > minAreaPx
                && area < maxAreaPx) {
            this.areaPx = area;
            this.centerPx = getCenterOfContour(contour);
            this.bottomLeftPx = getBottomLeftOfContour(contour);
            this.bottomRightPx = getBottomRightOfContour(contour);
        } else {
            this.areaPx = INVALID_AREA;
            this.centerPx = INVALID_POINT;
            this.bottomLeftPx = INVALID_POINT;
            this.bottomRightPx = INVALID_POINT;
        }
    }

    /** Gets the center (centroid) of the contour normalized
     * such that -50 < x < 50, and -50 < y < 50.
     * @return The center of the detection's contour.
     */
    public Point getCenter() {
        if (!isValid()) {
            return INVALID_POINT;
        }

        double normalizedX = ((centerPx.x / maxSizePx.width) * 100) - 50;
        double normalizedY = ((centerPx.y / maxSizePx.height) * -100) + 50;

        return new Point(normalizedX, normalizedY);
    }

    /** Gets the area of the contour normalized as a percentage (0-100) of the total frame.
     * @return The area of the detection's contour.
     */
    public double getArea() {
        if (!isValid()) {
            return INVALID_AREA;
        }

        return (areaPx / (maxSizePx.width * maxSizePx.height)) * 100;
    }
}