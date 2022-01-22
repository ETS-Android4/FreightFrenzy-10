package org.firstinspires.ftc.teamcode.visioncode;

import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.GREEN;
import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.INVALID_AREA;
import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.INVALID_POINT;
import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.drawConvexHull;
import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.drawPoint;
import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.fillConvexHull;
import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.getCenterOfContour;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class Detection {
    private Size frameSize;
    private double areaPx =  INVALID_AREA;
    private Point centerPx = INVALID_POINT;
    private MatOfPoint contour;

    // Constructor
    public Detection(Size frameSize) {
        this.frameSize = frameSize;
    }

    // Draw a convex hull around the current detection on the given image
    public void draw(Mat img, Scalar color) {
        if (isValid()) {
            drawConvexHull(img, contour, color);
            drawPoint(img, centerPx, GREEN);
        }
    }

    // Draw a convex hull around the current detection on the given image
    public void fill(Mat img, Scalar color) {
        if (isValid()) {
            fillConvexHull(img, contour, color);
            drawPoint(img, centerPx, GREEN);
        }
    }

    // Check if the current Detection is valid
    public boolean isValid() {
        return (this.contour != null) && (this.centerPx != INVALID_POINT) && (this.areaPx != INVALID_AREA);
    }

    // Get the current contour
    public MatOfPoint getContour() {
        return contour;
    }

    // Set the values of the current contour
    public void setContour(MatOfPoint contour) {
        this.contour = contour;

        double area;
        if (contour != null) {
            area = Imgproc.contourArea(contour);
            this.areaPx = area;
            this.centerPx = getCenterOfContour(contour);
        } else {
            this.areaPx = INVALID_AREA;
            this.centerPx = INVALID_POINT;
        }
    }

    // Returns the center of the Detection, normalized so that the width and height of the frame is from [-50,50]
    public Point getCenter() {
        if (!isValid()) {
            return INVALID_POINT;
        }

        double normalizedX = ((centerPx.x / frameSize.width)  * 100) - 50;
        double normalizedY = ((centerPx.y / frameSize.height) * 100) + 50;

        return new Point(normalizedX, normalizedY);
    }

    // Get the center point in pixels
    public Point getCenterPx() {
        return centerPx;
    }

    // Get the area of the Detection, normalized so that the area of the frame is 100
    public double getArea() {
        if (!isValid()) {
            return INVALID_AREA;
        }

        return (areaPx / (frameSize.width * frameSize.height)) * 100;
    }

    // Get the area of the Detection
    public double getAreaPx() {
        return areaPx;
    }
}
