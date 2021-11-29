package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.OpenCVUtil.Constants.*;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

import lombok.Getter;

public class BarcodePipeline extends OpenCvPipeline {
    Mat blurred = new Mat();
    Mat hsv = new Mat();
    Mat blobs = new Mat();

    @Getter
    private BarcodeDetection barcode;

    @Override
    public void init(Mat input) {
        this.barcode = new BarcodeDetection(
                input.size(),
                Constants.MIN_TEAM_ELEMENT_AREA,
                Constants.MAX_TEAM_ELEMENT_AREA);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.GaussianBlur(input, blurred, BLUR_SIZE, 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        updateBarcodeDetection();

        barcode.draw(input, WHITE);
        return input;
    }

    private void updateBarcodeDetection() {
        // take pixels that are in the color range and put them into a mask,
        // eroding and dilating them to remove white noise
        Core.inRange(
                hsv,
                Constants.TEAM_MARKER_DETECTION_RANGE_LOWER,
                Constants.TEAM_MARKER_DETECTION_RANGE_UPPER,
                blobs);
        Imgproc.erode(
                blobs,
                blobs,
                STRUCTURING_ELEMENT,
                ANCHOR,
                ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(
                blobs,
                blobs,
                STRUCTURING_ELEMENT,
                ANCHOR,
                ERODE_DILATE_ITERATIONS);

        // make a new mask that will be used to find the contours of the StarterStack
        Mat barcodeMask = new Mat(hsv.size(), CvType.CV_8U);
        Imgproc.rectangle(barcodeMask, Constants.BARCODE_WINDOW, WHITE, -1);
        Core.bitwise_and(blobs, barcodeMask, barcodeMask);

        // set the largest detection that was found to be the StarterStack
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(
                barcodeMask,
                contours,
                new Mat(),
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE);
        barcode.setContour(OpenCVUtil.getLargestContour(contours));
    }

    public static class Constants {
        public static final Scalar TEAM_MARKER_DETECTION_RANGE_LOWER = new Scalar(255, 0, 0);
        public static final Scalar TEAM_MARKER_DETECTION_RANGE_UPPER = new Scalar(255, 0, 0);
        public static final Scalar DUCK_DETECTION_RANGE_LOWER = new Scalar(255, 0, 0);
        public static final Scalar DUCK_DETECTION_RANGE_UPPER = new Scalar(255, 0, 0);

        public static final double MIN_TEAM_ELEMENT_AREA = 0.0;
        public static final double MAX_TEAM_ELEMENT_AREA = 1.0;
        public static final double MIN_DUCK_AREA = 0.0;

        public static final Rect BARCODE_WINDOW = new Rect(0,0,0,0);
    }
}
