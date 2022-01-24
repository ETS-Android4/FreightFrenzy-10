package org.firstinspires.ftc.teamcode.oldutil.vision;

import org.firstinspires.ftc.teamcode.oldutil.BarcodeLocation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.oldutil.BarcodeLocation.LEFT;
import static org.firstinspires.ftc.teamcode.oldutil.BarcodeLocation.MIDDLE;
import static org.firstinspires.ftc.teamcode.oldutil.BarcodeLocation.RIGHT;
import static org.firstinspires.ftc.teamcode.oldutil.BarcodeLocation.UNKNOWN;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.LEFT_BOUNDARY;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.RIGHT_BOUNDARY;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.YELLOW_LOWER;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.YELLOW_UPPER;
import static org.firstinspires.ftc.teamcode.oldutil.Constants.ANCHOR;
import static org.firstinspires.ftc.teamcode.oldutil.Constants.ERODE_DILATE_ITERATIONS;
import static org.firstinspires.ftc.teamcode.oldutil.Constants.STRUCTURING_ELEMENT;
import static org.firstinspires.ftc.teamcode.oldutil.Constants.WHITE;
import static org.firstinspires.ftc.teamcode.oldutil.vision.OpenCVUtil.getBottomLeftOfContour;
import static org.firstinspires.ftc.teamcode.oldutil.vision.OpenCVUtil.getTopLeftOfContour;

// Class for the pipeline that is used to detect the StarterStack
public class BarcodePipeline extends OpenCvPipeline  {
    private Mat blurred = new Mat();
    private Mat hsv = new Mat();
    private Mat yellowMask = new Mat();

    //apriltag stuff
    private Mat grayMask = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();
    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private long nativeApriltagPtr;
    private final Object detectionsUpdateSync = new Object();
    Mat cameraMatrix;
    double fx, fy, cx, cy;
    double tagsize, tagsizeX, tagsizeY;
    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    private Detection teamElement;

    // Init
    @Override
    public void init(Mat input) {
        teamElement = new Detection(input.size(), 0.01);
//        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

//    @Override
//    public void finalize() {
//        AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
//    }

    // Process each frame that is received from the webcam
    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.GaussianBlur(input, blurred, new Size(7, 7), 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        findTeamElement(input);

        //aruco stuff
//        Imgproc.cvtColor(input, grayMask, Imgproc.COLOR_RGB2GRAY);
//        Core.bitwise_not(grayMask, grayMask);
//        findTeamElementUsingAprilTags(grayMask);

        return input;
    }

    private void findTeamElement(Mat input) {
        Core.inRange(hsv, new Scalar(YELLOW_LOWER.get()), new Scalar(YELLOW_UPPER.get()), yellowMask);
        Imgproc.erode(yellowMask, yellowMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(yellowMask, yellowMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        // set the largest detection that was found to be the Team Element
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(yellowMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++) {
            if (getTopLeftOfContour(contours.get(i)).y < 45) {
                contours.remove(i);
                i--;
            } else if (getBottomLeftOfContour(contours.get(i)).y > 240-45) {
                contours.remove(i);
                i--;
            }
        }
        teamElement.setContour(OpenCVUtil.getLargestContour(contours));

        // draw the Team Element detection
        teamElement.draw(input, WHITE);
    }

    // Get the StarterStack
    public Detection getTeamElement() {
        return teamElement;
    }

    public BarcodeLocation getTeamElementLocation() {
        if (teamElement.isValid()) {
            if (teamElement.getCenter().x < LEFT_BOUNDARY) {
                return LEFT;
            } else if (teamElement.getCenter().x > RIGHT_BOUNDARY) {
                return RIGHT;
            } else {
                return MIDDLE;
            }
        }
        return UNKNOWN;
    }

    // Apriltag Stuff
    private void findTeamElementUsingAprilTags(Mat grayMask) {
        synchronized (decimationSync) {
            if(needToSetDecimation) {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grayMask, tagsize, fx, fy, cx, cy);

        synchronized (detectionsUpdateSync) {
            detectionsUpdate = detections;
        }
    }

    public BarcodeLocation getTeamElementLocationUsingAprilTag() {
        if (detections.size() > 0) {
            if (detections.get(0).center.x < LEFT_BOUNDARY) {
                return LEFT;
            } else if (detections.get(0).center.x > RIGHT_BOUNDARY) {
                return RIGHT;
            } else {
                return MIDDLE;
            }
        }
        return UNKNOWN;
    }
}
