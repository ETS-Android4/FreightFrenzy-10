package org.firstinspires.ftc.teamcode.visioncode;

import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.ANCHOR;
import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.BarcodeLocation.LEFT;
import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.BarcodeLocation.MIDDLE;
import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.BarcodeLocation.RIGHT;
import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.BarcodeLocation.UNKNOWN;
import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.ERODE_DILATE_ITERATIONS;
import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.STRUCTURING_ELEMENT;
import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.WHITE;
import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.YELLOW_LOWER;
import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.YELLOW_UPPER;
import static org.firstinspires.ftc.teamcode.visioncode.Cvhelper.getLargestContour;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class BarcodePipeline extends OpenCvPipeline {
    Mat blurred = new Mat();
    Mat hsv = new Mat();
    Mat yellowMask = new Mat();

    private Detection teamElement;

    // Init
    @Override
    public void init(Mat input) {
        teamElement = new Detection(input.size());
    }

    // Process each frame that is received from the webcam
    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.GaussianBlur(input, blurred, new Size(7, 7), 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        findTeamElement(input);

        return input;
    }

    private void findTeamElement(Mat input) {
        Core.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER, yellowMask);
        Imgproc.erode(yellowMask, yellowMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(yellowMask, yellowMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        // set the largest detection that was found to be the Team Element
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(yellowMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        teamElement.setContour(getLargestContour(contours));

        // draw the Team Element detection
        teamElement.draw(input, WHITE);
    }

    // Get the StarterStack
    public Detection getTeamElement() {
        return teamElement;
    }

    public Cvhelper.BarcodeLocation getTeamElementLocation() {
        if (teamElement.isValid()) {
            if (teamElement.getCenter().x < -10) {
                return LEFT;
            } else if (teamElement.getCenter().x > 25) {
                return RIGHT;
            } else {
                return MIDDLE;
            }
        }
        return UNKNOWN;
    }





}
