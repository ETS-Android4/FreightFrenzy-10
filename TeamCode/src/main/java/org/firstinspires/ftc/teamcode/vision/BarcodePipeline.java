package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

// Class for the pipeline that is used to detect the StarterStack
public class BarcodePipeline extends OpenCvPipeline  {
    Mat blurred = new Mat();
    Mat hsv = new Mat();

    private Detection teamElement;

    // Init
    @Override
    public void init(Mat input) {
    }

    // Process each frame that is received from the webcam
    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.GaussianBlur(input, blurred, new Size(7, 7), 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        updateStarterStack(input);

        return input;
    }

    private void updateStarterStack(Mat input) {
        Imgproc.rectangle(input, new Rect(0, 0, 100, 100), new Scalar(0, 0, 0), 2);
    }

    // Get the StarterStack
    public Detection getStarterStack() {
        return teamElement;
    }
}
