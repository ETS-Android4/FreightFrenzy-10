package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.OpenCVUtil;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;


// Class for the pipeline that is used to detect the StarterStack
public class CameraVision extends OpenCvPipeline  {
    Mat blurred = new Mat();
    Mat hsv = new Mat();
    Mat orangeMask = new Mat();

    // Init
    @Override
    public void init(Mat input) {
    }

    // Process each frame that is received from the webcam
    @Override
    public Mat processFrame(Mat input)
    {
        //Imgproc.GaussianBlur(input, blurred, BLUR_SIZE, 0);
//        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

//        updateStarterStack(input);

        return input;
    }

}