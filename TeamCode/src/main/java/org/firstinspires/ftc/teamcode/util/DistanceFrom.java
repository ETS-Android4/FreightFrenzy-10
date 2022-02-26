package org.firstinspires.ftc.teamcode.util;
//package org.firstinspires.ftc.teamcode.oldutil;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
//package org.firstinspires.ftc.teamcode.oldutil.vision;
//package org.opencv.aruco as aruco;

//package es.ava.aruco as aruco;

import java.util.Collections;
import java.util.Dictionary;
import java.util.List;
import java.util.Vector;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/*this class is intended to recognize how far away an object primaraly id'd by color is from the
camera, using known apparent sized at certain known distances measured beforehand
 */
public class DistanceFrom {
    //configurables and settings

        //hsv values for color detection
            private int h_min = 0;
            private int h_max = 100;

            private int s_min = 0;
            private int s_max = 100;

            private int v_min = 0;
            private int v_max = 100;

        //other known object properties
            //number of pixles on the small image size that the object takes up
            private double baseline_area = 100000;
            //distance in inches from the camera that the number of pixles was measured at
            private double baseline_distance = 10;

    //constructs a watcher that will calculate distance when requested for its item
    public DistanceFrom(){ //creates an object
        //private double redHubAngle;
        //private double redHubDistance;
    }

    //public double distance(Mat cool_frame){
        //Core.bitwise_not(cool_frame,cool_frame);
        //Imgproc.cvtColor(cool_frame, cool_frame, Imgproc.COLOR_RGB2GRAY);
        //Dictionary aruco_dict = aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

    //}

}//end of class