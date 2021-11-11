package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Configurables {
    public static Color YELLOW_LOWER = new Color(30, 70, 50);
    public static Color YELLOW_UPPER = new Color(80, 255, 255);

    public static double DRIVE_SPEED = 0.75;
    public static double INTAKE_SPEED = 0.75;

    //320 240
    public static int LEFT_BOUNDARY = -25;
    public static int RIGHT_BOUNDARY = 25;

    public static double SLIDE_SPEED = 0.75;

    public static double HOPPER_INIT = 0.15;
    public static Range HOPPER_START = new Range(0.15, 0.99);
    public static Range HOPPER_MID = new Range(0.32, 0.99);
    public static double HOPPER_DROP_HIGH = 0.65;
    public static double HOPPER_DROP_MIDDLE = 0.66;
    public static double HOPPER_DROP_LOW_POS1 = 0.55;
    public static double HOPPER_DROP_LOW = 0.68;

    public static int SLIDE_DROP_HIGH = -2400;
    public static int SLIDE_DROP_MIDDLE = -1411;
    public static int SLIDE_DROP_LOW = -943;

    public static int SLIDE_CUTOFF = -250;

    public static double SERVO_MOVEMENT = 250;
    public static double SLIDE_MOVEMENT = 10;

    // 0 0.1504
    //  going up 0.305 or .3262
    // -2400 0.6531
    // -943 0.6801 (low)
    // -1411 0.6642 (middle)
}
