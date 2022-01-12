package org.firstinspires.ftc.teamcode.oldutil;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Configurables {
    public static Color YELLOW_LOWER = new Color(70, 50, 50);
    public static Color YELLOW_UPPER = new Color(100, 150, 200);

    public static double DRIVE_SPEED = 0.5;
    public static double INTAKE_SPEED = 0.75;

    public static double HOPPER_DELAY = 1.5;

    public static double DEADZONE = 0.1;

    //320 240
    public static int LEFT_BOUNDARY = -25;
    public static int RIGHT_BOUNDARY = 25;

    public static double SLIDE_SPEED = 1;

    public static double CAPPER_SPEED = 0.01;

    public static double HOPPER_INIT = 0.33;
    public static Range HOPPER_START = new Range(0.20, 0.99);
    public static Range HOPPER_MID = new Range(0.32, 0.99);
    public static double HOPPER_DROP_HIGH = 0.50;
    public static double HOPPER_DROP_MIDDLE = 0.66;
    public static double HOPPER_DROP_LOW_POS1 = 0.55;
    public static double HOPPER_DROP_LOW = 0.66;

    public static int SLIDE_DROP_HIGH = -2400;
    public static int SLIDE_DROP_MIDDLE = -1600;
    public static int SLIDE_DROP_LOW = -943;

    public static int SLIDE_CUTOFF = -250;

    public static double SERVO_MOVEMENT = 250;
    public static double SLIDE_TICKS_PER_CYCLE = 20;

    public static double AUTO_P = 0.4;
    public static double AUTO_MIN = 0.1;
    public static int AUTO_CUTOFF = 20;

    public static double CAPPER_UP_MAX = 0.85;//actually down
    public static double CAPPER_UP_CUTOFF = 0.7;
    public static double CAPPER_DOWN_CUTOFF = 0.4;
    public static double CAPPER_DOWN_MAX = 0.15;//actually up

    // 0 0.1504
    //  going up 0.305 or .3262
    // -2400 0.6531
    // -943 0.6801 (low)
    // -1411 0.6642 (middle)
}
