package org.firstinspires.ftc.teamcode.util.controller;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Joystick {
    public static double DEADZONE = 0.05;
    double x = 0;
    double y = 0;

    public Joystick() {}

    public void update(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        if(x<DEADZONE && x>-DEADZONE) {
            return 0;
        }
        else{
            return x;
        }
    }

    public double getY() {
        if(y<DEADZONE && y>-DEADZONE) {
            return 0;
        }
        else{
            return y;
        }
    }
}