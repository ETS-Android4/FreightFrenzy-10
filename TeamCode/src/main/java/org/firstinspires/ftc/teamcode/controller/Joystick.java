package org.firstinspires.ftc.teamcode.controller;

import static org.firstinspires.ftc.teamcode.util.Configurables.DEADZONE;

public class Joystick {
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