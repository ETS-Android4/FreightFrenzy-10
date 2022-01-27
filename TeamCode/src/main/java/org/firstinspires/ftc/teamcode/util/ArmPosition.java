package org.firstinspires.ftc.teamcode.util;

public class ArmPosition {
    public double down;
    public double almostDown;
    public double up;
    public double deposit;
    
    public ArmPosition(double down, double almostDown, double up, double deposit) {
        this.down = down;
        this.almostDown = almostDown;
        this.up = up;
        this.deposit = deposit;
    }

    public double getDown() {
        return down;
    }

    public double getAlmostDown() {
        return almostDown;
    }

    public double getUp() {
        return up;
    }

    public double getDeposit() {
        return deposit;
    }
}
