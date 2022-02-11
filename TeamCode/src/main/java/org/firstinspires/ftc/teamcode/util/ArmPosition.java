package org.firstinspires.ftc.teamcode.util;

public class ArmPosition {
    public double init;
    public double down;
    public double almostDown;
    public double up;
    public double almostLow;
    public double almostMid;
    public double almostHigh;
    public double low;
    public double mid;
    public double high;

    //old
    public double almostDeposit;
    public double deposit;

    // old constructor
    public ArmPosition(double down, double almostDown, double up, double deposit) {
        this.down = down;
        this.almostDown = almostDown;
        this.up = up;
        this.deposit = deposit;
    }

    // new constructor
    public ArmPosition(double init, double down, double almostDown, double up, double almostLow, double almostMid, double almostHigh, double low, double mid, double high) {
        this.init = init;
        this.down = down;
        this.almostDown = almostDown;
        this.up = up;
        this.almostLow = almostLow;
        this.almostMid = almostMid;
        this.almostHigh = almostHigh;
        this.low = low;
        this.mid = mid;
        this.high = high;
    }

    public double getInit() {
        return init;
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

    public double getAlmostLow() {
        return almostLow;
    }

    public double getAlmostMid() {
        return almostMid;
    }

    public double getAlmostHigh() {
        return almostHigh;
    }

    public double getLow() {
        return low;
    }

    public double getMid() {
        return mid;
    }

    public double getHigh() {
        return high;
    }

    // old
    public double getAlmostDeposit() {
        return almostDeposit;
    }

    public void setInit(double init) {
        this.init = init;
    }

    public void setDown(double down) {
        this.down = down;
    }

    public void setAlmostDown(double almostDown) {
        this.almostDown = almostDown;
    }

    public void setUp(double up) {
        this.up = up;
    }

    public void setAlmostLow(double almostLow) {
        this.almostLow = almostLow;
    }

    public void setAlmostMid(double almostMid) {
        this.almostMid = almostMid;
    }

    public void setAlmostHigh(double almostHigh) {
        this.almostHigh = almostHigh;
    }

    public void setLow(double low) {
        this.low = low;
    }

    public void setMid(double mid) {
        this.mid = mid;
    }

    public void setHigh(double high) {
        this.high = high;
    }

    public void setAlmostDeposit(double almostDeposit) {
        this.almostDeposit = almostDeposit;
    }

    public void setDeposit(double deposit) {
        this.deposit = deposit;
    }

}
