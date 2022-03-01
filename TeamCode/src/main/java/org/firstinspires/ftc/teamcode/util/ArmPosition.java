package org.firstinspires.ftc.teamcode.util;

public class ArmPosition {
    public double init;
    public double down;
    public double almostDown;
    public double up;
    public double almostGeneral;
    public double almostShared;
    public double almostLow;
    public double almostMid;
    public double almostHigh;
    public double general;
    public double shared;
    public double low;
    public double mid;
    public double high;

    public ArmPosition(double init, double down, double almostDown, double up, double almostGeneral, double almostShared, double almostLow, double almostMid, double almostHigh, double general, double shared, double low, double mid, double high) {
        this.init = init;
        this.down = down;
        this.almostDown = almostDown;
        this.up = up;
        this.almostGeneral = almostGeneral;
        this.almostShared = almostShared;
        this.almostLow = almostLow;
        this.almostMid = almostMid;
        this.almostHigh = almostHigh;
        this.general = general;
        this.shared = shared;
        this.low = low;
        this.mid = mid;
        this.high = high;
    }

    public double getInit() {
        return init;
    }

    public void setInit(double init) {
        this.init = init;
    }

    public double getDown() {
        return down;
    }

    public void setDown(double down) {
        this.down = down;
    }

    public double getAlmostDown() {
        return almostDown;
    }

    public void setAlmostDown(double almostDown) {
        this.almostDown = almostDown;
    }

    public double getUp() {
        return up;
    }

    public void setUp(double up) {
        this.up = up;
    }

    public double getAlmostGeneral() {
        return almostGeneral;
    }

    public void setAlmostGeneral(double almostGeneral) {
        this.almostGeneral = almostGeneral;
    }

    public double getAlmostShared() {
        return almostShared;
    }

    public void setAlmostShared(double almostShared) {
        this.almostShared = almostShared;
    }

    public double getAlmostLow() {
        return almostLow;
    }

    public void setAlmostLow(double almostLow) {
        this.almostLow = almostLow;
    }

    public double getAlmostMid() {
        return almostMid;
    }

    public void setAlmostMid(double almostMid) {
        this.almostMid = almostMid;
    }

    public double getAlmostHigh() {
        return almostHigh;
    }

    public void setAlmostHigh(double almostHigh) {
        this.almostHigh = almostHigh;
    }

    public double getGeneral() {
        return general;
    }

    public void setGeneral(double general) {
        this.general = general;
    }

    public double getShared() {
        return shared;
    }

    public void setShared(double shared) {
        this.shared = shared;
    }

    public double getLow() {
        return low;
    }

    public void setLow(double low) {
        this.low = low;
    }

    public double getMid() {
        return mid;
    }

    public void setMid(double mid) {
        this.mid = mid;
    }

    public double getHigh() {
        return high;
    }

    public void setHigh(double high) {
        this.high = high;
    }
}
