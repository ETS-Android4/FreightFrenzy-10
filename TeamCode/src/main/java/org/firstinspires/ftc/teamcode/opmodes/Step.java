package org.firstinspires.ftc.teamcode.opmodes;


import org.firstinspires.ftc.teamcode.oldutil.vision.Detection;

// Class for every step that the autonomous program will take
public abstract class Step {
    private final double timeout;
    private String telemetry;

    // variables when moving
    public double destinationHeading;
    public double currentHeading;
    public double x;
    public double y;
    public double z;
    public double xErr;
    public double yErr;
    public double zErr;
    public double zRuntime;
    public double xRuntime;
    public double yRuntime;
    public double power;
    public boolean centeredLeftRight;

    // variables when shooting
    public Detection teamElement;

    // Constructors
    public Step(String telemetry) {
        this.telemetry = telemetry;
        this.timeout = -1;
    }
    public Step(String telemetry, double timeout) {
        this.telemetry = telemetry;
        this.timeout = timeout;
    }

    // Abstract methods to be overrode
    public abstract void start();
    public abstract void whileRunning();
    public abstract void end();
    public abstract boolean isFinished();

    // Return the timeout for the step
    public double getTimeout() {
        return timeout;
    }

    public void setTelemetry(String telemetry) {
        this.telemetry = telemetry;
    }
    // Return the Telemetry for the step
    public String getTelemetry() {
        return telemetry;
    }
}