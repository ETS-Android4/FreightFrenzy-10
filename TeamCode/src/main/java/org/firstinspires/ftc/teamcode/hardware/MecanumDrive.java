package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import static org.firstinspires.ftc.teamcode.oldutil.Constants.WHEEL_BACK_LEFT;
import static org.firstinspires.ftc.teamcode.oldutil.Constants.WHEEL_BACK_RIGHT;
import static org.firstinspires.ftc.teamcode.oldutil.Constants.WHEEL_FRONT_LEFT;
import static org.firstinspires.ftc.teamcode.oldutil.Constants.WHEEL_FRONT_RIGHT;

public class MecanumDrive {
    private final double wheelDiameter = 6.0;
    private final double wheelCircumference = Math.PI * wheelDiameter;
    private final double ticksPerRev;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    private double ticksToMove;
    private boolean trackingFrontLeft;

    public MecanumDrive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, WHEEL_FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, WHEEL_FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, WHEEL_BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, WHEEL_BACK_RIGHT);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        this.ticksPerRev = this.frontLeft.getMotorType().getTicksPerRev();

        this.setRunMode(RunMode.RUN_USING_ENCODER);
        this.setBrakeMode(ZeroPowerBehavior.BRAKE);
    }

    public boolean isBusy() {
        return this.getRunMode() != RunMode.RUN_TO_POSITION
                || this.frontLeft.isBusy()
                || this.frontRight.isBusy()
                || this.backLeft.isBusy()
                || this.backRight.isBusy();
    }

    // Move in two directions a certain number of inches
    public void setTargetPositionRelative(double x, double y, double power) {
        double ticksX = (x / wheelCircumference) * 537.7;
        double ticksY = (y / wheelCircumference) * 537.7;

        this.setRunMode(RunMode.STOP_AND_RESET_ENCODER);

        // if the 2 directions to go in have the same sign, track the front left, otherwise the back left motor
        // if either one of the directions are 0, then it doesn't matter so make it true just because
        trackingFrontLeft = x * y >= 0;
        // the ticks to move will be Y + X if it is the front left, otherwise the back left is calculated with Y - X
        ticksToMove = Math.abs(trackingFrontLeft ? ticksY + ticksX : ticksY - ticksX);

        this.frontLeft.setTargetPosition((int) (ticksY + ticksX));
        this.frontRight.setTargetPosition((int) (ticksY - ticksX));
        this.backLeft.setTargetPosition((int) (ticksY - ticksX));
        this.backRight.setTargetPosition((int) (ticksY + ticksX));

        this.setRunMode(RunMode.RUN_TO_POSITION);

        this.setPower(power);
    }

    // Get the total number of ticks for the current movement
    public double getTargetDistance() {
        return ticksToMove;
    }

    // Get the number of ticks remaining for the current movement
    public double getTargetDistanceRemaining() {
        return ticksToMove - Math.abs(trackingFrontLeft ? frontLeft.getCurrentPosition() : backLeft.getCurrentPosition());
    }

    public void setPower(double power) {
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(power);
        this.backRight.setPower(power);
    }

    public void setRunMode(RunMode runMode) {
        this.frontLeft.setMode(runMode);
        this.frontRight.setMode(runMode);
        this.backLeft.setMode(runMode);
        this.backRight.setMode(runMode);
    }

    public RunMode getRunMode() {
        return this.frontLeft.getMode();
    }

    public void setBrakeMode(ZeroPowerBehavior brakeMode) {
        this.frontLeft.setZeroPowerBehavior(brakeMode);
        this.frontRight.setZeroPowerBehavior(brakeMode);
        this.backLeft.setZeroPowerBehavior(brakeMode);
        this.backRight.setZeroPowerBehavior(brakeMode);
    }

    public void setWheels(double x, double y, double z) {
        // instantiate motor power variables
        double flPower, frPower, blPower, brPower;

        // compute motor powers
        // (see gm0.org/en/latest/docs/software/mecanum-drive.html for more info)
        flPower =  x + y + z; frPower = -x + y - z;
        blPower = -x + y + z; brPower =  x + y - z;

        // make sure that the motor powers don't exceed 1 or -1
        // (a value greater than 1 or -1 will just be truncated down so it's ok, however it will throw off the ratio with the other motors)
        double max = Math.max(Math.max(flPower, frPower), Math.max(blPower, brPower));
        if (max > 1) {
            flPower /= max; frPower /= max;
            blPower /= max; brPower /= max;
        }

        // actually set the motor powers
        frontLeft.setPower(flPower); frontRight.setPower(frPower);
        backLeft.setPower(blPower);  backRight.setPower(brPower);
    }
}