package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_BACK_LEFT;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_BACK_RIGHT;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_FRONT_LEFT;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_FRONT_RIGHT;

public class MecanumDrive {
    private final double wheelDiameter = 4.0;
    private final double wheelCircumference = Math.PI * wheelDiameter;
    private final double ticksPerRev;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

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

    public void setTargetForwardPositionRelative(double inches, double power) {
        int ticks = (int)((inches / wheelCircumference) * 560);
        this.setRunMode(RunMode.STOP_AND_RESET_ENCODER);
        this.setRunMode(RunMode.RUN_TO_POSITION);

        this.frontLeft.setTargetPosition(ticks);
        this.frontRight.setTargetPosition(ticks);
        this.backLeft.setTargetPosition(ticks);
        this.backRight.setTargetPosition(ticks);

        this.setPower(power);
    }

    public void setTargetStrafePositionRelative(double inches, double power) {
        int ticks = (int)((inches / wheelCircumference) * 560);
        this.setRunMode(RunMode.STOP_AND_RESET_ENCODER);
        this.setRunMode(RunMode.RUN_TO_POSITION);

        this.frontLeft.setTargetPosition(ticks);
        this.frontRight.setTargetPosition(-ticks);
        this.backLeft.setTargetPosition(-ticks);
        this.backRight.setTargetPosition(ticks);

        this.setPower(power);
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