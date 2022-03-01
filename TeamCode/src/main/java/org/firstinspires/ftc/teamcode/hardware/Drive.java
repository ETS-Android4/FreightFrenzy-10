package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import lombok.Builder;

public class Drive {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final BNO055IMU imu;

    @Builder
    private Drive(
            DcMotor frontLeft,
            DcMotor frontRight,
            DcMotor backLeft,
            DcMotor backRight,
            BNO055IMU imu) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.imu = imu;
        this.initialize();
    }

    private void initialize() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        this.imu.initialize(parameters);
    }

    public void setInput(Gamepad gamepad) {
        // Read the things we care about from the gamepad
        boolean turbo = gamepad.left_trigger > 0.2;
        boolean slow = gamepad.right_trigger > 0.2;
        double drive = -gamepad.right_stick_y;
        double turn  =  gamepad.left_stick_x;
        double difference = 0;

        // Mix the two joysticks
        double left  = drive + turn;
        double right = drive - turn;

        // Normalize the resultant values. We don't want values outside [-1, 1].
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Scale the result by the appropriate modifier (Default, Turbo, Slow).
        if (turbo) {
            left *= Constants.TURBO_MODE_MAX_POWER;
            right *= Constants.TURBO_MODE_MAX_POWER;
        } else if (slow || Robot.Constants.autoSlowdown == 1) {
            left *= Constants.SLOW_MODE_MAX_POWER;
            right *= Constants.SLOW_MODE_MAX_POWER;
            difference = left-right;
            left = left+difference;
            right = right-difference;
        } else {
            left *= Constants.DEFAULT_MAX_POWER;
            right *= Constants.DEFAULT_MAX_POWER;
        }

        // Set the calculated motor powers on each of the four motors.
        this.frontLeft.setPower(left);
        this.backLeft.setPower(left);
        this.frontRight.setPower(right);
        this.backRight.setPower(right);
//        telemetry.addData("FL", frontLeft.getCurrentPosition());
//        telemetry.addData("FR", frontRight.getCurrentPosition());
//        telemetry.addData("BL", backRight.getCurrentPosition());
//        telemetry.addData("BR", backLeft.getCurrentPosition());
//        telemetry.update();
    }

    // Move in two directions a certain number of inches
    public void setTargetPositionRelative(double distance, double power) {
        double ticks = (distance / Constants.WHEEL_CIRCUMFERENCE) * 537.7;

        this.setRunMode(RunMode.STOP_AND_RESET_ENCODER);

        this.frontLeft.setTargetPosition((int) ticks);
        this.frontRight.setTargetPosition((int) ticks);
        this.backLeft.setTargetPosition((int) ticks);
        this.backRight.setTargetPosition((int) ticks);

        this.setRunMode(RunMode.RUN_TO_POSITION);

        this.setPower(power);
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

    public void setPower(double power) {
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(power);
        this.backRight.setPower(power);
    }

    public boolean isBusy() {
        return this.getRunMode() != RunMode.RUN_TO_POSITION
                || this.frontLeft.isBusy()
                || this.frontRight.isBusy()
                || this.backLeft.isBusy()
                || this.backRight.isBusy();
    }

    public void turn(double degrees) {
        double currentHeadingRad = imu.getAngularOrientation().thirdAngle; //radians
        double currentHeadingDeg = currentHeadingRad * (180/Math.PI);
        frontLeft.setPower(-0.5);
        backLeft.setPower(-0.5);
        frontRight.setPower(0.5);
        frontRight.setPower(0.5);
    }
    public String getTelemetry() {
        String telemetry;
        telemetry = String.format("FL %s \n FR %s \n BL %s \n BR %s", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition());
        return telemetry;
    }
    public static class Constants {
        public static final double DEFAULT_MAX_POWER = 0.6;
        public static final double SLOW_MODE_MAX_POWER = 0.2;
        public static final double TURBO_MODE_MAX_POWER = 1.0;
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * 6;
    }
}