package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import lombok.Builder;

public class Drive {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    @Builder
    private Drive(
            DcMotor frontLeft,
            DcMotor frontRight,
            DcMotor backLeft,
            DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.initialize();
    }

    private void initialize() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // Put any hardware initialization that may be required here
    }

    public void setInput(Gamepad gamepad) {
        // Read the things we care about from the gamepad
        boolean turbo = gamepad.left_trigger > 0.2;
        boolean slow = gamepad.right_trigger > 0.2;
        double drive = -gamepad.right_stick_y;
        double turn  =  gamepad.left_stick_x;

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
        } else if (slow) {
            left *= Constants.SLOW_MODE_MAX_POWER;
            right *= Constants.SLOW_MODE_MAX_POWER;
        } else {
            left *= Constants.DEFAULT_MAX_POWER;
            right *= Constants.DEFAULT_MAX_POWER;
        }

        // Set the calculated motor powers on each of the four motors.
        this.frontLeft.setPower(left);
        this.backLeft.setPower(left);
        this.frontRight.setPower(right);
        this.backRight.setPower(right);
    }

    public static class Constants {
        public static final double DEFAULT_MAX_POWER = 0.6;
        public static final double SLOW_MODE_MAX_POWER = 0.2;
        public static final double TURBO_MODE_MAX_POWER = 1.0;
    }
}