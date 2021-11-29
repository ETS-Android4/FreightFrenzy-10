package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import lombok.Builder;

public class Actuators {
    private final DcMotor intakeMotor;
    private final Servo hopperServo;
    private final DcMotor linearSlideMotor;
    private final CRServo duckWheelServo;
    private final Servo elementHolderServo;

    private double targetHopperPosition;
    private double intakePower;
    private double linearSlidePower;
    private double duckWheelServoPower;

    @Builder
    private Actuators(
            DcMotor intakeMotor,
            Servo hopperServo,
            DcMotor linearSlideMotor,
            CRServo duckWheelServo,
            Servo elementHolderServo) {
        this.intakeMotor = intakeMotor;
        this.hopperServo = hopperServo;
        this.linearSlideMotor = linearSlideMotor;
        this.duckWheelServo = duckWheelServo;
        this.elementHolderServo = elementHolderServo;
        this.initialize();
    }

    private void initialize() {
        this.linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.elementHolderServo.setPosition(0.25);

        this.hopperServo.scaleRange(Constants.HOPPER_RANGE_MIN, Constants.HOPPER_RANGE_MAX);
    }

    public void setInput(Gamepad gamepad) {
        boolean intakePress = gamepad.right_bumper;
        boolean slideUp = gamepad.y;
        boolean slideDown = gamepad.x;
        double duckSpinLeft = gamepad.left_trigger;
        double duckSpinRight = gamepad.right_trigger;
        boolean hopperPress = gamepad.a;

        // Intake
        if(intakePress) {
            this.intakePower = 1;
            this.targetHopperPosition = 0;
        } else {
            this.intakePower = 0;
            this.targetHopperPosition = 0.5;
        }

        // Hopper
        if(hopperPress) {
            this.targetHopperPosition = 1.0;
        }

        // Linear Slide
        int currentPosition = this.linearSlideMotor.getCurrentPosition();
        if(slideUp && currentPosition > Constants.LINEAR_SLIDE_MAX_POSITION) {
            this.linearSlidePower = -1;
        } else if(slideDown && currentPosition < Constants.LINEAR_SLIDE_DEADZONE) {
            this.linearSlidePower = 1;
        } else{
            this.linearSlidePower = 0;
        }

        // Duck Wheel
        this.duckWheelServoPower = duckSpinLeft - duckSpinRight;

        this.setPower();
    }

    private void setPower() {
      this.intakeMotor.setPower(this.intakePower);
      this.hopperServo.setPosition(this.targetHopperPosition);
      this.linearSlideMotor.setPower(this.linearSlidePower);
      this.duckWheelServo.setPower(this.duckWheelServoPower);
    }

    public static class Constants {
        public static final double HOPPER_RANGE_MIN = 0.25;
        public static final double HOPPER_RANGE_MAX = 1.0;

        private static final double LINEAR_SLIDE_MAX_ROTATIONS = 5.8;
        private static final double LINEAR_SLIDE_ROTATION_SCALE = 537.7;
        public static final double LINEAR_SLIDE_MAX_POSITION =
                -1 * LINEAR_SLIDE_MAX_ROTATIONS * LINEAR_SLIDE_ROTATION_SCALE;
        public static final int LINEAR_SLIDE_DEADZONE = -200;
        public static final int LINEAR_SLIDE_BOTTOM_POSITION = 0;
        public static final int LINEAR_SLIDE_MIDDLE_POSITION = -750;
        public static final int LINEAR_SLIDE_TOP_POSITION = -1750;
    }
}
