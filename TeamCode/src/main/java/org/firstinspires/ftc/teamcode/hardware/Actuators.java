package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import lombok.Builder;
import lombok.Data;
import lombok.Getter;
import lombok.Setter;
import lombok.Value;

public class Actuators {
    private final DcMotor intakeMotor;
    private final Servo hopperServo;
    private final DcMotor linearSlideMotor;
    private final CRServo duckWheelServo;
    private final Servo elementHolderServo;
    public RevColorSensorV3 hopperSensor;
    public RevBlinkinLedDriver hopperLights;
    public RevBlinkinLedDriver.BlinkinPattern hopperPatternBlock;
    public RevBlinkinLedDriver.BlinkinPattern hopperPatternNoBlock;
    public RevBlinkinLedDriver.BlinkinPattern hopperPatternNeutral;

    private double targetHopperPosition;
    private double intakePower;
    private double linearSlidePower;
    private double duckWheelServoPower;

    private boolean intakeWasPressed;
    private JiggleState jiggleState = JiggleState.Idle;

    private enum JiggleState { Idle, FirstUp, Down, SecondUp }
    private long jiggleStartTime;

    @Builder
    private Actuators(
            DcMotor intakeMotor,
            Servo hopperServo,
            DcMotor linearSlideMotor,
            CRServo duckWheelServo,
            Servo elementHolderServo,
            RevColorSensorV3 hopperSensor,
            RevBlinkinLedDriver hopperLights) {
        this.intakeMotor = intakeMotor;
        this.hopperServo = hopperServo;
        this.linearSlideMotor = linearSlideMotor;
        this.duckWheelServo = duckWheelServo;
        this.elementHolderServo = elementHolderServo;
        this.hopperSensor = hopperSensor;
        this.hopperLights = hopperLights;
        this.initialize();
    }

    private void initialize() {
        this.linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.elementHolderServo.scaleRange(0.08, 1.0);
        //this.elementHolderServo.setPosition(0);

        this.hopperServo.scaleRange(Constants.HOPPER_RANGE_MIN, Constants.HOPPER_RANGE_MAX);
        this.hopperPatternBlock = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        this.hopperPatternNoBlock = RevBlinkinLedDriver.BlinkinPattern.RED;
        this.hopperPatternNeutral = RevBlinkinLedDriver.BlinkinPattern.RED;
        hopperLights.setPattern(hopperPatternNeutral);

    }

    public void setInput(Gamepad gamepad) {
        boolean intakePress = gamepad.right_bumper;
        //New Stuff 4/5
        final double initLSPos = Math.abs(this.linearSlideMotor.getCurrentPosition());
        boolean slideUp = gamepad.y;
        boolean slideDown = gamepad.x;
        double duckSpinLeft = gamepad.left_trigger;
        double duckSpinRight = gamepad.right_trigger;
        boolean hopperPress = gamepad.a;
        double elementStick = gamepad.left_stick_y;

        // Middle Element Position
        if(gamepad.left_bumper){
            elementStick = .5;
        }

        // Intake
        if(intakePress) {
            this.intakePower = 1;
            this.targetHopperPosition = 0;
        } else {
            this.intakePower = 0;
            this.targetHopperPosition = .5;

        }

        if(!intakePress && this.intakeWasPressed){
            this.jiggleState = JiggleState.FirstUp;
            this.jiggleStartTime = System.currentTimeMillis();
        }

        // First up takes 300ms
//        if(this.jiggleState != JiggleState.Idle) {
//            switch(this.jiggleState) {
//                case FirstUp:
//                    if (System.currentTimeMillis() >= this.jiggleStartTime + 150) {
//                        this.jiggleState = JiggleState.Down;
//                    } else {
//                        // set servo position
//                        this.targetHopperPosition = .15;
//                    }
//                    break;
//                case Down:
//                    if (System.currentTimeMillis() >= this.jiggleStartTime + 350){
//                        this.jiggleState = JiggleState.SecondUp;
//                    } else {
//                        //set servo position
//                        this.targetHopperPosition = 0;
//                    }
//                    break;
//                case SecondUp:
//                    if (System.currentTimeMillis() >= this.jiggleStartTime + 850){
//                        this.jiggleState = JiggleState.Idle;
//                    } else {
//                        //set servo position
//                        this.targetHopperPosition = .5;
//                    }
//                    break;
//            }
//        }

        // Hopper
        if(hopperPress) {
            this.targetHopperPosition = 1.0;
        }

        //Hopper Lights
        double distance = hopperSensor.getDistance(DistanceUnit.MM);
        if(distance >= 10 && distance <= 55){
            hopperLights.setPattern(hopperPatternBlock);
        } else{
            hopperLights.setPattern(hopperPatternNeutral);
        }
        // Linear Slide
        int currentPosition = this.linearSlideMotor.getCurrentPosition();
        if(slideUp && currentPosition > Constants.LINEAR_SLIDE_MAX_POSITION) {
            this.linearSlidePower = -1;
            //New Stuff 5/5
        } else if(slideDown && this.linearSlideMotor.getCurrentPosition() < 125) { //&& currentPosition < Constants.LINEAR_SLIDE_DEADZONE
            this.linearSlidePower = 1;
        } else{
            this.linearSlidePower = 0;
        }

        // Duck Wheel
        this.duckWheelServoPower = duckSpinLeft - duckSpinRight;

        // Element Holder
        this.elementHolderServo.setPosition(elementStick);
        this.intakeWasPressed = intakePress;
        this.setPower();
    }

    private void setPower() {
        this.intakeMotor.setPower(this.intakePower);
        this.hopperServo.setPosition(this.targetHopperPosition);
        this.linearSlideMotor.setPower(this.linearSlidePower);
        this.duckWheelServo.setPower(this.duckWheelServoPower);
    }

    public void setLiftPosition(int pos) {
        this.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.linearSlideMotor.setTargetPosition(pos);
        this.linearSlideMotor.setPower(1);
    }

    public int getLiftPosition() {
        return this.linearSlideMotor.getCurrentPosition();
    }
    //New Stuff 1/5
    public String getLSTelemetry() {
        String telemetryLS;
        telemetryLS = String.format("LS %s \n %s %s %s", linearSlideMotor.getCurrentPosition(), hopperSensor.red(), hopperSensor.green(), hopperSensor.blue());
        return telemetryLS;
    }
    public static class Constants {
        public static final double HOPPER_RANGE_MIN = 0.2;
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