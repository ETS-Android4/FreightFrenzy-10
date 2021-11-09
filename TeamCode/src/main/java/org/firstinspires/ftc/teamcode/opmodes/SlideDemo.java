package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.controller.Controller;

// starter linear slide demo
// (the name and group arguments are optional, they organize how the opmodes are shown on the driver station)
@TeleOp(name = "Slide Demo", group = "Development")
public class SlideDemo extends OpMode {
    Controller driver1;

    // instantiate motor variable
    private DcMotor slide = null;

    private boolean retracted = true;
    private double fullRotation = 384.5;
    private int targetPos = 0;
    private boolean moving = true;

    // robot initialization
    @Override
    public void init() {
        // initialize motor variables
        // (the names are what is set through the driver station)
        slide = hardwareMap.get(DcMotor.class, "slide");

        // set the default direction for the motors
        // (some are reversed because they face opposite directions)
        slide.setDirection(DcMotor.Direction.FORWARD);

        // set the motors to run without encoders
        // (encoders make the motor speeds more accurate, but require extra wires)
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setTargetPosition(0);

        driver1 = new Controller(gamepad1);
    }

    // main loop
    @Override
    public void loop() {
        driver1.update();

        // get input from the gamepad
        if (driver1.getDUp().isJustPressed()) {
            targetPos = (int)(fullRotation * 3.1);//6.25
            slide.setTargetPosition(targetPos);
            moving = true;
        } else if (driver1.getDDown().isJustPressed()) {
            targetPos = 0;
            slide.setTargetPosition(targetPos);
            moving = true;
        }
        if (Math.abs(Math.abs(slide.getCurrentPosition())-Math.abs(targetPos)) > 10) {
            moving = true;
        }
        if (moving) {
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);//vroom vroom
        } else {
            slide.setPower(0);
        }
        telemetry.addData("Current Position", slide.getCurrentPosition());
        telemetry.addData("Wanted Position", targetPos);
        telemetry.update();

////        double y = driver1.getLeftStick().getY();
//        double y = -gamepad1.left_stick_y;
//        if (Math.abs(y) > 0.2) {
//            slide.setPower(y/2);
//        } else {
//            slide.setPower(0);
//        }
//        telemetry.addData("Wanted Power", y);
//        telemetry.addData("Power", slide.getPower());
//        telemetry.update();
    }
}
