package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import static org.firstinspires.ftc.teamcode.util.Configurables.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_DROP_HIGH;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_DROP_LOW;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_DROP_MIDDLE;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_INIT;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_MID;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_START;
import static org.firstinspires.ftc.teamcode.util.Configurables.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Configurables.SERVO_MOVEMENT;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_CUTOFF;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_DROP_HIGH;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_DROP_LOW;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_DROP_MIDDLE;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_MOVEMENT;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_SPEED;
import static org.firstinspires.ftc.teamcode.util.MathUtil.clamp;

@Config
@TeleOp(name = "Red TeleOp", group = "Competition")
public class RedTeleOp extends OpMode {
    Controller driver1;
    Controller driver2;

    Robot robot;

    private int targetPos;
    private double servoPos;

    private boolean scoringLow = false;
    private int scoringLowPos = 0;

    @Override
    public void init() {
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        driver1 = new Controller(gamepad1);
        driver2 = new Controller(gamepad2);

        robot = new Robot(hardwareMap);
//        robot.camera.initBarcodeWebcam();

        targetPos = 0;
        servoPos = HOPPER_INIT;

        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slide.setTargetPosition(targetPos);
        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.setPower(SLIDE_SPEED);

        robot.hopper.setPosition(servoPos);
    }

//    @Override
//    public void init_loop() {
//        if (robot.camera.getFrameCount() > 0) {
//            telemetry.addLine("Initialized");
//            telemetry.update();
//        }
//    }

    @Override
    public void loop() {
        driver1.update();
        driver2.update();

        // wheels
        double x, y, z;
        if (driver1.getLeftBumper().isPressed()) {
            x = driver1.getLeftStick().getX();
            y = driver1.getLeftStick().getY();
            z = driver1.getRightStick().getX();
        } else {
            x = driver1.getLeftStick().getX() * DRIVE_SPEED;
            y = driver1.getLeftStick().getY() * DRIVE_SPEED;
            z = driver1.getRightStick().getX() * DRIVE_SPEED;
        }

        setWheels(x, y, z);

        // intake
        robot.intake.setPower(driver2.getRightTrigger().getValue()*INTAKE_SPEED);
        if (robot.intake.getPower() == 0) {
            robot.intake.setPower(-driver2.getLeftTrigger().getValue()*INTAKE_SPEED);
        }

        // slide
        targetPos -= driver2.getLeftStick().getY()*SLIDE_MOVEMENT;
        targetPos = clamp(targetPos, -2400, 0);

        // hopper
        servoPos += driver2.getRightStick().getY()/ SERVO_MOVEMENT;

        if (driver2.getDUp().isJustPressed()) {
            targetPos = SLIDE_DROP_HIGH;
        } else if (driver2.getDLeft().isJustPressed()) {
            targetPos = SLIDE_DROP_MIDDLE;
        } else if (driver2.getDDown().isJustPressed()) {
//            scoringLow = true;
        } else if (driver2.getY().isJustPressed()) {
            if (targetPos == SLIDE_DROP_HIGH) {
                servoPos = HOPPER_DROP_HIGH;
            } else if (targetPos == SLIDE_DROP_MIDDLE) {
                servoPos = HOPPER_DROP_MIDDLE;
            } else if (targetPos == SLIDE_DROP_LOW) {
                servoPos = HOPPER_DROP_LOW;
            } else {
                servoPos = HOPPER_DROP_MIDDLE;
            }
        } else if (driver2.getX().isJustPressed()) {
            servoPos = HOPPER_MID.l;
        }

//        if (scoringLow) {
//            switch (scoringLowPos) {
//                case 0:
//                    targetPos = SLIDE_DROP_MIDDLE;
//                    servoPos = HOPPER_DROP_LOW_POS1;
//                    if (about(robot.slide.getCurrentPosition(), SLIDE_DROP_MIDDLE)) {
//                        scoringLowPos++;
//                    }
//                    break;
//                case 1:
//                    targetPos = SLIDE_DROP_LOW;
//                    if (about(robot.slide.getCurrentPosition(), SLIDE_DROP_LOW)) {
//                        scoringLowPos++;
//                    }
//                    break;
//                case 2:
//                    servoPos = HOPPER_DROP_LOW;
////                    if (wait a little) {
////                        ++
////                    }
//                    break;
//                case 3:
//                    servoPos =
//            }
//        }


        if (targetPos > SLIDE_CUTOFF) {
            servoPos = clamp(servoPos, HOPPER_START.l, HOPPER_START.u);
        } else {
            servoPos = clamp(servoPos, HOPPER_MID.l, HOPPER_MID.u);
        }

        robot.slide.setTargetPosition(targetPos);
        robot.hopper.setPosition(servoPos);

//        if (driver2.getRightStick().getY() < -0.1 || 0.1 < driver2.getRightStick().getY()) {
//            servoPos += driver2.getRightStick().getY()/500.0;
//            servoPos = clamp(servoPos, 0.01, 0.99);
//            robot.hopper.setPosition(servoPos);
//        }

        // 0 0.1504
        //  going up 0.305 or .3262
        // -2400 0.6531
        // -943 0.6801 (low)
        // -1411 0.6642 (middle)

        // ducky
        if (driver2.getA().isPressed()) {
            robot.ducky.setPower(-1);
        } else if (driver2.getB().isPressed()) {
            robot.ducky.setPower(1);
        } else {
            robot.ducky.setPower(0);
        }

        // telemetry
        telemetry.addData("servo pos", servoPos);
        telemetry.addData("slide pos", targetPos);
        telemetry.update();
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
        robot.frontLeft.setPower(flPower); robot.frontRight.setPower(frPower);
        robot.backLeft.setPower(blPower);  robot.backRight.setPower(brPower);
    }
}
