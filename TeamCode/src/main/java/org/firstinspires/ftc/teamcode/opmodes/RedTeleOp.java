package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import static org.firstinspires.ftc.teamcode.util.MathUtil.clamp;

@TeleOp(name = "Red TeleOp", group = "Competition")
public class RedTeleOp extends OpMode {
    Controller driver1;
    Controller driver2;

    Robot robot;

    private int targetPos;
    private double servoPos;

    @Override
    public void init() {
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        driver1 = new Controller(gamepad1);
        driver2 = new Controller(gamepad2);

        robot = new Robot(hardwareMap);
//        robot.camera.initBarcodeWebcam();

        targetPos = 0;
        servoPos = 0.01;

        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slide.setTargetPosition(targetPos);
        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.setPower(0.5);

        robot.hopper.setPosition(0.2);
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
        double x = driver1.getLeftStick().getX();
        double y = driver1.getLeftStick().getY();
        double z = driver1.getRightStick().getX();

        setWheels(x, y, z);

        // intake
        robot.intake.setPower(driver2.getRightTrigger().getValue()*.75);

        // slide
        targetPos -= driver2.getLeftStick().getY()*10;
        targetPos = clamp(targetPos, -2400, 0);
        robot.slide.setTargetPosition(targetPos);

        // hopper
        if (driver2.getRightStick().getY() < -0.1 || 0.1 < driver2.getRightStick().getY()) {
            servoPos += driver2.getRightStick().getY()/500.0;
            servoPos = clamp(servoPos, 0.01, 0.99);
            robot.hopper.setPosition(servoPos);
        }

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
