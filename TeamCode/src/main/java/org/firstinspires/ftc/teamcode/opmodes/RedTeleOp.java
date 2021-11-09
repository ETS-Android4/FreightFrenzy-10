package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.Controller;

@TeleOp(name = "Red TeleOp", group = "Competition")
public class RedTeleOp extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor intake;
    private DcMotor slide;
    private Servo hopper;
    private CRServo ducky;

    private int targetPos;
    private double servoPos;

    Controller driver1;
    Controller driver2;

    // robot initialization
    @Override
    public void init() {
        driver1 = new Controller(gamepad1);
        driver2 = new Controller(gamepad2);

        // drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // intake
        intake = hardwareMap.get(DcMotor.class, "intake");

        // drop off
        targetPos = 0;
        servoPos = 0.01;

        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(targetPos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.5);

        hopper = hardwareMap.get(Servo.class, "hopper");
        hopper.setPosition(0.01);

        // ducky
        ducky = hardwareMap.get(CRServo.class, "ducky");
    }

    @Override
    public void loop() {
        driver1.update();
        driver2.update();

        // driver 1
        double x = driver1.getLeftStick().getX();
        double y = driver1.getLeftStick().getY();
        double z = driver1.getRightStick().getX();

        setWheels(x, y, z);

        // driver 2

        // intake
        intake.setPower(driver2.getRightTrigger().getValue()*.75);

        // slide
        targetPos -= driver2.getLeftStick().getY()*10;
        targetPos = Math.min(0, (int)Math.max(-384.5*3.1, targetPos));
        slide.setTargetPosition(targetPos);

        // hopper
        if (-0.1 < driver2.getRightStick().getY() || driver2.getRightStick().getY() > 0.1) {
            servoPos += driver2.getRightStick().getY()/2500;
            servoPos = Math.max(Math.min(.99,servoPos), 0.01);
            hopper.setPosition(servoPos);
        }

        // ducky
        if (driver2.getA().isPressed()) {
            ducky.setPower(-1);
        } else if (driver2.getB().isPressed()) {
            ducky.setPower(1);
        } else {
            ducky.setPower(0);
        }

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
        frontLeft.setPower(flPower); frontRight.setPower(frPower);
        backLeft.setPower(blPower);  backRight.setPower(brPower);
    }
}
