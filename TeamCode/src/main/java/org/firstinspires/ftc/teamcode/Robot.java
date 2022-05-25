package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

public class Robot {
    String telemetry;

    public MecanumDrive drive;
    public DcMotor intake;
    public DcMotor slide;
    public Servo hopper;
    public Servo leftSweeper;
    public Servo rightSweeper;
    public Servo flipperServo;
    public DcMotor leftFlipper;
    public DcMotor rightFlipper;

    public Robot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap);
//        intake = hardwareMap.get(DcMotor.class, "intake");
//        slide = hardwareMap.get(DcMotor.class, "slide");
//        hopper = hardwareMap.get(Servo.class, "hopper");
        leftSweeper = hardwareMap.get(Servo.class, "leftSweeper");
        rightSweeper = hardwareMap.get(Servo.class, "rightSweeper");
        flipperServo = hardwareMap.get(Servo.class, "flipperServo");
        leftFlipper = hardwareMap.get(DcMotor.class, "leftFlipper");
        rightFlipper = hardwareMap.get(DcMotor.class, "rightFlipper");

//        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFlipper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFlipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightFlipper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public String getTelemetry() {
        telemetry = "";
//        telemetry = String.format(Locale.US, "%s\n" +
//                        "Intake: Pow %.2f Pos %s\n" +
//                        "Slides: Pow %.2f Pos %s\n" +
//                        "Hopper:          Pos %s\n" +
//                        "LeftSweeper:     Pos %s\n" +
//                        "RightSweeper:    Pos %s\n" +
//                        "FlipperServo:    Pos %s\n" +
//                        "LeftFlipper:     Pos %s\n",
////                        "RightFlipper:    Pos %s\n",
//                    drive.getTelemetry(),
//                intake.getPower(), intake.getCurrentPosition(),
//                slide.getPower(), slide.getCurrentPosition(),
//                hopper.getPosition(),
//                leftSweeper.getPosition(), rightSweeper.getPosition(),
//                flipperServo.getPosition());
////                leftFlipper.getCurrentPosition(), rightFlipper.getCurrentPosition());
        return telemetry;
    }
}
