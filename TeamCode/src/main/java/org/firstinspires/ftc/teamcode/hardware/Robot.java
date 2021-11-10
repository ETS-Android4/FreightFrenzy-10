package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.util.Constants.DUCK_SPINNER;
import static org.firstinspires.ftc.teamcode.util.Constants.HOPPER;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.util.Constants.SLIDE;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_BACK_LEFT;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_BACK_RIGHT;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_FRONT_LEFT;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_FRONT_RIGHT;

public class Robot {
    private FtcDashboard dashboard;
    public Camera camera;
    String telemetry;

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor intake;
    public DcMotor slide;
    public Servo hopper;
    public CRServo ducky;

    public Robot(HardwareMap hardwareMap) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
//        camera = new Camera(hardwareMap);

        frontLeft = hardwareMap.get(DcMotor.class, WHEEL_FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, WHEEL_FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, WHEEL_BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, WHEEL_BACK_RIGHT);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = hardwareMap.get(DcMotor.class, INTAKE);
        slide = hardwareMap.get(DcMotor.class, SLIDE);
        hopper = hardwareMap.get(Servo.class, HOPPER);
        ducky = hardwareMap.get(CRServo.class, DUCK_SPINNER);
    }

    public String getTelemetry() {
        telemetry = String.format(Locale.US, "Slide Position %s\nServo Position %s\n%s",
                slide.getCurrentPosition(), hopper.getPosition(), camera.getTelemetry());
        return telemetry;
    }
}
