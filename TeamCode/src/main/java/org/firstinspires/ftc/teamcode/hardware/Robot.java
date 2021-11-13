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

public class Robot {
    private FtcDashboard dashboard;
    public Camera camera;
    String telemetry;

    public MecanumDrive drive;
    public Sensors sensors;

    public DcMotor intake;
    public DcMotor slide;
    public Servo hopper;
    public CRServo ducky;

    public Robot(HardwareMap hardwareMap) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        camera = new Camera(hardwareMap);

        drive = new MecanumDrive(hardwareMap);
        sensors = new Sensors(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, INTAKE);
        slide = hardwareMap.get(DcMotor.class, SLIDE);
        hopper = hardwareMap.get(Servo.class, HOPPER);
        ducky = hardwareMap.get(CRServo.class, DUCK_SPINNER);
    }

    public String getTelemetry() {
        telemetry = String.format(Locale.US, "Slide Position %s\nServo Position %s\n%s\n%s",
                slide.getCurrentPosition(), hopper.getPosition(), camera.getTelemetry(), sensors.getGyroHeading360());
        return telemetry;
    }
}
