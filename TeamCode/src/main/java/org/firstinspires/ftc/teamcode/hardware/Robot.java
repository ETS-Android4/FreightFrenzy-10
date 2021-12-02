package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.oldutil.Constants.DUCK_SPINNER;
import static org.firstinspires.ftc.teamcode.oldutil.Constants.HOPPER;
import static org.firstinspires.ftc.teamcode.oldutil.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.oldutil.Constants.SLIDE;

public class Robot {
    private FtcDashboard dashboard;
    public Camera camera;
    String telemetry;

    public SampleMecanumDrive drive;
    public Sensors sensors;

    public DcMotor intake;
    public DcMotor slide;
    public Servo hopper;
    public CRServo ducky;
    public Servo capper;

    public Robot(HardwareMap hardwareMap) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        camera = new Camera(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
//        sensors = new Sensors(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, INTAKE);
        slide = hardwareMap.get(DcMotor.class, SLIDE);
        hopper = hardwareMap.get(Servo.class, HOPPER);
        ducky = hardwareMap.get(CRServo.class, DUCK_SPINNER);
        capper = hardwareMap.get(Servo.class, "capper");
    }

    public String getTelemetry() {
        telemetry = String.format(Locale.US, "Slide Position %s\nServo Position %s\n%s",
                slide.getCurrentPosition(), hopper.getPosition(), camera.getTelemetry());
        return telemetry;
    }
}
