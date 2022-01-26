package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.Locale;

public class Robot {
    String telemetry;

    public SampleMecanumDrive drive;
    public Actuators actuators;
    public Camera camera;

    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        actuators = new Actuators(hardwareMap);
//        camera = new Camera(hardwareMap);
//        camera.initBarcodeWebcam();
    }

    public String getTelemetry() {
        telemetry = String.format(Locale.US, "%s", actuators.getTelemetry());
//        telemetry = String.format(Locale.US, "%s\n%s", actuators.getTelemetry(), camera.getTelemetry());
        return telemetry;
    }
}
