package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.CameraPosition;

import java.util.Locale;

public class Robot {
    String telemetry;

    public SampleMecanumDrive drive;
    public Actuators actuators;
    public Camera camera;

    public Robot(HardwareMap hardwareMap, CameraPosition cameraPosition) {
        drive = new SampleMecanumDrive(hardwareMap);
        actuators = new Actuators(hardwareMap);
        camera = new Camera(hardwareMap, cameraPosition);
        camera.initBarcodeWebcam();
    }

    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        actuators = new Actuators(hardwareMap);
    }

    public String getTelemetry() {
//        telemetry = String.format(Locale.US, "%s", actuators.getTelemetry());
        telemetry = String.format(Locale.US, "%s\n%s", actuators.getTelemetry(), camera.getTelemetry());
        return telemetry;
    }
}
