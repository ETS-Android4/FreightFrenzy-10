package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CameraPosition;

import java.util.Locale;

public class Robot {

    String telemetry;
    public Alliance alliance;

    public SampleMecanumDrive drive;
    public Actuators actuators;
    //public Camera camera;
    //public Lights lights;

    public Robot(HardwareMap hardwareMap, CameraPosition cameraPosition, Alliance alliance) {
        drive = new SampleMecanumDrive(hardwareMap);
        actuators = new Actuators(hardwareMap);
        //lights = new Lights(hardwareMap);
        //camera = new Camera(hardwareMap, cameraPosition);
        //camera.initBarcodeWebcam();
        this.alliance = alliance;
    }

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        drive = new SampleMecanumDrive(hardwareMap);
        actuators = new Actuators(hardwareMap);
        //lights = new Lights(hardwareMap);
        this.alliance = alliance;
    }


    public String getTelemetry() {
        telemetry = "nope";//String.format(Locale.US, "%s\n%s\n%s", drive.getTelemetry(), actuators.getTelemetry(), lights.getTelemetry());
        return telemetry;
    }

    public String getTelemetryWithCamera() {
        telemetry = "nope"; //String.format(Locale.US, "%s\n%s\n%s\n%s", camera.getTelemetry(), drive.getTelemetry(), actuators.getTelemetry(), lights.getTelemetry());
        return telemetry;
    }
}
