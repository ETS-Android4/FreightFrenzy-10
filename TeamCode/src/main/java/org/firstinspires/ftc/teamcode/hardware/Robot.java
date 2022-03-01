package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.Lights.BLUE;
import static org.firstinspires.ftc.teamcode.hardware.Lights.BLUEFULL;
import static org.firstinspires.ftc.teamcode.hardware.Lights.BLUESCORING;
import static org.firstinspires.ftc.teamcode.hardware.Lights.RED;
import static org.firstinspires.ftc.teamcode.hardware.Lights.REDFULL;
import static org.firstinspires.ftc.teamcode.hardware.Lights.REDSCORING;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CameraPosition;

import java.util.Locale;

public class Robot {
    String telemetry;
    private Alliance alliance;

    public SampleMecanumDrive drive;
    public Actuators actuators;
    public Camera camera;
    public Lights lights;

    public Robot(HardwareMap hardwareMap, CameraPosition cameraPosition, Alliance alliance) {
        drive = new SampleMecanumDrive(hardwareMap);
        actuators = new Actuators(hardwareMap);
        lights = new Lights(hardwareMap);
        camera = new Camera(hardwareMap, cameraPosition);
        camera.initBarcodeWebcam();
        this.alliance = alliance;
    }

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        drive = new SampleMecanumDrive(hardwareMap);
        actuators = new Actuators(hardwareMap);
        lights = new Lights(hardwareMap);
        this.alliance = alliance;
    }

    public void updateLights() {
        if (alliance == Alliance.RED) {
            if (actuators.runningExtend || actuators.runningRetract) {
                lights.setPattern(REDSCORING);
            } else if (actuators.hopperIsFull()) {
                lights.setPattern(REDFULL);
            } else {
                lights.setPattern(RED);
            }
        } else if (alliance == Alliance.BLUE) {
            if (actuators.runningExtend || actuators.runningRetract) {
                lights.setPattern(BLUESCORING);
            } else if (actuators.hopperIsFull()) {
                lights.setPattern(BLUEFULL);
            } else {
                lights.setPattern(BLUE);
            }
        }
    }

    public String getTelemetry() {
//        telemetry = String.format(Locale.US, "%s", actuators.getTelemetry());
        telemetry = String.format(Locale.US, "%s\n%s", actuators.getTelemetry(), lights.getTelemetry());
        return telemetry;
    }
}
