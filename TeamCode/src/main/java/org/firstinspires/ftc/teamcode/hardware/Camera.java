package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.BarcodeLocation;
import org.firstinspires.ftc.teamcode.vision.BarcodePipeline;
import org.firstinspires.ftc.teamcode.vision.Detection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.UNKNOWN;
import static org.firstinspires.ftc.teamcode.util.Constants.BARCODE_WEBCAM;
import static org.firstinspires.ftc.teamcode.util.Constants.INVALID_DETECTION;
import static org.firstinspires.ftc.teamcode.util.Constants.WEBCAM_HEIGHT;
import static org.firstinspires.ftc.teamcode.util.Constants.WEBCAM_ROTATION;
import static org.firstinspires.ftc.teamcode.util.Constants.WEBCAM_WIDTH;

// Class for the camera
public class Camera {
    private HardwareMap hardwareMap;
    private OpenCvCamera barcodeWebcam;
    private BarcodePipeline barcodePipeline;

    private boolean barcodeWebcamInitialized;

    // Constructor
    public Camera(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    // Initiate the Barcode Camera
    public void initBarcodeWebcam() {
        int stackCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.barcodeWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, BARCODE_WEBCAM), stackCameraMonitorViewId);
        this.barcodePipeline = new BarcodePipeline();
        barcodeWebcam.setPipeline(barcodePipeline);
        barcodeWebcam.openCameraDeviceAsync(() -> {
            barcodeWebcam.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, WEBCAM_ROTATION);
            barcodeWebcamInitialized = true;
        });
        FtcDashboard.getInstance().startCameraStream(barcodeWebcam, 0);
    }

    // Close the Barcode Camera
    public void stopBarcodeWebcam() {
        if (barcodeWebcamInitialized) {
            barcodeWebcam.closeCameraDeviceAsync(() -> barcodeWebcamInitialized = false);
        }
    }

    // Check what StarterStack configuration is on the field
    public BarcodeLocation checkTeamElementLocation() {
        return (barcodeWebcamInitialized ? barcodePipeline.getTeamElementLocation() : UNKNOWN);
    }

    // Get the StarterStack Detection
    public Detection getTeamElement() {
        return (barcodeWebcamInitialized ? barcodePipeline.getTeamElement() : INVALID_DETECTION);
    }

    // Get the number of frames the current camera has processed
    public int getFrameCount() {
        if (barcodeWebcamInitialized) {
            return barcodeWebcam.getFrameCount();
        } else {
            return 0;
        }
    }

    // Get Telemetry for the current active camera
    public String getTelemetry() {
        if (barcodeWebcamInitialized) {
            return String.format(Locale.US, "Barcode Location: %s\nSize: %.4f", checkTeamElementLocation(), getTeamElement().getArea());
        }
        return ("No Camera Initialized");
    }
}