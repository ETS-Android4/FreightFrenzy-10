package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.util.Constants.DUCK_SPINNER;
import static org.firstinspires.ftc.teamcode.util.Constants.HOPPER;
import static org.firstinspires.ftc.teamcode.util.Constants.IMU_SENSOR;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.util.Constants.SLIDE;
import static org.firstinspires.ftc.teamcode.util.MathUtil.piTo2Pi;

public class Robot {
    private FtcDashboard dashboard;
    public Camera camera;
    String telemetry;

    public MecanumDrive drive;

    public DcMotor intake;
    public DcMotor slide;
    public Servo hopper;
    public CRServo ducky;

    private BNO055IMU imu;

    public Robot(HardwareMap hardwareMap) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        camera = new Camera(hardwareMap);

        drive = new MecanumDrive(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, INTAKE);
        slide = hardwareMap.get(DcMotor.class, SLIDE);
        hopper = hardwareMap.get(Servo.class, HOPPER);
        ducky = hardwareMap.get(CRServo.class, DUCK_SPINNER);

        this.imu = hardwareMap.get(BNO055IMU.class, IMU_SENSOR);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        this.imu.initialize(parameters);
    }

    private float baseGyroHeading;

    public void resetGyroHeading() {
        baseGyroHeading = getGyroHeading180();
    }

    public float getGyroHeading360() {
        float euler =  getGyroHeading180();
        return piTo2Pi(euler);
    }

    public float getGyroHeading180() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - baseGyroHeading;
    }

    public String getTelemetry() {
        telemetry = String.format(Locale.US, "Slide Position %s\nServo Position %s\n%s",
                slide.getCurrentPosition(), hopper.getPosition(), camera.getTelemetry());
        return telemetry;
    }
}
