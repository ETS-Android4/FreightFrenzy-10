package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.oldutil.MathUtil;

import static org.firstinspires.ftc.teamcode.oldutil.Constants.IMU_SENSOR;

// Class for the IMU Sensor on the Control Hub
public class Sensors {
    private final BNO055IMU imu;
    private final double initialGyroHeading;
    private double baseGyroHeading;

    // Constructor
    public Sensors(HardwareMap hardwareMap) {
        // initialize imu
        this.imu = hardwareMap.get(BNO055IMU.class, IMU_SENSOR);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        this.imu.initialize(parameters);

        resetGyroHeading();
        this.initialGyroHeading = baseGyroHeading;
    }

    // Reset the Gyro heading
    public void resetGyroHeading() {
        baseGyroHeading = getGyroHeading180();
    }

    // Reset the Gyro heading to the initial value
    public void resetGyroHeadingToInitial() {
        baseGyroHeading = initialGyroHeading;
    }

    // Get the heading out of 360 degrees (0 is default, counterclockwise is increasing from 0 to 360, clockwise is decreasing from 359 to 0)
    public double getGyroHeading360() {
        double euler =  getGyroHeading180();
        return MathUtil.piTo2Pi(euler);
    }

    // Get the heading out of 180 degrees (0 is default, counterclockwise is increasing from 0 to 180, clockwise is decreasing from to 0 to -180)
    public double getGyroHeading180() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - baseGyroHeading;
    }

    // Get Telemetry for the current heading
    public String getTelemetry() {
//        return String.format(Locale.US, "Heading: %.2f\nColor: %.2f %s", getGyroHeading360(), getColor(), Arrays.toString(getRGBA()));
//        return String.format(Locale.US, "Heading: %.2f\nDistance: %.2f", getGyroHeading360(), getDistance());
//        return String.format(Locale.US, "Heading: %.2f\nDistance: %.2f %.2f", getGyroHeading360(), getRawLight(), getLight());
        return "";
    }
}
