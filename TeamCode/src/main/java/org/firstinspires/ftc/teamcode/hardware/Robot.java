package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

import lombok.Getter;

public class Robot {
    private HardwareMap hardwareMap;
    private Drive drive;
    public SampleTankDrive drive2;
    @Getter
    private Actuators actuators;

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.initialize();
    }

    private void initialize() {
        this.drive = Drive.builder()
                .frontLeft(this.hardwareMap.get(DcMotor.class, Constants.FRONT_LEFT_DRIVE_NAME))
                .frontRight(this.hardwareMap.get(DcMotor.class, Constants.FRONT_RIGHT_DRIVE_NAME))
                .backLeft(this.hardwareMap.get(DcMotor.class, Constants.BACK_LEFT_DRIVE_NAME))
                .backRight(this.hardwareMap.get(DcMotor.class, Constants.BACK_RIGHT_DRIVE_NAME))
                .imu(this.hardwareMap.get(BNO055IMU.class, Constants.IMU_NAME))
                .build();

        this.actuators = Actuators.builder()
                .intakeMotor(this.hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR_NAME))
                .hopperServo(this.hardwareMap.get(Servo.class, Constants.HOPPER_NAME))
                .linearSlideMotor(this.hardwareMap.get(DcMotor.class, Constants.LINEAR_SLIDE_NAME))
                .duckWheelServo(this.hardwareMap.get(CRServo.class, Constants.DUCK_WHEEL_NAME))
                .elementHolderServo(this.hardwareMap.get(Servo.class,Constants.ELEMENT_HOLDER_NAME))
                .build();
        drive2 = new SampleTankDrive(hardwareMap);
    }

    public void setInput(Gamepad gamepad1, Gamepad gamepad2) {
        this.drive.setInput(gamepad1);
        this.actuators.setInput(gamepad2);
    }

    public static class Constants {
        // Drive Base
        public static final String FRONT_LEFT_DRIVE_NAME = "driveFrontLeft";
        public static final String FRONT_RIGHT_DRIVE_NAME = "driveFrontRight";
        public static final String BACK_LEFT_DRIVE_NAME = "driveBackLeft";
        public static final String BACK_RIGHT_DRIVE_NAME = "driveBackRight";

        // Actuators
        public static final String INTAKE_MOTOR_NAME = "intakeMotor";
        public static final String HOPPER_NAME = "hopper";
        public static final String LINEAR_SLIDE_NAME = "linearSlide";
        public static final String DUCK_WHEEL_NAME = "duckWheel";
        public static final String ELEMENT_HOLDER_NAME = "elementHolder";
        public static final String IMU_NAME = "imu";
    }
}
