package org.firstinspires.ftc.teamcode.hardware;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.opmodes.PoseStorage.intakeOffset;
import static org.firstinspires.ftc.teamcode.opmodes.PoseStorage.slidesOffset;
import static org.firstinspires.ftc.teamcode.opmodes.PoseStorage.turretOffset;
import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.LEFT;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.MIDDLE;
//import static org.firstinspires.ftc.teamcode.util.Constants.COLOR;
//import static org.firstinspires.ftc.teamcode.util.Constants.HOPPER_SERVO;
//import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_SERVO;
//import static org.firstinspires.ftc.teamcode.util.Constants.LEFT_DUCKY;
import static org.firstinspires.ftc.teamcode.util.Constants.ODO_SERVO;
//import static org.firstinspires.ftc.teamcode.util.Constants.RIGHT_DUCKY;
//import static org.firstinspires.ftc.teamcode.util.Constants.SLIDES;
//import static org.firstinspires.ftc.teamcode.util.Constants.PIVOT_SERVO;
//import static org.firstinspires.ftc.teamcode.util.Constants.TURRET;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_BL;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_BR;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_FL;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_FR;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.GENERAL;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.HIGH;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.LOW;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.MID;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.SHARED;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.ArmPosition;
import org.firstinspires.ftc.teamcode.util.BarcodeLocation;
import org.firstinspires.ftc.teamcode.util.DepositPosition;

import java.util.Locale;

@Config
public class Actuators {
    // misc
    public static double INTAKE_RESET_TIME = 1;
    public static double HOPPER_DISTANCE_CUTOFF = 25;
    public int intakeStartPos = (int) (145.1/8.0);
    public boolean hasBlock = false;
    double startTime = 0;
    public double timeElapsed = 0;

    // driver variables
    public static int TURRET_SPEED = 5;
    public static int SLIDES_SPEED = 50;
    public static double ARM_HOPPER_SPEED = 0.015;
    public static double ARM_PIVOT_SPEED = 0.005;
    public static double INTAKE_SERVO_SPEED = 0.02;
    public static double DUCKY_SPEED = 1.0;

    public static double INTAKE_SPEED = 0.6;
    public static double INTAKE_SLOW_SPEED = 0.6;


    public boolean odoRetracted = false;
    public boolean intakeRetracted;

    // pid variables
    private PIDController turretController;
    private PIDController slidesController;
    private PIDController intakeController;




    private int state;
    private double time;

    // hardware
    private DcMotor driveFL;
    private DcMotor driveFR;
    private DcMotor driveBL;
    private DcMotor driveBR;

    private Servo hopperServo;
    private Servo pivotServo;
    private CRServo leftDucky;
    private CRServo rightDucky;
    private Servo intakeServo;
    private Servo odoServo;



    public Actuators(HardwareMap hardwareMap) {
        this.driveBR = hardwareMap.get(DcMotor.class, WHEEL_BR);
        this.driveBL = hardwareMap.get(DcMotor.class, WHEEL_BL);
        this.driveFR = hardwareMap.get(DcMotor.class, WHEEL_FR);
        this.driveFL = hardwareMap.get(DcMotor.class, WHEEL_FL);


//        this.hopperServo = hardwareMap.get(Servo.class, HOPPER_SERVO);
//        this.pivotServo = hardwareMap.get(Servo.class, PIVOT_SERVO);
//        this.leftDucky = hardwareMap.get(CRServo.class, LEFT_DUCKY);
//        this.rightDucky = hardwareMap.get(CRServo.class, RIGHT_DUCKY);
        this.intakeServo = hardwareMap.get(Servo.class, INTAKE_SERVO);
        this.odoServo = hardwareMap.get(Servo.class, ODO_SERVO);


//        this.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void startup(){
        driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //driveBR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // pid update for motor and slides
    public void update(double x, double y, double z) {
        double powerFL, powerFR, powerBL, powerBR, temp;
        powerBR = (   +x   -y   +z   ) ;
        powerBL = (   -x   +y   +z   ) ;
        powerFR = (   +x   +y   +z   ) ;
        powerFL = (   -x   -y   +z   ) ;

        temp = Math.abs(Math.max(Math.max(powerBR,powerBL),Math.max(powerFR,powerFL)));
        if(Math.abs(temp)>1){
            powerBR = powerBR/temp;
            powerBL = powerBL/temp;
            powerFR = powerFR/temp;
            powerFL = powerFL/temp;
        }

        driveBR.setPower(powerBR);
        driveBL.setPower(powerBL);
        driveFR.setPower(powerFR);
        driveFL.setPower(powerFL);
    }


    // intake servo
    public double getIntakeServo() {
        return intakeServo.getPosition();
    }

    public void setIntakeServo(double position) {
        intakeServo.setPosition(position);
    }

    // odometry
    public double getOdoServo() {
        return odoServo.getPosition();
    }

    public void setOdoServo(double position) {
        odoServo.setPosition(position);
    }


    // telemetry
    public String getTelemetry() {
        return String.format(Locale.US, "" +
            "time = %s",
            timeElapsed
        );
    }
}
