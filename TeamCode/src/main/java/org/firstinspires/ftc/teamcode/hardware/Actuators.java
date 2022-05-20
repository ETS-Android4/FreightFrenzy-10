package org.firstinspires.ftc.teamcode.hardware;

//import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_SERVO;
//import static org.firstinspires.ftc.teamcode.util.Constants.ODO_SERVO;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_BL;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_BR;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_FL;
import static org.firstinspires.ftc.teamcode.util.Constants.WHEEL_FR;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


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

    private int state;
    private double time;

    // hardware
    private DcMotor driveFL;
    private DcMotor driveFR;
    private DcMotor driveBL;
    private DcMotor driveBR;

//    private Servo intakeServo;
//    private Servo odoServo;

    public Actuators(HardwareMap hardwareMap) {
        this.driveBR = hardwareMap.get(DcMotor.class, WHEEL_BR);
        this.driveBL = hardwareMap.get(DcMotor.class, WHEEL_BL);
        this.driveFR = hardwareMap.get(DcMotor.class, WHEEL_FR);
        this.driveFL = hardwareMap.get(DcMotor.class, WHEEL_FL);

        //this.intakeServo = hardwareMap.get(Servo.class, INTAKE_SERVO);
        //this.odoServo = hardwareMap.get(Servo.class, ODO_SERVO);

    }

    public void startup(){
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //driveBR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // pid update for motor and slides
    public void update(double x, double y, double z) {
        double powerFL, powerFR, powerBL, powerBR, temp;
        powerBR = (   -x     +z   ) ;
        powerBL = (   +x     +z   ) ;
        powerFR = (   +x     +z   ) ;
        powerFL = (   -x     +z   ) ;

        temp = Math.max(Math.max(Math.abs(x+y+z),Math.abs(x+y-z)),Math.max(Math.abs(x-y+z),Math.abs(x-y-z)));
        temp = Math.max(Math.max(Math.max(Math.abs(powerBR),Math.abs(powerBL)),Math.max(Math.abs(powerFR),Math.abs(powerFL))),temp);

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
//    public double getIntakeServo() {
//        return intakeServo.getPosition();
//    }
//
//    public void setIntakeServo(double position) {
//        intakeServo.setPosition(position);
//    }
//
//    // odometry
//    public double getOdoServo() {
//        return odoServo.getPosition();
//    }
//
//    public void setOdoServo(double position) {
//        odoServo.setPosition(position);
//    }


    // telemetry
    public String getTelemetry() {
        return String.format(Locale.US, "" +
                        "time = %s",
                timeElapsed
        );
    }
}
