package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.util.Constants.HOPPER_SERVO;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.util.Constants.LEFT_DUCKY;
import static org.firstinspires.ftc.teamcode.util.Constants.RIGHT_DUCKY;
import static org.firstinspires.ftc.teamcode.util.Constants.SLIDES;
import static org.firstinspires.ftc.teamcode.util.Constants.SLIDES_SERVO;
import static org.firstinspires.ftc.teamcode.util.Constants.TURRET;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Range;

import java.util.Locale;

@Config
public class Actuators {
    public static int TURRET_SPEED = 5;
    public static int SLIDES_SPEED = 10;
    public static double ARM_HOPPER_SPEED = 0.003;
    public static double ARM_PIVOT_SPEED = 0.003;

    public static Range TURRET_RANGE = new Range(-1000,1000);
    public static Range SLIDES_RANGE = new Range(0, 2400);
    public static Range ARM_HOPPER_RANGE = new Range(0.01, 0.99);
    public static Range ARM_PIVOT_RANGE = new Range(0.01, 0.99);

    public static double TURRET_TOLERANCE = 90;
    public static double SLIDES_TOLERANCE = 50000000;
    public static double INTAKE_TOLERANCE = 10;

    public static double DUCKY_SPEED = 1.0;

    public static PIDCoefficients TURRET_COEFFICIENTS = new PIDCoefficients(0.01, 0.00001, 0.00001);
    public static PIDCoefficients SLIDES_COEFFICIENTS = new PIDCoefficients(0.002, 0, 0);
    public static PIDCoefficients INTAKE_COEFFICIENTS = new PIDCoefficients(0.002, 0, 0);

    private PIDController turretController;
    private PIDController slidesController;
    private PIDController intakeController;

    private DcMotor intake;
    private DcMotor turret;
    private DcMotor slides;
    private Servo hopperServo;
    private Servo slidesServo;
    private CRServo leftDucky;
    private CRServo rightDucky;

    public Actuators(HardwareMap hardwareMap) {
        this.intake = hardwareMap.get(DcMotor.class, INTAKE);
        this.turret = hardwareMap.get(DcMotor.class, TURRET);
        this.slides = hardwareMap.get(DcMotor.class, SLIDES);
        this.hopperServo = hardwareMap.get(Servo.class, HOPPER_SERVO);
        this.slidesServo = hardwareMap.get(Servo.class, SLIDES_SERVO);
        this.leftDucky = hardwareMap.get(CRServo.class, LEFT_DUCKY);
        this.rightDucky = hardwareMap.get(CRServo.class, RIGHT_DUCKY);

        this.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        this.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.turret.setTargetPosition(0);
//        this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        this.slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.slides.setTargetPosition(0);
//        this.slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretController = new PIDController(TURRET_COEFFICIENTS.kP, TURRET_COEFFICIENTS.kI, TURRET_COEFFICIENTS.kD);
        slidesController = new PIDController(SLIDES_COEFFICIENTS.kP, SLIDES_COEFFICIENTS.kI, SLIDES_COEFFICIENTS.kD);
        intakeController = new PIDController(INTAKE_COEFFICIENTS.kP, INTAKE_COEFFICIENTS.kI, INTAKE_COEFFICIENTS.kD);
    }

    public void setIntake(double power) {
        this.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intake.setPower(power);
    }

    public void resetIntake() {
        double currentPosition = this.intake.getCurrentPosition();
        int newPosition = (int)(currentPosition - (currentPosition % 145.1));
//        this.intake.setTargetPosition(newPosition);
//        this.intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.intake.setPower(0.1);
        intakeController.setSetPoint(newPosition);
        intake.setPower(intakeController.calculate());
    }

    public void setTurret(int position) {
        turretController.setSetPoint(position);
//        this.turret.setTargetPosition(position);
//        this.turret.setPower(TURRET_POWER);
    }

    public int getTurret() {
        return this.turret.getCurrentPosition();
    }

    public void setSlides(int position) {
        slidesController.setSetPoint(position);
//        this.slides.setTargetPosition(position);
//        this.slides.setPower(SLIDES_POWER);
    }

    public int getSlides() {
        return this.slides.getCurrentPosition();
    }

    public void update() {
        turretController.setPID(TURRET_COEFFICIENTS.kP, TURRET_COEFFICIENTS.kI, TURRET_COEFFICIENTS.kD);
        slidesController.setPID(SLIDES_COEFFICIENTS.kP, SLIDES_COEFFICIENTS.kI, SLIDES_COEFFICIENTS.kD);
        intakeController = new PIDController(INTAKE_COEFFICIENTS.kP, INTAKE_COEFFICIENTS.kI, INTAKE_COEFFICIENTS.kD);
        turretController.setTolerance(TURRET_TOLERANCE);
        slidesController.setTolerance(SLIDES_TOLERANCE);
        intakeController.setTolerance(INTAKE_TOLERANCE);
        this.turret.setPower(turretController.calculate(turret.getCurrentPosition()));
        this.slides.setPower(slidesController.calculate(slides.getCurrentPosition()));
    }

    public void setArmHopper(double position) {
        this.hopperServo.setPosition(position);
    }

    public double getArmHopper() {
        return this.hopperServo.getPosition();
    }

    public void setArmPivot(double position) {
        this.slidesServo.setPosition(position);
    }

    public double getArmPivot() {
        return this.slidesServo.getPosition();
    }

    public void setDuckies(double power, Alliance alliance) {
        this.leftDucky.setPower(alliance == Alliance.RED ? -power : power);
        this.rightDucky.setPower(alliance == Alliance.RED ? -power : power);
    }

    public void setLeftDucky(double power, Alliance alliance) {
        this.leftDucky.setPower(alliance == Alliance.RED ? -power : power);
    }

    public void setRightDucky(double power, Alliance alliance) {
        this.rightDucky.setPower(alliance == Alliance.RED ? -power : power);
    }

    public String getTelemetry() {
        return String.format(Locale.US, ""+
                        "Intake:      pos %s pow %.2f\n" +
                        "Turret:      pos %s pow %.2f err %.2f\n" +
                        "Slides:      pos %s pow %.2f\n" +
                        "HopperServo: pos %.2f\n" +
                        "SlidesServo: pos %.2f\n" +
                        "Duckies: left %.2f right %.2f",
                intake.getCurrentPosition(), intake.getPower(), turret.getCurrentPosition(), turret.getPower(), turretController.getPositionError(),
                slides.getCurrentPosition(), slides.getPower(), hopperServo.getPosition(), slidesServo.getPosition(),
                leftDucky.getPower(), rightDucky.getPower());
    }
}
