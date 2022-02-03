package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
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
import org.firstinspires.ftc.teamcode.util.ArmPosition;
import org.firstinspires.ftc.teamcode.util.Range;

import java.util.Locale;

@Config
public class Actuators {
    public static int TURRET_SPEED = 5;
    public static int SLIDES_SPEED = 10;
    public static double ARM_HOPPER_SPEED = 0.003;
    public static double ARM_PIVOT_SPEED = 0.003;

    public static Range TURRET_RANGE = new Range(-1000,1000);
    public static Range SLIDES_RANGE = new Range(0, 2500);
    public static Range ARM_HOPPER_RANGE = new Range(0.01, 0.99);
    public static Range ARM_PIVOT_RANGE = new Range(0.01, 0.99);

    public static double TURRET_TOLERANCE = 90;
    public static double SLIDES_TOLERANCE = 500;
    public static double INTAKE_TOLERANCE = 100;

    public static double DUCKY_SPEED = 1.0;

    public static PIDCoefficients TURRET_COEFFICIENTS = new PIDCoefficients(0.01, 0.00001, 0.00001);
    public static PIDCoefficients SLIDES_COEFFICIENTS = new PIDCoefficients(0.002, 0, 0);
    public static PIDCoefficients INTAKE_COEFFICIENTS = new PIDCoefficients(0.002, 0, 0);

    public static ArmPosition ARM_PIVOT_POSITION = new ArmPosition(0.01, 0.1, 0.42, 0.75);
    public static ArmPosition ARM_HOPPER_POSITION = new ArmPosition(0.65, 0.75, 0.74, 0.59);

    public static int TURRET_ALLIANCE = 650;
    public static int TURRET_SHARED = -800;
    public static int SLIDES_ALLIANCE = 2400;
    public static int SLIDES_SHARED = 0;

    public static double DEPOSIT1 = 0.4;
    public static double DEPOSIT2 = 0.7;
    public static double DEPOSIT3 = 0.6;
    public static double DEPOSIT4 = 1.3;

    public static double RETRACT1 = 0.4;
    public static double RETRACT2 = 0.6;
    public static double RETRACT3 = 0.6;
    public static double RETRACT4 = 0.6;
    public static double RETRACT5 = 0.25;

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

    // state machine variables
    public boolean runningAlliance;
    public boolean runningShared;
    public boolean runningDeposit;
    public boolean depositQueue;
    public boolean justFinishedAllianceMacro;
    public boolean justFinishedSharedMacro;
    public boolean justFinishedAMacro;

    private int state;
    private double time;

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

    public void setIntakePosition(int position){
//        this.intake.setTargetPosition(position);
//        this.intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.intake.setPower(Math.abs(position-getIntakePosition())/145.1*0.9);
        setIntakePositionPID(position);
    }

    public void setIntakePositionPID(int position) {
        intakeController.setPID(INTAKE_COEFFICIENTS.kP, INTAKE_COEFFICIENTS.kI, INTAKE_COEFFICIENTS.kD);
        intakeController.setTolerance(INTAKE_TOLERANCE);
        intakeController.setSetPoint(position);
        intake.setPower(intakeController.calculate(intake.getCurrentPosition()));
    }

    public int getIntakePosition() {
        return this.intake.getCurrentPosition();
    }

    public void resetIntake() {
        intakeController.setPID(INTAKE_COEFFICIENTS.kP, INTAKE_COEFFICIENTS.kI, INTAKE_COEFFICIENTS.kD);
        intakeController.setTolerance(INTAKE_TOLERANCE);
        intake.setPower(intakeController.calculate(intake.getCurrentPosition()));
    }

    //left here just in case renaming it to getIntakePosition breaks something in the future
    public int getIntake() {
        return this.intake.getCurrentPosition();
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
        turretController.setTolerance(TURRET_TOLERANCE);
        slidesController.setTolerance(SLIDES_TOLERANCE);
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

    public void runningAlliance(double currentTime, Alliance alliance) {
        if (runningAlliance) {
            switch (state) {
                case 0:
                    runningAlliance = true;
                    time = currentTime;
                    setArmPivot(ARM_PIVOT_POSITION.getAlmostDown());
                    setArmHopper(ARM_HOPPER_POSITION.getAlmostDown());
                    state++;
                    break;
                case 1:
                    if (currentTime > time + DEPOSIT1) {
                        state++;
                    }
                    break;
                case 2:
                    time = currentTime;
                    setArmPivot(ARM_PIVOT_POSITION.getUp());
                    setArmHopper(ARM_HOPPER_POSITION.getUp());
                    state++;
                    break;
                case 3:
                    if (currentTime > time + DEPOSIT2) {
                        state++;
                    }
                    break;
                case 4:
                    time = currentTime;
                    setTurret(alliance == RED ? TURRET_ALLIANCE : -TURRET_ALLIANCE);
                    state++;
                    break;
                case 5:
                    if (currentTime > time + DEPOSIT3) {
                        state++;
                    }
                    break;
                case 6:
                    time = currentTime;
                    setSlides(SLIDES_ALLIANCE);
                    setArmPivot(ARM_PIVOT_POSITION.getDeposit());
                    setArmHopper(.99);
                    state++;
                    break;
                case 7:
                    if (currentTime > time + DEPOSIT4) {
                        state++;
                    }
                    break;
                case 8:
                    runningAlliance = false;
                    justFinishedAllianceMacro = true;
                    if (depositQueue) {
                        depositQueue = false;
                        runningDeposit = true;
                    } else {
                        justFinishedAMacro = true;
                    }
                    state = 0;
            }
        }
    }

    public void runningShared(double currentTime, Alliance alliance) {
        if (runningShared) {
            switch (state) {
                case 0:
                    runningShared = true;
                    time = currentTime;
                    setArmPivot(ARM_PIVOT_POSITION.getAlmostDown());
                    setArmHopper(ARM_HOPPER_POSITION.getAlmostDown());
                    state++;
                    break;
                case 1:
                    if (currentTime > time + DEPOSIT1) { state++; }
                    break;
                case 2:
                    time = currentTime;
                    setArmPivot(ARM_PIVOT_POSITION.getUp());
                    setArmHopper(ARM_HOPPER_POSITION.getUp());
                    state++;
                    break;
                case 3:
                    if (currentTime > time + DEPOSIT2) { state++; }
                    break;
                case 4:
                    time = currentTime;
                    setTurret(alliance == RED ? TURRET_SHARED : -TURRET_SHARED);
                    state++;
                    break;
                case 5:
                    if (currentTime > time + DEPOSIT3) { state++; }
                    break;
                case 6:
                    time = currentTime;
                    setSlides(SLIDES_ALLIANCE);
                    setArmPivot(ARM_PIVOT_POSITION.getDeposit());
                    setArmHopper(.99);
                    state++;
                    break;
                case 7:
                    if (currentTime > time + DEPOSIT4) { state++; }
                    break;
                case 8:
                    runningShared = false;
                    justFinishedSharedMacro = true;
                    if (depositQueue) {
                        depositQueue = false;
                        runningDeposit = true;
                    } else {
                        justFinishedAMacro = true;
                    }
                    state = 0;
            }
        }
    }

    public void runningDeposit(double currentTime, Alliance alliance) {
        if (runningDeposit) {
            switch (state) {
                case 0:
                    //"memory" stuff
                    if (justFinishedAllianceMacro) {
                        TURRET_ALLIANCE = getTurret();
                        SLIDES_ALLIANCE = getSlides();
                    } else if (justFinishedSharedMacro) {
                        TURRET_SHARED = getTurret();
                        SLIDES_SHARED = getSlides();
                    }

                    time = currentTime;
                    setArmHopper(ARM_HOPPER_POSITION.getDeposit());
                    state++;
                    break;
                case 1:
                    if (currentTime > time + RETRACT1) { state++; }
                    break;
                case 2:
                    time = currentTime;
                    setSlides(0);
                    setArmPivot(ARM_PIVOT_POSITION.getUp());
                    setArmHopper(ARM_HOPPER_POSITION.getUp());
                    state++;
                    break;
                case 3:
                    if (currentTime > time + RETRACT2) { state++; }
                    break;
                case 4:
                    time = currentTime;
                    setTurret(0);
                    state++;
                    break;
                case 5:
                    if (currentTime > time + RETRACT3) { state++; }
                    break;
                case 6:
                    time = currentTime;
                    setArmPivot(ARM_PIVOT_POSITION.getAlmostDown());
                    setArmHopper(ARM_HOPPER_POSITION.getAlmostDown());
                    state++;
                    break;
                case 7:
                    if (currentTime > time + RETRACT4) { state++; }
                    break;
                case 8:
                    time = currentTime;
                    setArmPivot(ARM_PIVOT_POSITION.getDown());
                    setArmHopper(ARM_HOPPER_POSITION.getDown());
                    state++;
                    break;
                case 9:
                    if (currentTime > time + RETRACT5) { state++; }
                    break;
                case 10:
                    runningDeposit = false;
                    justFinishedAMacro = true;
                    state = 0;
            }
        }
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
