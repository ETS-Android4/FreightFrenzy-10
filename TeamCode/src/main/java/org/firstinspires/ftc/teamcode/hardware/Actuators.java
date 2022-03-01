package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.Constants.COLOR;
import static org.firstinspires.ftc.teamcode.util.Constants.HOPPER_SERVO;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_SERVO;
import static org.firstinspires.ftc.teamcode.util.Constants.LEFT_DUCKY;
import static org.firstinspires.ftc.teamcode.util.Constants.ODO_SERVO;
import static org.firstinspires.ftc.teamcode.util.Constants.RIGHT_DUCKY;
import static org.firstinspires.ftc.teamcode.util.Constants.SLIDES;
import static org.firstinspires.ftc.teamcode.util.Constants.SLIDES_SERVO;
import static org.firstinspires.ftc.teamcode.util.Constants.TURRET;
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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.ArmPosition;
import org.firstinspires.ftc.teamcode.util.DepositPosition;
import org.firstinspires.ftc.teamcode.util.Range;

import java.util.Locale;

@Config
public class Actuators {
    // range & speed
    public static int TURRET_SPEED = 15;
    public static int SLIDES_SPEED = 50;
    public static double ARM_HOPPER_SPEED = 0.015;
    public static double ARM_PIVOT_SPEED = 0.01;
    public static double DUCKY_SPEED = 1.0;

    public static Range TURRET_RANGE = new Range(-1000, 1000);
    public static Range SLIDES_RANGE = new Range(0, 2300*0.377373212);
    public static Range ARM_HOPPER_RANGE = new Range(0.01, 0.99);
    public static Range ARM_PIVOT_RANGE = new Range(0.01, 0.99);

    // pid variables
    public static double TURRET_TOLERANCE = 0;
    public static double SLIDES_TOLERANCE = 50;
    public static double INTAKE_TOLERANCE = 50;

    public static PIDCoefficients TURRET_COEFFICIENTS = new PIDCoefficients(0.003, 0, 0);
    public static PIDCoefficients SLIDES_COEFFICIENTS = new PIDCoefficients(0.002, 0, 0);
    public static PIDCoefficients INTAKE_COEFFICIENTS = new PIDCoefficients(0.005, 0, 0.0001);

    // actuator positions
    public static ArmPosition ARM_PIVOT_POSITION = new ArmPosition(0.06, 0.03, 0.12, 0.51, 0.95, 0.95, 0.95, 0.85, 0.8, 0.95, 0.95, 0.95, 0.85, 0.8);
    public static ArmPosition ARM_HOPPER_POSITION = new ArmPosition(0.62, 0.62, 0.68, 0.74, 0.95, 0.95, 0.92, 0.92, 0.92, 0.68, 0.68, 0.68, 0.64, 0.49);

    public int INTAKE_SERVO_START = (int) (145.1/8.0);
    public static double INTAKE_SERVO_DOWN = 0.01;
    public static double INTAKE_SERVO_UP = 0.99;
    public static double ODO_SERVO_DOWN = 0.01;
    public static double ODO_SERVO_UP = 0.99;

    public static int TURRET_GENERAL = 0;
    public static int TURRET_SHARED = -800;
    public static int TURRET_ALLIANCE = 650;

    public static int SLIDES_GENERAL = 0;
    public static int SLIDES_SHARED = 0;
    public static int SLIDES_ALLIANCE_LOW = (int)(1422*0.377373212);
    public static int SLIDES_ALLIANCE_MID = (int)(1649*0.377373212);
    public static int SLIDES_ALLIANCE_HIGH = (int)(2200*0.377373212);

    // timing variables
    public static double INTAKE_RESET_TIME = 1;

    public static double EXTEND_ARM_ALMOST_DOWN = 0.4;
    public static double EXTEND_ARM_ALMOST_SCORE = 1;

    public static double RETRACT_ARM_SCORE = 0.4;
    public static double RETRACT_ARM_DEPOSIT = 0.25;
    public static double RETRACT_ARM_ALMOST_DOWN = 1;
    public static double RETRACT_ARM_DOWN = 0.4;

    private PIDController turretController;
    private PIDController slidesController;
    private PIDController intakeController;

    // state machine variables
    public boolean runningExtend;
    public boolean runningRetract;
    public boolean retractQueue;
    public DepositPosition justFinishedPos = HIGH;
    public boolean justFinishedAMacro;

    private int state;
    private double time;

    public static double HOPPER_DISTANCE_CUTOFF = 30;

    // hardware
    private DcMotor intake;
    private DcMotor turret;
    private DcMotor slides;
    private Servo hopperServo;
    private Servo pivotServo;
    private CRServo leftDucky;
    private CRServo rightDucky;
    private Servo intakeServo;
    private Servo odoServo;
    private RevColorSensorV3 colorSensor;

    public Actuators(HardwareMap hardwareMap) {
        this.intake = hardwareMap.get(DcMotor.class, INTAKE);
        this.turret = hardwareMap.get(DcMotor.class, TURRET);
        this.slides = hardwareMap.get(DcMotor.class, SLIDES);
        this.hopperServo = hardwareMap.get(Servo.class, HOPPER_SERVO);
        this.pivotServo = hardwareMap.get(Servo.class, SLIDES_SERVO);
        this.leftDucky = hardwareMap.get(CRServo.class, LEFT_DUCKY);
        this.rightDucky = hardwareMap.get(CRServo.class, RIGHT_DUCKY);
        this.intakeServo = hardwareMap.get(Servo.class, INTAKE_SERVO);
        this.odoServo = hardwareMap.get(Servo.class, ODO_SERVO);
        this.colorSensor = hardwareMap.get(RevColorSensorV3.class, COLOR);

        this.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void setState(int newState){
        state = newState;
    }

    public double getHopperDistance() {
        return colorSensor.getDistance(DistanceUnit.MM);
    }

    public boolean hopperIsFull() {
        return getHopperDistance() < HOPPER_DISTANCE_CUTOFF;
    }

    public void setOdoServo(double position) {
        odoServo.setPosition(position);
    }

    public double getOdoServo() {
        return odoServo.getPosition();
    }

    public void setIntakeServo(double position) {
        intakeServo.setPosition(position);
    }

    public double getIntakeServo() {
        return intakeServo.getPosition();
    }

    public void setIntake(double power) {
        this.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intake.setPower(power);
    }

    public void setIntakePosition(int position) {
        intakeController.setSetPoint(position);
        intake.setPower(intakeController.calculate(intake.getCurrentPosition()));
    }

    public int getIntakePosition() {
        return this.intake.getCurrentPosition();
    }

    public boolean intakeIsReset() {
        return intakeController.atSetPoint();
    }

    public void resetIntake() {
        intakeController.setPID(INTAKE_COEFFICIENTS.kP, INTAKE_COEFFICIENTS.kI, INTAKE_COEFFICIENTS.kD);
        intakeController.setTolerance(INTAKE_TOLERANCE);
        intake.setPower(intakeController.calculate(intake.getCurrentPosition()));
    }

    public void setTurret(int position) {
        turretController.setSetPoint(position);
    }

    public int getTurret() {
        return this.turret.getCurrentPosition();
    }

    public void setSlides(int position) {
        slidesController.setSetPoint(position);
    }

    public int getSlides() {
        return this.slides.getCurrentPosition();
    }

    public void setArmHopper(double position) {
        this.hopperServo.setPosition(position);
    }

    public double getArmHopper() {
        return this.hopperServo.getPosition();
    }

    public void setArmPivot(double position) {
        this.pivotServo.setPosition(position);
    }

    public double getArmPivot() {
        return this.pivotServo.getPosition();
    }

    // ducks
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

    // macros
    public void runningExtend(double currentTime, Alliance alliance, DepositPosition depoPos) {
        if (runningExtend) {
            // steps
            switch(state) {
                // reset intake
                case 0:
                    setIntakePosition((int) (INTAKE_SERVO_START + (getIntakePosition() - (getIntakePosition() % 145.1))));
                    state++;
                    break;
                case 1:
                    if (intakeController.atSetPoint()) {
                        state++;
                    }
                    break;
                // arm almost
                case 2:
                    setArmPivot(ARM_PIVOT_POSITION.getAlmostDown());
                    setArmHopper(ARM_HOPPER_POSITION.getAlmostDown());
                    time = currentTime;
                    state++;
                    break;
                case 3:
                    if (currentTime > time + EXTEND_ARM_ALMOST_DOWN) {
                        state++;
                    }
                    break;
                // arm full
                case 4:
                    if (depoPos == GENERAL) {
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostGeneral());
                    } else if (depoPos == SHARED) {
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostShared());
                    } else if (depoPos == LOW) {
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostLow());
                    } else if (depoPos == MID) {
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostMid());
                    } else if (depoPos == HIGH) {
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostHigh());
                    }
//                    setArmHopper(0.3); // make configurable?
                    time = currentTime;
                    state++;
                    break;
                case 5:
                    if (currentTime > time + EXTEND_ARM_ALMOST_SCORE) {
                        state++;
                    }
                    break;
                // turret and slides
                case 6:
                    if (depoPos == GENERAL && alliance == RED) {
                        setTurret(TURRET_GENERAL);
                    } else if (depoPos == GENERAL && alliance == BLUE) {
                        setTurret(-TURRET_GENERAL);
                    } else if (depoPos == SHARED && alliance == RED) {
                        setTurret(TURRET_SHARED);
                    } else if (depoPos == SHARED && alliance == BLUE) {
                        setTurret(-TURRET_SHARED);
                    } else if (alliance == RED) {
                        setTurret(TURRET_ALLIANCE);
                    } else if (alliance == BLUE) {
                        setTurret(-TURRET_ALLIANCE);
                    }
                    if (depoPos == GENERAL) {
                        setSlides(SLIDES_GENERAL);
                    } else if (depoPos == SHARED) {
                        setSlides(SLIDES_SHARED);
                    } else if (depoPos == LOW) {
                        setSlides(SLIDES_ALLIANCE_LOW);
                    } else if (depoPos == MID) {
                        setSlides(SLIDES_ALLIANCE_MID);
                    } else if (depoPos == HIGH) {
                        setSlides(SLIDES_ALLIANCE_HIGH);
                    }
                    time = currentTime;
                    state++;
                    break;
                case 7:
                    if ((((getTurret()>turretController.getSetPoint()-10 && getTurret()<turretController.getSetPoint()+10)) && slidesController.atSetPoint()) || currentTime>time+2) {
                        state++;
                    }
                    break;
                // finish
                case 8:
                    runningExtend = false;
                    justFinishedPos = depoPos;
                    if (retractQueue) {
                        retractQueue = false;
                        runningRetract = true;
                    } else {
                        justFinishedAMacro = true;
                    }
                    state = 0;
                    break;
            }
            resetIntake();
        }
    }


    public void runningRetract(double currentTime, Alliance alliance, DepositPosition depoPos) {
        if (runningRetract) {
            switch (state) {
                // deposit
                case 0:
                    // "memory" stuff
                    if (justFinishedPos == GENERAL) {
                        TURRET_GENERAL = alliance == RED ? (int) turretController.getSetPoint() : -(int) turretController.getSetPoint();
                        SLIDES_GENERAL = (int) slidesController.getSetPoint();
                    } else if (justFinishedPos == SHARED) {
                        TURRET_SHARED = alliance == RED ? (int) turretController.getSetPoint() : -(int) turretController.getSetPoint();
                        SLIDES_SHARED = (int) slidesController.getSetPoint();
                    } else if (justFinishedPos == LOW) {
                        TURRET_ALLIANCE = alliance == RED ? (int) turretController.getSetPoint() : -(int) turretController.getSetPoint();
                        SLIDES_ALLIANCE_LOW = (int) slidesController.getSetPoint();
                    } else if (justFinishedPos == MID) {
                        TURRET_ALLIANCE = alliance == RED ? (int) turretController.getSetPoint() : -(int) turretController.getSetPoint();
                        SLIDES_ALLIANCE_MID = (int) slidesController.getSetPoint();
                    } else if (justFinishedPos == HIGH) {
                        TURRET_ALLIANCE = alliance == RED ? (int) turretController.getSetPoint() : -(int) turretController.getSetPoint();
                        SLIDES_ALLIANCE_HIGH = (int) slidesController.getSetPoint();
                    }

                    // score
                    if (depoPos == GENERAL) {
                        setArmHopper(ARM_HOPPER_POSITION.getGeneral());
                        setArmPivot(ARM_PIVOT_POSITION.getGeneral());
                    } else if (depoPos == SHARED) {
                        setArmHopper(ARM_HOPPER_POSITION.getShared());
                        setArmPivot(ARM_PIVOT_POSITION.getShared());
                    } else if (depoPos == LOW) {
                        setArmHopper(ARM_HOPPER_POSITION.getLow());
                        setArmPivot(ARM_PIVOT_POSITION.getLow());
                    } else if (depoPos == MID) {
                        setArmHopper(ARM_HOPPER_POSITION.getMid());
                        setArmPivot(ARM_PIVOT_POSITION.getMid());
                    } else if (depoPos == HIGH) {
                        setArmHopper(ARM_HOPPER_POSITION.getHigh());
                        setArmPivot(ARM_PIVOT_POSITION.getHigh());
                    }

                    time = currentTime;
                    state++;
                    break;
                case 1:
                    if (currentTime > time + RETRACT_ARM_SCORE) { //use hopper?
                        state++;
                    }
                    break;
                // return arm
                case 2:
                    time = currentTime;
                    setArmHopper(ARM_HOPPER_POSITION.getAlmostDown());
                    state++;
                    break;
                case 3:
                    if (currentTime > time + RETRACT_ARM_DEPOSIT) {
                        state++;
                    }
                    break;
                case 4:
                    setSlides(0);
                    time = currentTime;
                    state++;
                    break;
                case 5:
                    if (slidesController.atSetPoint() ) {
                        state++;
                    }
                    break;
                case 6:
                    setTurret(0);
                    time = currentTime;
                    state++;
                    break;
                case 7:
                    if ((getTurret()>turretController.getSetPoint()-10 && getTurret()<turretController.getSetPoint()+10) || currentTime>time+2) {
                        state++;
                    }
                    break;
                case 8:
                    time = currentTime;
                    setArmPivot(ARM_PIVOT_POSITION.getAlmostDown());
                    setArmHopper(ARM_HOPPER_POSITION.getAlmostDown());
                    state++;
                    break;
                case 9:
                    if (currentTime > time + RETRACT_ARM_ALMOST_DOWN) {
                        state++;
                    }
                    break;
                case 10:
                    time = currentTime;
                    setArmPivot(ARM_PIVOT_POSITION.getDown());
                    setArmHopper(ARM_HOPPER_POSITION.getDown());
                    state++;
                    break;
                case 11:
                    if (currentTime > time + RETRACT_ARM_DOWN) {
                        state++;
                    }
                    break;
                case 12:
                    runningRetract = false;
                    justFinishedAMacro = true;
                    state = 0;
            }
            resetIntake();
        }
    }

    public void update() {
        turretController.setPID(TURRET_COEFFICIENTS.kP, TURRET_COEFFICIENTS.kI, TURRET_COEFFICIENTS.kD);
        slidesController.setPID(SLIDES_COEFFICIENTS.kP, SLIDES_COEFFICIENTS.kI, SLIDES_COEFFICIENTS.kD);
        turretController.setTolerance(TURRET_TOLERANCE);
        slidesController.setTolerance(SLIDES_TOLERANCE);
        this.turret.setPower(turretController.calculate(turret.getCurrentPosition()));
        this.slides.setPower(slidesController.calculate(slides.getCurrentPosition()));

        intakeController.setPID(INTAKE_COEFFICIENTS.kP, INTAKE_COEFFICIENTS.kI, INTAKE_COEFFICIENTS.kD);
        intakeController.setTolerance(INTAKE_TOLERANCE);
    }

    public String getTelemetry() {
        return String.format(Locale.US, "" +
                        "Intake:      pos %s pow %.2f\n" +
                        "Turret:      pos %s pow %.2f err %.2f\n" +
                        "Slides:      pos %s pow %.2f\n" +
                        "HopperServo: pos %.2f\n" +
                        "SlidesServo: pos %.2f\n" +
                        "Duckies: left %.2f right %.2f\n" +
                        "IntakeServo: pos %.2f\n" +
                        "OdoServo: pos %.2f\n" +
                        "Hopper: dist %.2f",
                intake.getCurrentPosition(), intake.getPower(), turret.getCurrentPosition(), turret.getPower(), turretController.getPositionError(),
                slides.getCurrentPosition(), slides.getPower(), hopperServo.getPosition(), pivotServo.getPosition(),
                leftDucky.getPower(), rightDucky.getPower(), intakeServo.getPosition(), odoServo.getPosition(), getHopperDistance());
    }
}
