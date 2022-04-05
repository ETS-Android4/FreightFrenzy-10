package org.firstinspires.ftc.teamcode.hardware;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.LEFT;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.MIDDLE;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.RIGHT;
import static org.firstinspires.ftc.teamcode.util.Constants.COLOR;
import static org.firstinspires.ftc.teamcode.util.Constants.HOPPER_SERVO;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_SERVO;
import static org.firstinspires.ftc.teamcode.util.Constants.LEFT_DUCKY;
import static org.firstinspires.ftc.teamcode.util.Constants.ODO_SERVO;
import static org.firstinspires.ftc.teamcode.util.Constants.RIGHT_DUCKY;
import static org.firstinspires.ftc.teamcode.util.Constants.SLIDES;
import static org.firstinspires.ftc.teamcode.util.Constants.PIVOT_SERVO;
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
import org.firstinspires.ftc.teamcode.util.BarcodeLocation;
import org.firstinspires.ftc.teamcode.util.DepositPosition;

import java.util.Locale;

@Config
public class Actuators {
    // misc
    public static double INTAKE_RESET_TIME = 1;
    public static double HOPPER_DISTANCE_CUTOFF = 25;
    public int intakeStartPos = (int) (145.1/8.0);

    // driver variables
    public static int TURRET_SPEED = 5;
    public static int SLIDES_SPEED = 50;
    public static double ARM_HOPPER_SPEED = 0.015;
    public static double ARM_PIVOT_SPEED = 0.005;
    public static double INTAKE_SERVO_SPEED = 0.02;
    public static double DUCKY_SPEED = 1.0;

    public boolean odoRetracted;
    public boolean intakeRetracted;

    // pid variables
    private PIDController turretController;
    private PIDController slidesController;
    private PIDController intakeController;

    public static PIDCoefficients TURRET_COEFFICIENTS = new PIDCoefficients(0.0015, 0, 0);
    public static PIDCoefficients SLIDES_COEFFICIENTS = new PIDCoefficients(0.002, 0, 0);
    public static PIDCoefficients INTAKE_COEFFICIENTS = new PIDCoefficients(0.005, 0, 0.0001);

    public static double TURRET_TOLERANCE = 10;
    public static double SLIDES_TOLERANCE = 15;
    public static double INTAKE_TOLERANCE = 30;

    // ranges
    public static int TURRET_MIN = -1100;
    public static int TURRET_MAX = 1100;
    public static int SLIDES_MIN = 0;
    public static int SLIDES_MAX = (870);
    public static double ARM_HOPPER_MIN = 0.01;
    public static double ARM_HOPPER_MAX = 0.99;
    public static double ARM_PIVOT_MIN = 0.01;
    public static double ARM_PIVOT_MAX = 0.99;
//    public static double INTAKE_SERVO_DOWN = 0.01;
//    public static double INTAKE_SERVO_UP = 0.85;
    public static double INTAKE_SERVO_DOWN = 0.99;
    public static double INTAKE_SERVO_UP = 0.35;
    public static double ODO_SERVO_DOWN = 0.99;
    public static double ODO_SERVO_UP = 0.01;

    // macro timeouts
    public static double DEPOSIT1_ALMOST = 0.4;
    public static double DEPOSIT2_ARM = 0.6;
    public static double DEPOSIT3_EXTEND = 1.5;

    public static double RETRACT1_SCORE = 0.4;
    public static double RETRACT2_RETRACT = 0.7;//.8
    public static double RETRACT3_TURRET = 0.6;
    public static double RETRACT4_ALMOST = 1.2;//1.2
    public static double RETRACT5_DOWN = 0.4;

    // old state machine variables
    public boolean pickingUpFreight;
    public boolean runningAlliance;
    public boolean runningShared;
    public boolean runningArm;
    public boolean runningDeposit;
    public boolean allianceQueue;
    public boolean sharedQueue;
    public boolean depositQueue;
    public boolean justFinishedAllianceMacro;
    public boolean justFinishedSharedMacro;
    public boolean justFinishedAMacro; // this one is still used in the new macros

    // new state machine variables & macro timeouts
    public boolean capPickedUp;
    public boolean runningExtend;
    public boolean runningRetract;
    public boolean retractQueue;
    public boolean justCancledMacro = false;
    public DepositPosition justFinishedPos = HIGH;

    public static double EXTEND_ALMOST = 0.4;
    public static double EXTEND_FULL = 0.7;
    public static double EXTEND_TURRET_SLIDES = 0.7;
    public static double RETRACT_WAIT_FOR_HOPPER = 0.2;
    public static double RETRACT_SLIDES = 0.5;
    public static double RETRACT_TURRET = 0.35;
    public static double RETRACT_ALMOST_GENERAL = 0.9;
    public static double RETRACT_ALMOST_SHARED = 0.9;
    public static double RETRACT_ALMOST_ALLIANCE = 0.9;
    public static double RETRACT_DOWN = 1.2;//0.4

    private int state;
    private double time;

    // hardware
    private DcMotor turret;
    private DcMotor slides;
    private DcMotor intake;
    private Servo hopperServo;
    private Servo pivotServo;
    private CRServo leftDucky;
    private CRServo rightDucky;
    private Servo intakeServo;
    private Servo odoServo;
    private RevColorSensorV3 colorSensor;

    // macro positions
    // IF YOU EDIT THESE MACRO POSITIONS, COPY THEM TO THE CLEARMEMORY FUNCTION LINE 189!!!!
    public static int TURRET_CAP = 0;
    public static int TURRET_SHARED = -1000;
    public static int TURRET_ALLIANCE = 680;

    public static int SLIDES_CAP = 140;
    public static int SLIDES_GENERAL = 150;
    public static int SLIDES_SHARED = 30;
    public static int SLIDES_ALLIANCE_LOW = 691;
    public static int SLIDES_ALLIANCE_MID = 635;
    public static int SLIDES_ALLIANCE_HIGH = 780;


    public static ArmPosition ARM_PIVOT_POSITION = new ArmPosition(0.9, 0.9, 0.9, 0.48, 0.48, 0.09, 0.01, 0.1, 0.24, 0.8, 0.09, 0.01, 0.1, 0.24);
    public static ArmPosition ARM_HOPPER_POSITION = new ArmPosition(0.55, 0.55, 0.55, 0.4, 0.4, 0.2, 0.24, 0.2, 0.25, 0.99, 0.66, 0.48, 0.48, 0.6);

    public void clearMemory() {
        TURRET_CAP = 0;
        TURRET_SHARED = -1000;
        TURRET_ALLIANCE = 680;

        SLIDES_CAP = 140;
        SLIDES_SHARED = 30;
        SLIDES_ALLIANCE_LOW = 691;
        SLIDES_ALLIANCE_MID = 635;
        SLIDES_ALLIANCE_HIGH = 780;

        ARM_PIVOT_POSITION = new ArmPosition(0.9, 0.9, 0.9, 0.48, 0.48, 0.09, 0.01, 0.1, 0.24, 0.8, 0.09, 0.01, 0.1, 0.24);
        ARM_HOPPER_POSITION = new ArmPosition(0.55, 0.55, 0.55, 0.4, 0.55, 0.2, 0.24, 0.2, 0.25, 0.99, 0.66, 0.48, 0.48, 0.6);
    }

    public Actuators(HardwareMap hardwareMap) {
        this.turret = hardwareMap.get(DcMotor.class, TURRET);
        this.slides = hardwareMap.get(DcMotor.class, SLIDES);
        this.intake = hardwareMap.get(DcMotor.class, INTAKE);
        this.hopperServo = hardwareMap.get(Servo.class, HOPPER_SERVO);
        this.pivotServo = hardwareMap.get(Servo.class, PIVOT_SERVO);
        this.leftDucky = hardwareMap.get(CRServo.class, LEFT_DUCKY);
        this.rightDucky = hardwareMap.get(CRServo.class, RIGHT_DUCKY);
        this.intakeServo = hardwareMap.get(Servo.class, INTAKE_SERVO);
        this.odoServo = hardwareMap.get(Servo.class, ODO_SERVO);
        this.colorSensor = hardwareMap.get(RevColorSensorV3.class, COLOR);

        this.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretController = new PIDController(TURRET_COEFFICIENTS.kP, TURRET_COEFFICIENTS.kI, TURRET_COEFFICIENTS.kD);
        slidesController = new PIDController(SLIDES_COEFFICIENTS.kP, SLIDES_COEFFICIENTS.kI, SLIDES_COEFFICIENTS.kD);
        intakeController = new PIDController(INTAKE_COEFFICIENTS.kP, INTAKE_COEFFICIENTS.kI, INTAKE_COEFFICIENTS.kD);
    }

    // turret
    public int getTurret() {
        return turret.getCurrentPosition();
    }

    public void setTurret(int position) {
        //position = clamp(position, TURRET_MIN, TURRET_MAX);
        turretController.setSetPoint(position);
    }

    public void resetTurret(){
        turretController.reset();
    };

    // slides
    public int getSlides() {
        return slides.getCurrentPosition();
    }

    public void setSlides(int position) {
        position = Math.min(Math.max(position, SLIDES_MIN), SLIDES_MAX);
        slidesController.setSetPoint(position);
    }

    // intake
    public int getIntakePosition() {
        return intake.getCurrentPosition();
    }

    public void setIntake(double power){
        setIntakePower(power);
    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void setIntakePosition(int position) {
        intakeController.setSetPoint(position);
        intake.setPower(intakeController.calculate(intake.getCurrentPosition()));
    }

    public boolean intakeIsReset() {
        return intakeController.atSetPoint();
    }

    public void resetIntake() {
        intakeController.setPID(INTAKE_COEFFICIENTS.kP, INTAKE_COEFFICIENTS.kI, INTAKE_COEFFICIENTS.kD);
        intakeController.setTolerance(INTAKE_TOLERANCE);
        intake.setPower(intakeController.calculate(intake.getCurrentPosition()));
    }

    // pid update for motor and slides
    public void update() {
        turretController.setPID(TURRET_COEFFICIENTS.kP, TURRET_COEFFICIENTS.kI, TURRET_COEFFICIENTS.kD);
        slidesController.setPID(SLIDES_COEFFICIENTS.kP, SLIDES_COEFFICIENTS.kI, SLIDES_COEFFICIENTS.kD);
        intakeController.setPID(INTAKE_COEFFICIENTS.kP, INTAKE_COEFFICIENTS.kI, INTAKE_COEFFICIENTS.kD);

        turretController.setTolerance(TURRET_TOLERANCE);
        slidesController.setTolerance(SLIDES_TOLERANCE);
        intakeController.setTolerance(INTAKE_TOLERANCE);

        turret.setPower(turretController.calculate(turret.getCurrentPosition()));
        slides.setPower(slidesController.calculate(slides.getCurrentPosition()));
//        this.intake.setPower(intakeController.calculate(intake.getCurrentPosition()));
    }

    // arm hopper
    public double getArmHopper() {
        return hopperServo.getPosition();
    }

    public void setArmHopper(double position) {
        hopperServo.setPosition(position);
    }

    // arm pivot
    public double getArmPivot() {
        return pivotServo.getPosition();
    }

    public void setArmPivot(double position) {
        pivotServo.setPosition(position);
    }

    // duckies
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

    // color sensor
    public double getHopperDistance() {
        return colorSensor.getDistance(DistanceUnit.MM);
    }

    public boolean hopperIsFull() {
        return getHopperDistance() < HOPPER_DISTANCE_CUTOFF;
    }

    // macros
    public void setState(int newState) {
        state = newState;
    }

    public int getState() {
        return state;
    }

    public void runningAlliance(double currentTime, Alliance alliance, BarcodeLocation barcodeLocation) {
        //runningAlliance_OLD(currentTime, alliance, barcodeLocation);
        runningExtend(currentTime, alliance, barcodeLocation == LEFT ? LOW : (barcodeLocation == MIDDLE ? MID : HIGH));
    }

    public void runningShared(double currentTime, Alliance alliance, BarcodeLocation barcodeLocation) {
//        runningShared_OLD(currentTime, alliance, barcodeLocation);
        runningExtend(currentTime, alliance, SHARED);
    }

    public void runningDeposit(double currentTime, Alliance alliance, BarcodeLocation barcodeLocation) {
//        runningDeposit_OLD(currentTime, alliance, barcodeLocation);
        runningRetract(currentTime, alliance, barcodeLocation == LEFT ? LOW : (barcodeLocation == MIDDLE ? MID : HIGH));
    }

    // NEW MACROS
    public void runningExtend(double currentTime, Alliance alliance, DepositPosition depoPos) {
        if (runningExtend) {
            // steps
            switch(state) {
                // reset intake
//                case 0:
//                    setIntakePosition((int) (intakeStartPos + (getIntakePosition() - (getIntakePosition() % 145.1))));
//                    time = currentTime;
//                    state++;
//                    break;
//                case 1:
//                    resetIntake();
//                    if (intakeController.atSetPoint()) {
//                        intake.setPower(0);
//                        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//allow hopper to push intake out of the way
//                        time = currentTime;
//                        state++;
//                    }
//                    break;
                // arm full
                case 0:
                    //if(!hopperIsFull()){state=5;} //if we dont actually have a block, don't bother to extend

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
                    time = currentTime;
                    state++;
                    break;
                // hopper full
                case 1:
                    if (currentTime > time + EXTEND_FULL/2.0) {
                        time = currentTime;
                        state++;
                    }
                    break;
                case 2:
                    // optional retract intake for shared hub
                    if (depoPos == SHARED && currentTime > time + 0.1) {
                        intakeRetracted = true;
                        setIntakeServo(INTAKE_SERVO_UP);
                    }
                    if (depoPos == GENERAL) {
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostGeneral());
                    } else if (depoPos == SHARED) {
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostShared());
                    } else if (depoPos == LOW) {
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostLow());
                    } else if (depoPos == MID) {
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostMid());
                    } else if (depoPos == HIGH) {
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostHigh());
                    }
                    if (currentTime > time + EXTEND_FULL/2.0) {
                        time = currentTime;
                        state++;
                    }
                    break;
                // turret and slides
                case 3:
                    if (depoPos == GENERAL && alliance == RED) {
                        setTurret(TURRET_CAP);
                    } else if (depoPos == GENERAL && alliance == BLUE) {
                        setTurret(-TURRET_CAP);
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
                        setSlides(SLIDES_CAP);
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
                case 4:
                    if (currentTime > time + EXTEND_TURRET_SLIDES || (turretController.atSetPoint() && slidesController.atSetPoint())) {
                        time = currentTime;
                        state++;
                    }
                    break;
                // finish
                case 5:
                    runningExtend = false;
                    runningAlliance = false;
                    runningShared = false;
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
        }
    }

    public void runningRetract(double currentTime, Alliance alliance, DepositPosition depoPos) {
        if (runningRetract) {
            switch (state) {
                // deposit
                case 0:
                    if(!justCancledMacro) {
                        // "memory" stuff
//                        if (justFinishedPos == GENERAL) {
//                            TURRET_CAP = alliance == RED ? (int) turretController.getSetPoint() : -(int) turretController.getSetPoint();
//                            SLIDES_CAP = (int) slidesController.getSetPoint();
//                            ARM_PIVOT_POSITION.setAlmostGeneral(pivotServo.getPosition());
//                            ARM_HOPPER_POSITION.setAlmostGeneral(hopperServo.getPosition());
                        if (justFinishedPos == SHARED) {
                            TURRET_SHARED = alliance == RED ? (int) turretController.getSetPoint() : -(int) turretController.getSetPoint();
                            SLIDES_SHARED = (int) slidesController.getSetPoint();
                            ARM_PIVOT_POSITION.setAlmostShared(pivotServo.getPosition());
                            ARM_HOPPER_POSITION.setAlmostShared(hopperServo.getPosition());
                        } else if (justFinishedPos == LOW) {
                            TURRET_ALLIANCE = alliance == RED ? (int) turretController.getSetPoint() : -(int) turretController.getSetPoint();
                            SLIDES_ALLIANCE_LOW = (int) slidesController.getSetPoint();
                            ARM_PIVOT_POSITION.setAlmostLow(pivotServo.getPosition());
                            ARM_HOPPER_POSITION.setAlmostLow(hopperServo.getPosition());
                        } else if (justFinishedPos == MID) {
                            TURRET_ALLIANCE = alliance == RED ? (int) turretController.getSetPoint() : -(int) turretController.getSetPoint();
                            SLIDES_ALLIANCE_MID = (int) slidesController.getSetPoint();
                            ARM_PIVOT_POSITION.setAlmostMid(pivotServo.getPosition());
                            ARM_HOPPER_POSITION.setAlmostMid(hopperServo.getPosition());
                        } else if (justFinishedPos == HIGH) {
                            TURRET_ALLIANCE = alliance == RED ? (int) turretController.getSetPoint() : -(int) turretController.getSetPoint();
                            SLIDES_ALLIANCE_HIGH = (int) slidesController.getSetPoint();
                            ARM_PIVOT_POSITION.setAlmostHigh(pivotServo.getPosition());
                            ARM_HOPPER_POSITION.setAlmostHigh(hopperServo.getPosition());
                        }
                    }

                    if (depoPos == GENERAL) { // this is meant to be used for capping, so it doesn't need to drop off
                        state = 4;
                        break;
                    }

                    if (depoPos == SHARED) {
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

                    justCancledMacro = false;

                    time = currentTime;
                    state++;
                    break;
                // wait for freight to fall out
                case 1:
                    if (!hopperIsFull()) {
                        intakeRetracted = false;
                        setIntakeServo(INTAKE_SERVO_DOWN);
                        state++;
                    }
                    break;
                // move hopper out of danger grabbing zone
                case 2:
                    if (depoPos == GENERAL) {
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostGeneral());
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostGeneral());
                    } else if (depoPos == SHARED) {
                        setArmHopper(ARM_HOPPER_POSITION.getUp());
                        setArmPivot(ARM_PIVOT_POSITION.getUp());
                    } else if (depoPos == LOW) {
//                        setArmHopper(ARM_HOPPER_POSITION.getAlmostLow());
//                        setArmPivot(ARM_PIVOT_POSITION.getAlmostLow());
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostHigh());//the low and mid return to high to clear the chassis
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostHigh());
                    } else if (depoPos == MID) {
//                        setArmHopper(ARM_HOPPER_POSITION.getAlmostMid());
//                        setArmPivot(ARM_PIVOT_POSITION.getAlmostMid());
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostHigh());
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostHigh());
                    } else if (depoPos == HIGH) {
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostHigh());
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostHigh());
                    }
                    time = currentTime;
                    state++;
                    break;
                case 3:
                    if (currentTime > time + RETRACT_WAIT_FOR_HOPPER) {
                        state++;
                    }
                    break;
                // return slides and turret
                case 4:
                    setSlides(30);
                    time = currentTime;
                    state++;
                    break;
                case 5:
                    if (currentTime > time + RETRACT_SLIDES || slidesController.atSetPoint()) {
                        state++;
                    }
                    break;
                case 6:
                    setTurret(0);
                    time = currentTime;
                    state++;
                    break;
                case 7:
                    if (currentTime > time + RETRACT_TURRET || turretController.atSetPoint()) {
                        state++;
                    }
                    break;
                // return arm down
                case 8:
                    setSlides(0);
                    setArmPivot(ARM_PIVOT_POSITION.getDown());
                    setArmHopper(ARM_HOPPER_POSITION.getDown());
                    time = currentTime;
                    state++;
                    break;
                case 9:
                    if (currentTime > time + RETRACT_DOWN) {
//                        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//hopper cannot be pushed anymore
                        state++;
                    }
                    break;
                case 10:
                    runningRetract = false;
                    runningDeposit = false;
                    justFinishedAMacro = true;
                    state = 0;
            }
        }
    }

    // OLD MACROS
    public void runningAlliance_OLD(double currentTime, Alliance alliance, BarcodeLocation barcodeLocation) {
        if (runningAlliance) {
            resetIntake(); // update intake PID
            switch (state) {
                case 0:
                    time = currentTime;
                    setIntakePosition((int) (intakeStartPos + (getIntakePosition() - (getIntakePosition() % 145.1))));
                    setArmPivot(ARM_PIVOT_POSITION.getAlmostDown());
                    state++;
                    break;
                case 1:
                    if (currentTime > time + DEPOSIT1_ALMOST) {
                        state++;
                    }
                    if (currentTime > time + DEPOSIT1_ALMOST*0.25) {
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostDown());
                    }
                    break;
                case 2:
                    time = currentTime;
                    if (barcodeLocation == LEFT) {
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostLow());
                    } else if (barcodeLocation == MIDDLE) {
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostMid());
                    } else {
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostHigh());
                    }
                    state++;
                    break;
                case 3:
                    if (currentTime > time + DEPOSIT2_ARM) {
                        state++;
                    }
                    if (currentTime > time + DEPOSIT2_ARM / 2.0) {
                        if (barcodeLocation == LEFT) {
                            setArmHopper(ARM_HOPPER_POSITION.getAlmostLow());
                        } else if (barcodeLocation == MIDDLE) {
                            setArmHopper(ARM_HOPPER_POSITION.getAlmostMid());
                        } else {
                            setArmHopper(ARM_HOPPER_POSITION.getAlmostHigh());
                        }
                    }
                    break;
                case 4:
                    time = currentTime;
                    setTurret(alliance == RED ? TURRET_ALLIANCE : -TURRET_ALLIANCE);
                    setSlides(SLIDES_ALLIANCE_HIGH);
                    state++;
                    break;
                case 5:
                    if (currentTime > time + DEPOSIT3_EXTEND || (turretController.atSetPoint() && slidesController.atSetPoint())) {
                        state++;
                    }
                    break;
                case 6:
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

    public void runningShared_OLD(double currentTime, Alliance alliance, BarcodeLocation barcodeLocation) {
        if (runningShared) {
            resetIntake();
            switch (state) {
                case 0:
                    time = currentTime;
                    setIntakePosition((int) (intakeStartPos + (getIntakePosition() - (getIntakePosition() % 145.1))));
                    setArmPivot(ARM_PIVOT_POSITION.getAlmostDown());
                    state++;
                    break;
                case 1:
                    if (currentTime > time + DEPOSIT1_ALMOST) {
                        state++;
                    }
                    if (currentTime > time + DEPOSIT1_ALMOST / 2.0) {
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostDown());
                    }
                    break;
                case 2:
                    time = currentTime;
                    if (barcodeLocation == LEFT) {
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostLow());
                    } else if (barcodeLocation == MIDDLE) {
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostMid());
                    } else {
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostHigh());
                    }
                    state++;
                    break;
                case 3:
                    if (currentTime > time + DEPOSIT2_ARM) {
                        state++;
                    }
                    if (currentTime > time + DEPOSIT2_ARM / 2.0) {
                        if (barcodeLocation == LEFT) {
                            setArmHopper(ARM_HOPPER_POSITION.getAlmostLow());
                        } else if (barcodeLocation == MIDDLE) {
                            setArmHopper(ARM_HOPPER_POSITION.getAlmostMid());
                        } else {
                            setArmHopper(ARM_HOPPER_POSITION.getAlmostHigh());
                        }
                    }
                    break;
                case 4:
                    time = currentTime;
                    setTurret(alliance == RED ? TURRET_SHARED : -TURRET_SHARED);
                    setSlides(SLIDES_SHARED);
                    state++;
                    break;
                case 5:
                    if (currentTime > time + DEPOSIT3_EXTEND || (turretController.atSetPoint() && slidesController.atSetPoint())) {
                        state++;
                    }
                    break;
                case 6:
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

    public void runningDeposit_OLD(double currentTime, Alliance alliance, BarcodeLocation barcodeLocation) {
        if (runningDeposit) {
            resetIntake();
            switch (state) {
                case 0:
                    //"memory" stuff
                    if (justFinishedAllianceMacro) {
                        TURRET_ALLIANCE = alliance == RED ? (int) turretController.getSetPoint() : -(int) turretController.getSetPoint();
                        SLIDES_ALLIANCE_HIGH = (int) slidesController.getSetPoint();
                    } else if (justFinishedSharedMacro) {
                        TURRET_SHARED = alliance == RED ? (int) turretController.getSetPoint() : -(int) turretController.getSetPoint();
                        SLIDES_SHARED = (int) slidesController.getSetPoint();
                    }

                    time = currentTime;
                    setIntakePosition((int) (intakeStartPos + (getIntakePosition() - (getIntakePosition() % 145.1))));
                    if (barcodeLocation == LEFT) {
                        setArmHopper(ARM_HOPPER_POSITION.getLow());
                        setArmPivot(ARM_PIVOT_POSITION.getLow());
                    } else if (barcodeLocation == MIDDLE) {
                        setArmHopper(ARM_HOPPER_POSITION.getMid());
                        setArmPivot(ARM_PIVOT_POSITION.getMid());
                    } else if (barcodeLocation == RIGHT) {
                        setArmHopper(ARM_HOPPER_POSITION.getHigh());
                        setArmPivot(ARM_PIVOT_POSITION.getHigh());
                    }

                    state++;
                    break;
                case 1:
                    if (currentTime > time + RETRACT1_SCORE) {
                        state++;
                    }
//                    if (justFinishedSharedMacro && currentTime > time + SLOW_DEPOSIT_TIME) {
//                        if(getArmHopper()>ARM_HOPPER_POSITION.getLow())
//                        {
//                            setArmHopper(getArmHopper()+SLOW_DEPOSIT_INCREMENT);
//                        }
//                    }
                    break;
                case 2:
                    time = currentTime;
                    setArmHopper(ARM_HOPPER_POSITION.getAlmostDown());
                    state++;
                    break;
                case 3:
                    if (currentTime > time + 0.25) {
                        state++;
                    }
                    break;
                case 4:
                    time = currentTime;
                    setSlides(0);
                    state++;
                    break;
                case 5:
                    if (currentTime > time + RETRACT2_RETRACT || slidesController.atSetPoint()) {
                        state++;
                    }
                    break;
                case 6:
                    time = currentTime;
                    setTurret(0);
                    state++;
                    break;
                case 7:
                    if (currentTime > time + RETRACT3_TURRET || turretController.atSetPoint()) {
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
                    if (currentTime > time + RETRACT4_ALMOST) {
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
                    if (currentTime > time + RETRACT5_DOWN) {
                        state++;
                    }
                    break;
                case 12:
                    runningDeposit = false;
                    justFinishedAMacro = true;
                    justFinishedAllianceMacro = false;
                    justFinishedSharedMacro = false;
                    state = 0;
            }
        }
    }

    public void runningArm(double currentTime) {
        if (runningArm) {
            resetIntake();
            switch (state) {
                case 0:
                    time = currentTime;
                    setIntakePosition((int) (intakeStartPos + (getIntakePosition() - (getIntakePosition() % 145.1))));
                    setArmPivot(ARM_PIVOT_POSITION.getAlmostDown());
                    setArmHopper(ARM_HOPPER_POSITION.getAlmostDown());
                    state++;
                    break;
                case 1:
                    if (currentTime > time + 1) {
                        state++;
                    }
                    break;
                case 2:
                    time = currentTime;
                    setArmPivot(ARM_PIVOT_POSITION.getAlmostHigh());
                    state++;
                    break;
                case 3:
                    if (currentTime > time + 1.1) {
                        state++;
                    }
                    if (currentTime > time + 0.4) {
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostHigh());
                    }
                    break;
                case 4:
                    runningArm = false;
                    justFinishedAMacro = true;
                    state = 0;
            }
        }
    }

    // telemetry
    public String getTelemetry() {
        return String.format(Locale.US, "" +
            "Turret:      pos %s pow %.2f err %.2f\n" +
            "Slides:      pos %s pow %.2f err %.2f\n" +
            "Intake:      pos %s pow %.2f err %.2f\n" +
            "HopperServo: pos %.2f\n" +
            "PivotServo:  pos %.2f\n" +
            "Duckies:     left %.2f right %.2f\n" +
            "IntakeServo: pos %.2f\n" +
            "OdoServo:    pos %.2f\n" +
            "Hopper:      dist %.2f",
            turret.getCurrentPosition(), turret.getPower(), turretController.getPositionError(),
            slides.getCurrentPosition(), slides.getPower(), slidesController.getPositionError(),
            intake.getCurrentPosition(), intake.getPower(), intakeController.getPositionError(),
            hopperServo.getPosition(),
            pivotServo.getPosition(),
            leftDucky.getPower(), rightDucky.getPower(),
            intakeServo.getPosition(),
            odoServo.getPosition(),
            colorSensor.getDistance(DistanceUnit.CM)
        );
    }
}
