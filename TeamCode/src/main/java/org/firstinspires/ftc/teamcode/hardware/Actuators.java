package org.firstinspires.ftc.teamcode.hardware;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.LEFT;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.MIDDLE;
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
    public boolean hasBlock = false;

    // driver variables
    public static int TURRET_SPEED = 5;
    public static int SLIDES_SPEED = 50;
    public static double ARM_HOPPER_SPEED = 0.015;
    public static double ARM_PIVOT_SPEED = 0.005;
    public static double INTAKE_SERVO_SPEED = 0.02;
    public static double DUCKY_SPEED = 1.0;

    public static double INTAKE_SPEED = 0.75;
    public static double INTAKE_SLOW_SPEED = 0.75;


    public boolean odoRetracted = false;
    public boolean intakeRetracted;

    // pid variables
    private PIDController turretController;
    private PIDController slidesController;
    private PIDController intakeController;

    public static PIDCoefficients TURRET_COEFFICIENTS = new PIDCoefficients(0.0022, 0, 0);
    public static PIDCoefficients SLIDES_COEFFICIENTS = new PIDCoefficients(0.0022, 0, 0);
    public static PIDCoefficients INTAKE_COEFFICIENTS = new PIDCoefficients(0.005, 0, 0.0001);

    public static double TURRET_TOLERANCE = 30;
    public static double SLIDES_TOLERANCE = 20;
    public static double INTAKE_TOLERANCE = 30;

    public static boolean INTAKE_PID_MODE = false;

    // ranges
    public static int TURRET_MIN = -1100;
    public static int TURRET_MAX = 1100;
    public static int SLIDES_MIN = 0;
    public static int SLIDES_MAX = (870);
    public static double ARM_HOPPER_MIN = 0.01;
    public static double ARM_HOPPER_MAX = 0.99;
    public static double ARM_PIVOT_MIN = 0.01;
    public static double ARM_PIVOT_MAX = 0.99;
    public static double INTAKE_SERVO_DOWN = 0.99;
    public static double INTAKE_SERVO_UP = 0.35;
    public static double ODO_SERVO_UP = 0.99;
    public static double ODO_SERVO_DOWN = 0.01;

    // old state machine variables
    public boolean runningAlliance;
    public boolean runningShared;
    public boolean runningDeposit;
    public boolean justFinishedAMacro;

    // new state machine variables & macro timeouts
    public boolean capPickedUp;
    public boolean runningExtend;
    public boolean runningRetract;
    public boolean retractQueue;
    public boolean justCancledMacro = false;
    public DepositPosition justFinishedPos = HIGH;

    public static double EXTEND_ARM = 1;
    public static double EXTEND_TURRET = 0.4;
    public static double EXTEND_SLIDES = 0.4;
    public static double RETRACT_WAIT_FOR_HOPPER = 0.2;
    public static double RETRACT_SLIDES = 0.4;
    public static double RETRACT_TURRET = 0.4;
    public static double RETRACT_DOWN = 0.6;

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
    public static int TURRET_GENERAL = -130;
    public static int TURRET_SHARED = -663;
    public static int TURRET_ALLIANCE = 680;

    public static int SLIDES_GENERAL = 800;
    public static int SLIDES_AUTO = 150;
    public static int SLIDES_SHARED = 30;
    public static int SLIDES_ALLIANCE_LOW = 691;
    public static int SLIDES_ALLIANCE_MID = 635;
    public static int SLIDES_ALLIANCE_HIGH = 780;


    public static double ARM_PIVOT_INTERMEDIATE_POSITION = 0.5; //recent recal, hopper +0.06 , pivot unchanged
    public static ArmPosition ARM_PIVOT_POSITION = new ArmPosition(0.9, 0.9, 0.9, 0.48, 0.48, 0.09, 0.01, 0.1, 0.24, 0.8, 0.09, 0.01, 0.1, 0.24);
    public static ArmPosition ARM_HOPPER_POSITION = new ArmPosition(0.55+0.06, 0.55+0.06, 0.55+0.06, 0.4+0.06, 0.55+0.06, 0.2+0.06, 0.24+0.06, 0.2+0.06, 0.25+0.06, 0.99, 0.66+0.06, 0.48+0.06, 0.48+0.06, 0.6+0.06);

    public void clearMemory() {
        TURRET_GENERAL = -130;
        TURRET_SHARED = -663;
        TURRET_ALLIANCE = 680;

        SLIDES_GENERAL = 800;
        SLIDES_SHARED = 30;
        SLIDES_ALLIANCE_LOW = 691;
        SLIDES_ALLIANCE_MID = 635;
        SLIDES_ALLIANCE_HIGH = 780;

        ARM_PIVOT_POSITION = new ArmPosition(0.9, 0.9, 0.9, 0.48, 0.48, 0.09, 0.01, 0.1, 0.24, 0.8, 0.09, 0.01, 0.1, 0.24);
        ARM_HOPPER_POSITION = new ArmPosition(0.55+0.06, 0.55+0.06, 0.55+0.06, 0.4+0.06, 0.55+0.06, 0.2+0.06, 0.24+0.06, 0.2+0.06, 0.25+0.06, 0.99, 0.66+0.06, 0.48+0.06, 0.48+0.06, 0.6+0.06);

//        ARM_PIVOT_POSITION = new ArmPosition(0.9, 0.9, 0.9, 0.48, 0.48, 0.09, 0.01, 0.1, 0.24, 0.8, 0.09, 0.01, 0.1, 0.24);
//        ARM_HOPPER_POSITION = new ArmPosition(0.55+0.06, 0.55+0.06, 0.55+0.06, 0.4+0.06, 0.55+0.06, 0.2+0.06, 0.24+0.06, 0.2+0.06, 0.25+0.06, 0.99, 0.66+0.06, 0.48+0.06, 0.48+0.06, 0.6+0.06);
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

//        this.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void setIntakePower(double power) {
        intake.setPower(power);
        INTAKE_PID_MODE = false;
    }

    public void setIntakePosition(int position) {
        intakeController.setSetPoint(position);
        INTAKE_PID_MODE = true;
    }

    public boolean intakeIsReset() {
        return intakeController.atSetPoint();
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
        if (INTAKE_PID_MODE) {
            this.intake.setPower(intakeController.calculate(intake.getCurrentPosition()));
        }
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

    public void runningDeposit(double currentTime, Alliance alliance, BarcodeLocation barcodeLocation) {
//        runningDeposit_OLD(currentTime, alliance, barcodeLocation);
        runningRetract(currentTime, alliance, barcodeLocation == LEFT ? LOW : (barcodeLocation == MIDDLE ? MID : HIGH));
    }

    // NEW MACROS
    public void runningExtend(double currentTime, Alliance alliance, DepositPosition depoPos) {
        if (runningExtend) {
            switch(state) {
                // arm full
                case 0:
                    setSlides(0);
                    if (depoPos == GENERAL) {
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostHigh());
                    } else if (depoPos == SHARED) {
                        setArmPivot(ARM_PIVOT_POSITION.getUp());
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
                    if (currentTime > time + 0.5) {
                        time = currentTime;
                        state++;
                    }
                    break;
                case 2:
                    if (currentTime > time + 0.2) {
                        intakeRetracted = true;
                        setIntakeServo(INTAKE_SERVO_UP);
                    }
                    if (depoPos == GENERAL) {
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostHigh());
                    } else if (depoPos == SHARED) {
                        setArmHopper(ARM_HOPPER_POSITION.getUp());
                    } else if (depoPos == LOW) {
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostLow());
                    } else if (depoPos == MID) {
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostMid());
                    } else if (depoPos == HIGH) {
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostHigh());
                    }
                    if (currentTime > time + 0.3) {
                        time = currentTime;
                        state++;
                    }
                    break;
                // turret and slides
                case 3:
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
                    time = currentTime;
                    state++;
                    break;
                case 4:
                    if (depoPos == SHARED && currentTime > time + 0.1) {
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostShared());
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostShared());
                    }
                    if (turretController.atSetPoint()) {
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
                    }
                    break;
                case 5:
                    if (slidesController.atSetPoint()) {
                        time = currentTime;
                        state++;
                    }
                    break;
                // finish
                case 6:
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

//                    if (depoPos == GENERAL) { // this is meant to be used for capping, so it doesn't need to drop off
//                        state = 4;
//                        break;
//                    }

                    if (depoPos == SHARED) {
                        setArmHopper(ARM_HOPPER_POSITION.getShared());
                        setArmPivot(ARM_PIVOT_POSITION.getShared());
                    } else if (depoPos == LOW) {
                        setArmHopper(ARM_HOPPER_POSITION.getLow());
                        setArmPivot(ARM_PIVOT_POSITION.getLow());
                    } else if (depoPos == MID) {
                        setArmHopper(ARM_HOPPER_POSITION.getMid());
                        setArmPivot(ARM_PIVOT_POSITION.getMid());
                    } else if (depoPos == HIGH || depoPos == GENERAL) {
                        setArmHopper(ARM_HOPPER_POSITION.getHigh());
                        setArmPivot(ARM_PIVOT_POSITION.getHigh());
                    }

                    justCancledMacro = false;

                    time = currentTime;
                    state++;
                    break;
                // wait for freight to fall out
                case 1:
                    if (currentTime > time + 0.15) {
                        //if (!hopperIsFull()) {
                        if(getHopperDistance()>30) {
                            intakeRetracted = false;
                            setIntakeServo(INTAKE_SERVO_DOWN);
                            state++;
                        }
                    }
                    break;
                // move hopper out of danger grabbing zone
                case 2:
                    if (depoPos == GENERAL) {
                        setArmHopper(ARM_HOPPER_POSITION.getAlmostHigh());
                        setArmPivot(ARM_PIVOT_POSITION.getAlmostHigh());
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
                    // update variable to keep track of blocks for when to rumble
                    hasBlock = false;
                    setSlides(0);
                    time = currentTime;
                    state++;
                    break;
                case 5:
                    if (slidesController.atSetPoint()) {
                        state++;
                    }
                    break;
                case 6:
                    setTurret(0);
                    setArmPivot(ARM_PIVOT_INTERMEDIATE_POSITION);
                    time = currentTime;
                    state++;
                    break;
                case 7:
                    if (turretController.atSetPoint()) { //currentTime > time + RETRACT_TURRET ||
                        state++;
                    }
                    break;
                // return arm down
                case 8:
                    setArmPivot(ARM_PIVOT_POSITION.getDown());
                    setArmHopper(ARM_HOPPER_POSITION.getDown());
                    time = currentTime;
                    state++;
                    break;
                case 9:
                    if (currentTime > time + RETRACT_DOWN) {
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
