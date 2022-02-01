package org.firstinspires.ftc.teamcode.opmodes;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_RANGE;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_RANGE;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.DUCKY_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.SLIDES_RANGE;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.SLIDES_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.TURRET_RANGE;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.TURRET_SPEED;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.ArmPosition;
import org.firstinspires.ftc.teamcode.util.controller.Controller;

@Config
public class AbstractTeleOp extends OpMode {
//    public static double HOPPER_SERVO_DOWN = 0.68;
//    public static double HOPPER_SERVO_INTAKE = 0.28;
//    public static double SLIDES_SERVO_DOWN = 0.34;
//    public static double HOPPER_SERVO_UP = 0.99;
//    public static double SLIDES_SERVO_UP = 0.8;
//    public static double HOPPER_SERVO_DEPOSIT = 0.99;
//    public static double SLIDES_SERVO_DEPOSIT = 0.99;
    public static ArmPosition ARM_PIVOT_POSITION = new ArmPosition(0.01, 0.1, 0.42, 0.75);
    public static ArmPosition ARM_HOPPER_POSITION = new ArmPosition(0.66, 0.75, 0.74, 0.59);

    public static int TURRET_ALLIANCE = 650;
    public static int TURRET_SHARED = -800;
    public static int SLIDES_ALLIANCE = 2400;
    public static int SLIDES_SHARED = 0;

    public static double DEPOSIT1 = 0.4;
    public static double DEPOSIT2 = 0.6;
    public static double DEPOSIT3 = 0.6;
    public static double DEPOSIT4 = 1.3;

    //this is for telling when we should be doing the memory stuff for adjusting the macro.
    public boolean justFinishedAllianceMacro = false;

    //this is for reseting the intake to the upright position
    public double intakeVerticalPos = 0;

    public static double RETRACT1 = 0.4;
    public static double RETRACT2 = 0.6;
    public static double RETRACT3 = 0.6;
    public static double RETRACT4 = 0.6;
    public static double RETRACT5 = 0.25;

    public static double INTAKE_SPEED = 1.0;
    public static double INTAKE_SLOW_SPEED = 0.25;


    Alliance alliance;

    private Controller driver1;
    private Controller driver2;

    private Robot robot;

    private int turretPosition = 0;
    private int slidesPosition = 0;
//    private double armHopperPosition = ARM_HOPPER_POSITION.getDown();
//    private double armPivotPosition = ARM_PIVOT_POSITION.getDown();
    private double armHopperPosition = ARM_HOPPER_POSITION.getUp();
    private double armPivotPosition = ARM_PIVOT_POSITION.getUp();

    // state machine variables
    private boolean runningAlliance;
    private boolean runningShared;
    private boolean runningDeposit;

    private double currentTime;
    private int state;
    private double time;

    private double intakeTime;


    public void setAlliance() {
        alliance = RED;
    }

    @Override
    public void init() {
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        setAlliance();

        driver1 = new Controller(gamepad1);
        driver2 = new Controller(gamepad2);

        robot = new Robot(hardwareMap);

        intakeVerticalPos = robot.actuators.getIntakePosition();
    }

    @Override
    public void init_loop() {
//        if (robot.camera.getFrameCount() > 0) {
//            telemetry.addLine("Alliance: "+alliance);
//            telemetry.addLine(robot.getTelemetry());
//            telemetry.update();
//        }
        telemetry.addLine(("Initialized: "+alliance+" alliance selected."));
        telemetry.update();
    }

    @Override
    public void loop() {
        currentTime = getRuntime();
        driver1.update();
        driver2.update();

        // drive base
        double x, y, z;
        if (driver1.getLeftBumper().isPressed()) {
            x = driver1.getLeftStick().getX();
            y = driver1.getLeftStick().getY();
            z = driver1.getRightStick().getX();
        } else {
            x = driver1.getLeftStick().getX() * DRIVE_SPEED;
            y = driver1.getLeftStick().getY() * DRIVE_SPEED;
            z = driver1.getRightStick().getX() * DRIVE_SPEED;
        }
        robot.drive.setWeightedDrivePower(new Pose2d(y,-x,-z));

        // intake
        if (driver2.getLeftBumper().isPressed()) {
            if (driver2.getRightTrigger().getValue() > 0.1) {
                robot.actuators.setIntake(-driver2.getRightTrigger().getValue() * INTAKE_SLOW_SPEED);
            } else if (driver2.getLeftTrigger().getValue() > 0.1) {
                robot.actuators.setIntake(driver2.getLeftTrigger().getValue() * INTAKE_SLOW_SPEED);
            } else {
                robot.actuators.setIntake(0);
            }
        } else if(driver2.getRightBumper().isPressed()){
            //robot.actuators.setIntakePosition((int) intakeVerticalPos); //swap with next line when it unwinds completly
            int newPos = (int) (robot.actuators.getIntakePosition() + intakeVerticalPos - (robot.actuators.getIntakePosition()  % (145.1)));
//            if (robot.actuators.getIntakePosition()  % (145.1)   >   145.1/2){
//                newPos = (int) (newPos - 145.1);
//            }
            robot.actuators.setIntakePosition(newPos);
        } else {
            if (driver2.getRightTrigger().getValue() > 0.1) {
                robot.actuators.setIntake(-driver2.getRightTrigger().getValue() * INTAKE_SPEED);
            } else if (driver2.getLeftTrigger().getValue() > 0.1) {
                robot.actuators.setIntake(driver2.getLeftTrigger().getValue() * INTAKE_SPEED);
            } else {
                robot.actuators.setIntake(0);
            }
        }
//        if (driver2.getRightTrigger().getValue() > 0.1) {
//            robot.actuators.setIntake(-driver2.getRightTrigger().getValue());
//        } else if (driver2.getLeftTrigger().getValue() > 0.1) {
//            robot.actuators.setIntake(driver2.getLeftTrigger().getValue());
//        }
//        } else  if (driver2.getLeftTrigger().isJustReleased() || driver2.getRightTrigger().isJustReleased()) {
////            robot.actuators.resetIntake();
////            intakeTime = getRuntime();
//        } else


        // deposit mechanism

//         automation
        if (driver2.getBack().isPressed() && driver2.getX().isJustPressed() && !(runningAlliance || runningShared || runningDeposit)) {
            runningAlliance = true;
            state = 0;
        } else if (driver2.getBack().isPressed() && driver2.getB().isJustPressed() && !(runningAlliance || runningShared || runningDeposit)) {
            runningShared = true;
            state = 0;
        } else if (driver2.getBack().isPressed() && driver2.getA().isJustPressed() && !(runningAlliance || runningShared || runningDeposit)) {
            runningDeposit = true;
            state = 0;
        }


        if (runningAlliance) {
            switch (state) {
                case 0:
                    time = getRuntime();
                    robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getAlmostDown());
                    robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getAlmostDown());
                    state++;
                    break;
                case 1:
                    if (getRuntime() > time + DEPOSIT1) { state++; }
                    break;
                case 2:
                    time = getRuntime();
                    robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getUp());
                    robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getUp());
                    state++;
                    break;
                case 3:
                    if (getRuntime() > time + DEPOSIT2) { state++; }
                    break;
                case 4:
                    time = getRuntime();
                    robot.actuators.setTurret(alliance == RED ? TURRET_ALLIANCE : -TURRET_ALLIANCE);
                    state++;
                    break;
                case 5:
                    if (getRuntime() > time + DEPOSIT3) { state++; }
                    break;
                case 6:
                    time = getRuntime();
                    robot.actuators.setSlides(SLIDES_ALLIANCE);
                    robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getDeposit());
                    robot.actuators.setArmHopper(.99);
                    state++;
                    break;
                case 7:
                    if (getRuntime() > time + DEPOSIT4) { state++; }
                    break;
                case 8:
                    runningAlliance = false;
                    turretPosition = robot.actuators.getTurret();
                    slidesPosition = robot.actuators.getSlides();
                    armPivotPosition = robot.actuators.getArmPivot();
                    armHopperPosition = robot.actuators.getArmHopper();
                    justFinishedAllianceMacro = true;
            }
        } else if (runningShared) {
            switch (state) {
                case 0:
                    time = getRuntime();
                    robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getAlmostDown());
                    robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getAlmostDown());
                    state++;
                    break;
                case 1:
                    if (getRuntime() > time + 1) { state++; }
                    break;
                case 2:
                    time = getRuntime();
                    robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getUp());
                    robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getUp());
                    state++;
                    break;
                case 3:
                    if (getRuntime() > time + 1.1) { state++; }
                    break;
                case 4:
                    time = getRuntime();
                    robot.actuators.setTurret(alliance == RED ? TURRET_SHARED : -TURRET_SHARED);
                    state++;
                    break;
                case 5:
                    if (getRuntime() > time + 1.2) { state++; }
                    break;
                case 6:
                    time = getRuntime();
                    robot.actuators.setSlides(SLIDES_SHARED);
                    robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getDeposit());
                    robot.actuators.setArmHopper(.99);
                    state++;
                    break;
                case 7:
                    if (getRuntime() > time + 1.3) { state++; }
                    break;
                case 8:
                    runningAlliance = false;
                    turretPosition = robot.actuators.getTurret();
                    slidesPosition = robot.actuators.getSlides();
                    armPivotPosition = robot.actuators.getArmPivot();
                    armHopperPosition = robot.actuators.getArmHopper();
            }
        } else if (runningDeposit) {
            switch (state) {
                case 0:
                    //"memory" stuff
                    if (justFinishedAllianceMacro){
                        TURRET_ALLIANCE = robot.actuators.getTurret();
                        SLIDES_ALLIANCE = robot.actuators.getSlides();
                        //robot.actuators.getArmPivot();
                        //robot.actuators.getArmHopper();
                    }
                    time = getRuntime();
                    robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getDeposit());
                    state++;
                    break;
                case 1:
                    if (getRuntime() > time + RETRACT1) { state++; }
                    break;
                case 2:
                    time = getRuntime();
                    robot.actuators.setSlides(0);
                    robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getUp());
                    robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getUp());
                    state++;
                    break;
                case 3:
                    if (getRuntime() > time + RETRACT2) { state++; }
                    break;
                case 4:
                    time = getRuntime();
                    robot.actuators.setTurret(0);
                    state++;
                    break;
                case 5:
                    if (getRuntime() > time + RETRACT3) { state++; }
                    break;
                case 6:
                    time = getRuntime();
                    robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getAlmostDown());
                    robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getAlmostDown());
                    state++;
                    break;
                case 7:
                    if (getRuntime() > time + RETRACT4) { state++; }
                    break;
                case 8:
                    time = getRuntime();
                    robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getDown());
                    robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getDown());
                    state++;
                    break;
                case 9:
                    if (getRuntime() > time + RETRACT5) { state++; }
                    break;
                case 10:
                    runningDeposit = false;
                    turretPosition = robot.actuators.getTurret();
                    slidesPosition = robot.actuators.getSlides();
                    armPivotPosition = robot.actuators.getArmPivot();
                    armHopperPosition = robot.actuators.getArmHopper();
            }
        }

        // manual
        else {
            turretPosition += driver2.getLeftStick().getX() * TURRET_SPEED;
            slidesPosition += driver2.getRightStick().getY() * SLIDES_SPEED;

            if (driver2.getDUp().isPressed()) {
                armPivotPosition += ARM_PIVOT_SPEED;
            } else if (driver2.getDDown().isPressed()) {
                armPivotPosition -= ARM_PIVOT_SPEED;
            }
            if (driver2.getDLeft().isPressed()) {
                armHopperPosition += ARM_HOPPER_SPEED;
            } else if (driver2.getDRight().isPressed()) {
                armHopperPosition -= ARM_HOPPER_SPEED;
            }

//            turretPosition = clamp(turretPosition, TURRET_RANGE.getMin(), TURRET_RANGE.getMax());
//            slidesPosition = clamp(slidesPosition, SLIDES_RANGE.getMin(), SLIDES_RANGE.getMax());
//            armHopperPosition = clamp(armHopperPosition, ARM_HOPPER_RANGE.getDoubleMin(), ARM_HOPPER_RANGE.getDoubleMax());
//            armPivotPosition = clamp(armPivotPosition, ARM_PIVOT_RANGE.getDoubleMin(), ARM_PIVOT_RANGE.getDoubleMax());
            turretPosition = clamp(turretPosition, -1000, 1000);
            slidesPosition = clamp(slidesPosition, 0, 2400);
            armHopperPosition = clamp(armHopperPosition, 0.01, 0.99);
            armPivotPosition = clamp(armPivotPosition, 0.01, 0.99);

            robot.actuators.setTurret(turretPosition);
            robot.actuators.setSlides(slidesPosition);
            robot.actuators.setArmHopper(armHopperPosition);
            robot.actuators.setArmPivot(armPivotPosition);
        }

        // duckies
        robot.actuators.setDuckies(driver2.getY().isPressed() ? DUCKY_SPEED : 0, alliance);

        robot.actuators.update();

        // telemetry
        telemetry.addLine("Alliance: "+alliance);
        telemetry.addLine(robot.getTelemetry());
        telemetry.update();
    }
}
