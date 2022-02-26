package org.firstinspires.ftc.teamcode.opmodes;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_RANGE;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_RANGE;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.DUCKY_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_UP;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ODO_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ODO_SERVO_UP;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.SLIDES_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.SLIDES_RANGE;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.TURRET_RANGE;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.TURRET_SPEED;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.LEFT;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.MIDDLE;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.RIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CameraPosition;
import org.firstinspires.ftc.teamcode.util.controller.Controller;

@Config
public class AbstractTeleOp extends OpMode {

    //this is for reseting the intake to the upright position
    public double intakeVerticalPos = 0;

    public static double INTAKE_SPEED = 0.25;
    public static double INTAKE_SLOW_SPEED = 0.15;
    public static double INTAKE_MID_SPEED = 0.4;

    Alliance alliance;

    private Controller driver1;
    private Controller driver2;

    private Robot robot;

    private int turretPosition = 0;
    private int slidesPosition = 0;
    private double armHopperPosition = ARM_HOPPER_POSITION.getInit();
    private double armPivotPosition = ARM_PIVOT_POSITION.getInit();

    public boolean odoRetracted;
    public boolean intakeRetracted;

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

        robot = new Robot(hardwareMap, alliance);

        intakeVerticalPos = robot.actuators.getIntakePosition();
        setAlliance();
    }

    @Override
    public void init_loop() {
//        if (robot.camera.getFrameCount() > 0) {
//            telemetry.addLine("Alliance: "+alliance);
//            telemetry.addLine(robot.getTelemetry());
//            telemetry.update();
//        }
        robot.lights.setPattern();
        telemetry.addLine(("Initialized: " + alliance + " alliance selected."));
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.updateLights();
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
        robot.drive.setWeightedDrivePower(new Pose2d(y, -x, -z));
        robot.drive.update();

        // automation
        if (!(robot.actuators.pickingUpFreight || robot.actuators.runningAlliance || robot.actuators.runningShared || robot.actuators.runningDeposit)) {
            if (false) {
                robot.actuators.pickingUpFreight = true;
            } else if (!driver2.getStart().isPressed() && driver2.getX().isJustPressed()) {
                robot.actuators.runningAlliance = true;
            } else if (!driver2.getStart().isPressed() && driver2.getB().isJustPressed()) {
                robot.actuators.runningShared = true;
            } else if (!driver2.getStart().isPressed() && driver2.getA().isJustPressed()) {
                robot.actuators.runningDeposit = true;
            }
        } else if (robot.actuators.pickingUpFreight && !(robot.actuators.runningAlliance || robot.actuators.runningShared || robot.actuators.runningDeposit)) {
            if (!driver2.getStart().isPressed() && driver2.getX().isJustPressed()) {
                robot.actuators.allianceQueue = true;
            } else if (!driver2.getStart().isPressed() && driver2.getA().isJustPressed()) {
                robot.actuators.sharedQueue = true;
            }
        } else if ((robot.actuators.runningAlliance || robot.actuators.runningShared) && !(robot.actuators.pickingUpFreight || robot.actuators.runningDeposit)) {
            if (!driver2.getStart().isPressed() && driver2.getA().isJustPressed()) {
                robot.actuators.depositQueue = true;
            }
        }

        if (robot.actuators.runningAlliance) {
            robot.actuators.runningAlliance(getRuntime(), alliance, RIGHT);
        } else if (robot.actuators.runningShared) {
            robot.actuators.runningShared(getRuntime(), alliance, LEFT);
        } else if (robot.actuators.runningDeposit) {
            robot.actuators.runningDeposit(getRuntime(), alliance, RIGHT);
        } else {
            if (robot.actuators.justFinishedAMacro) {
                turretPosition = robot.actuators.getTurret();
                slidesPosition = robot.actuators.getSlides();
                armPivotPosition = robot.actuators.getArmPivot();
                armHopperPosition = robot.actuators.getArmHopper();
                robot.actuators.justFinishedAMacro = false;
            }
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

            turretPosition = clamp(turretPosition, -1000, 1000);
            slidesPosition = clamp(slidesPosition, 0, 2500);

//            turretPosition = clamp(turretPosition, TURRET_RANGE.getMin(), TURRET_RANGE.getMax());
//            slidesPosition = clamp(slidesPosition, SLIDES_RANGE.getMin(), SLIDES_RANGE.getMax());
            armHopperPosition = clamp(armHopperPosition, ARM_HOPPER_RANGE.getDoubleMin(), ARM_HOPPER_RANGE.getDoubleMax());
            armPivotPosition = clamp(armPivotPosition, ARM_PIVOT_RANGE.getDoubleMin(), ARM_PIVOT_RANGE.getDoubleMax());
            robot.actuators.setTurret(turretPosition);
            robot.actuators.setSlides(slidesPosition);
            robot.actuators.setArmHopper(armHopperPosition);
            robot.actuators.setArmPivot(armPivotPosition);

            //cancel macro button
            if(driver2.getLeftStickButton().isJustPressed() || driver2.getRightStickButton().isJustPressed()){
                robot.actuators.runningDeposit = false;
                robot.actuators.runningAlliance = false;
                robot.actuators.runningShared = false;
                robot.actuators.setState(0);
                //robot.actuators.runningGenericExtend = false

            }

            // intake
            if (driver2.getLeftBumper().isPressed()) {
                if (driver2.getRightTrigger().getValue() > 0.1) {
                    robot.actuators.setIntake(-driver2.getRightTrigger().getValue() * INTAKE_SLOW_SPEED);
                } else if (driver2.getLeftTrigger().getValue() > 0.1) {
                    robot.actuators.setIntake(driver2.getLeftTrigger().getValue() * INTAKE_SLOW_SPEED);
                } else {
                    robot.actuators.setIntake(0);
                }
            } else if (driver2.getRightBumper().isJustPressed()) {
                int newPos = (int) (robot.actuators.getIntakePosition() + intakeVerticalPos - (robot.actuators.getIntakePosition() % (145.1)));
                robot.actuators.setIntakePosition(newPos);
            } else if (driver2.getRightBumper().isPressed()) {
                robot.actuators.resetIntake();
            } else if (driver2.getBack().isPressed()) {
                if (driver2.getRightTrigger().getValue() > 0.1) {
                    robot.actuators.setIntake(-driver2.getRightTrigger().getValue() * INTAKE_MID_SPEED);
                } else if (driver2.getLeftTrigger().getValue() > 0.1) {
                    robot.actuators.setIntake(driver2.getLeftTrigger().getValue() * INTAKE_MID_SPEED);
                } else {
                    robot.actuators.setIntake(0);
                }

            } else {
                if (driver2.getRightTrigger().getValue() > 0.1) {
                    robot.actuators.setIntake(-driver2.getRightTrigger().getValue() * INTAKE_SPEED);
                } else if (driver2.getLeftTrigger().getValue() > 0.1) {
                    robot.actuators.setIntake(driver2.getLeftTrigger().getValue() * INTAKE_SPEED);
                } else {
                    robot.actuators.setIntake(0);
                }
            }
        }
        robot.actuators.update();

        // duckies
        if (!driver2.getBack().isPressed()) {
            robot.actuators.setDuckies(driver2.getY().isPressed() ? DUCKY_SPEED : 0, alliance);
        } else {
            robot.actuators.setDuckies(0, alliance);
        }

        // retractables
        if (driver1.getX().isJustPressed()) {
            intakeRetracted = !intakeRetracted;
        }
        if (driver1.getA().isJustPressed()) {
            odoRetracted = !odoRetracted;
        }
        robot.actuators.setIntakeServo(intakeRetracted ? INTAKE_SERVO_UP : INTAKE_SERVO_DOWN);
        robot.actuators.setOdoServo(odoRetracted ? ODO_SERVO_UP : ODO_SERVO_DOWN);

        if (driver1.getBack().isJustPressed()) {
            if (alliance == RED) {
                alliance = BLUE;
            } else if (alliance == BLUE) {
                alliance = RED;
            }
        }

        // intake
//        if (driver1.getDUp().isPressed()) {
//            intakeServoPosition += INTAKE_SERVO_SPEED;
//        } else if (driver1.getDDown().isPressed()) {
//            intakeServoPosition -= INTAKE_SERVO_SPEED;
//        }
//        robot.actuators.setIntakeServo(intakeServoPosition);

        // telemetry
        telemetry.addLine("Alliance: " + alliance);
        telemetry.addLine(robot.getTelemetry());
        telemetry.update();
    }
}
