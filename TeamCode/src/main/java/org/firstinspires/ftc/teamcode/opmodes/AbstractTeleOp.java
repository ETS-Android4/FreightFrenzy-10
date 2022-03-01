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
import static org.firstinspires.ftc.teamcode.hardware.Actuators.TURRET_SPEED;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.HIGH;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.SHARED;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.DepositPosition;
import org.firstinspires.ftc.teamcode.util.controller.Controller;

@Config
public class AbstractTeleOp extends OpMode {
    public static double INTAKE_SPEED = 0.4;
    public static double INTAKE_SLOW_SPEED = 0.15;

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

    public DepositPosition extendTo = HIGH;
    public DepositPosition extendFrom = HIGH;

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

        setAlliance();

        robot.actuators.INTAKE_SERVO_START = 0;
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
            x = driver1.getLeftStick().getY();
            y = -driver1.getLeftStick().getX();
            z = -driver1.getRightStick().getX();
        } else {
            x = driver1.getLeftStick().getY() * DRIVE_SPEED;
            y = -driver1.getLeftStick().getX() * DRIVE_SPEED;
            z = -driver1.getRightStick().getX() * DRIVE_SPEED;
        }
        robot.drive.setWeightedDrivePower(new Pose2d(x, y, z));
        robot.drive.update();

        // automation
        if (!(robot.actuators.runningExtend || robot.actuators.runningRetract)) {
            if (!driver2.getStart().isPressed() && driver2.getX().isJustPressed()) {
                robot.actuators.runningExtend = true;
                extendTo = HIGH;
            } else if (!driver2.getStart().isPressed() && driver2.getB().isJustPressed()) {
                robot.actuators.runningExtend = true;
                extendTo = SHARED;
            } else if (!driver2.getStart().isPressed() && driver2.getA().isJustPressed()) {
                robot.actuators.runningRetract = true;
                extendFrom = extendTo;
            }
        } else if (robot.actuators.runningExtend && !robot.actuators.runningRetract) {
            if (!driver2.getStart().isPressed() && driver2.getA().isJustPressed()) {
                robot.actuators.retractQueue = true;
            }
        }

        if (robot.actuators.runningExtend) {
            robot.actuators.runningExtend(getRuntime(), alliance, extendTo);
        } else if (robot.actuators.runningRetract) {
            robot.actuators.runningRetract(getRuntime(), alliance, extendFrom);
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
            slidesPosition = clamp(slidesPosition, 0, (int)(2500*0.377373212));
            armHopperPosition = clamp(armHopperPosition, ARM_HOPPER_RANGE.getDoubleMin(), ARM_HOPPER_RANGE.getDoubleMax());
            armPivotPosition = clamp(armPivotPosition, ARM_PIVOT_RANGE.getDoubleMin(), ARM_PIVOT_RANGE.getDoubleMax());
            robot.actuators.setTurret(turretPosition);
            robot.actuators.setSlides(slidesPosition);
            robot.actuators.setArmHopper(armHopperPosition);
            robot.actuators.setArmPivot(armPivotPosition);

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
            int newPos = (int) (robot.actuators.getIntakePosition() - (robot.actuators.getIntakePosition() % (145.1)));
            robot.actuators.setIntakePosition(newPos);
        } else if (driver2.getRightBumper().isPressed()) {
            robot.actuators.resetIntake();
        } else {
            if (driver2.getRightTrigger().getValue() > 0.1) {
                robot.actuators.setIntake(-driver2.getRightTrigger().getValue() * INTAKE_SPEED);
            } else if (driver2.getLeftTrigger().getValue() > 0.1) {
                robot.actuators.setIntake(driver2.getLeftTrigger().getValue() * INTAKE_SPEED);
            } else {
                robot.actuators.setIntake(0);
            }
        }

        // cancel macro button
        if(driver2.getLeftStickButton().isJustPressed() || driver2.getRightStickButton().isJustPressed()) {
            robot.actuators.runningRetract = false;
            robot.actuators.runningExtend = false;
            robot.actuators.retractQueue = false;
            robot.actuators.justFinishedAMacro = true;
            robot.actuators.setState(0);
        }

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

        robot.actuators.update();
        // telemetry
        telemetry.addLine("Alliance: " + alliance);
        telemetry.addLine(robot.getTelemetry());
        telemetry.update();
    }
}
