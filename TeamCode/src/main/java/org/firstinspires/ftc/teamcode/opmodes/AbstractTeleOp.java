package org.firstinspires.ftc.teamcode.opmodes;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_MAX;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_MIN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_MAX;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_MIN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.DUCKY_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_UP;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ODO_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ODO_SERVO_UP;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.SLIDES_MAX;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.SLIDES_MIN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.SLIDES_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.TURRET_MAX;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.TURRET_MIN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.TURRET_SPEED;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.HIGH;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.SHARED;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Actuators;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.ArmPosition;
import org.firstinspires.ftc.teamcode.util.BarcodeLocation;
import org.firstinspires.ftc.teamcode.util.DepositPosition;
import org.firstinspires.ftc.teamcode.util.controller.Controller;

@Config
public class AbstractTeleOp extends OpMode {
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    public int state = 0;

    Vector2d score1Pos = new Vector2d(40, 63);
    double score1Heading = Math.toRadians(0);
    Vector2d score2Pos = new Vector2d(12, 63);
    double score2Heading = Math.toRadians(0);
    Trajectory pathToScore;
    Trajectory pathToScore2;

    public static double INTAKE_SPEED = 0.5;
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
        // set the pose from auto
        robot.drive.setPoseEstimate(PoseStorage.currentPose);

        setAlliance();

        robot.actuators.intakeStartPos = 0;

        // reset positions every teleop
        robot.actuators.clearMemory();
    }

    @Override
    public void init_loop() {
//        if (robot.camera.getFrameCount() > 0) {
//            telemetry.addLine("Alliance: "+alliance);
//            telemetry.addLine(robot.getTelemetry());
//            telemetry.update();
//        }
        robot.updateLights();
        telemetry.addLine(("Initialized: " + alliance + " alliance selected."));
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.updateLights();
        driver1.update();
        driver2.update();

        // drive base
//        PoseStorage.currentPose = robot.drive.getPoseEstimate();
        switch (currentMode) {
            case DRIVER_CONTROL:
                // normal driver stuff
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

                // if x is pressed, go into automatic mode
                if (driver1.getX().isJustPressed()) {

                    // make a finite state machine here to chain the 2 trajectories!
                    pathToScore = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(score1Pos.getX(), score1Pos.getY(), score1Heading),
                                    SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                            )
//                            .lineToSplineHeading(new Pose2d(score2Pos.getX(), score2Pos.getY(), score2Heading))
//                            .splineTo(score1Pos, score1Heading)
//                            .splineTo(score2Pos, score2Heading)
                            //.addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(pathToScore2))
                            .build();

                    pathToScore2 = robot.drive.trajectoryBuilder(pathToScore.end())
//                            .lineToSplineHeading(new Pose2d(score1Pos.getX(), score1Pos.getY(), score1Heading))
                            .lineToSplineHeading(new Pose2d(score2Pos.getX(), score2Pos.getY(), score2Heading),
                                    SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                            )
//                            .splineTo(score1Pos, score1Heading)
//                            .splineTo(score2Pos, score2Heading)
                            .build();

                    currentMode = Mode.AUTOMATIC_CONTROL;
                }
                break;
            case AUTOMATIC_CONTROL:
                switch(state) {
                    case 0:
                        robot.drive.followTrajectoryAsync(pathToScore);
                        robot.actuators.setIntakePosition((int) (robot.actuators.intakeStartPos + (robot.actuators.getIntakePosition() - (robot.actuators.getIntakePosition() % 145.1))));
                        state++;
                        break;
                    case 1:
                        if (!robot.drive.isBusy() && robot.actuators.intakeIsReset()) {
                            state++;
                        }
                        break;
                    case 2:
                        robot.drive.followTrajectoryAsync(pathToScore2);
                        // start macro
                        robot.actuators.runningExtend = true;
                        extendTo = HIGH;
                        state++;
                        break;
                    case 3:
                        if (!robot.actuators.runningExtend) {
//                            robot.drive.setPoseEstimate(PoseStorage.currentPose);//maybe get rid of this
                            state++;
                        }
                        break;
                }
                robot.actuators.resetIntake();


                // if drive finishes its task, cede control to the driver
                if (state == 4) {
                    currentMode = Mode.DRIVER_CONTROL;
                    state = 0;
                }

                // if x is pressed, we break out of the automatic following
                if (driver1.getLeftStickButton().isJustPressed() || driver1.getRightStickButton().isJustPressed() || driver2.getLeftStickButton().isJustPressed() || driver2.getRightStickButton().isJustPressed()) {
                    robot.drive.breakFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                }
                break;
        }
        robot.drive.update();

        // automation
        if (!(robot.actuators.runningExtend || robot.actuators.runningRetract)) {
            if (!driver2.getStart().isPressed() && driver2.getX().isJustPressed()) {
                // new:
                robot.actuators.runningExtend = true;
                extendTo = HIGH;
            } else if (!driver2.getStart().isPressed() && driver2.getB().isJustPressed()) {
                // new:
                robot.actuators.runningExtend = true;
                extendTo = SHARED;
            } else if (!driver2.getStart().isPressed() && driver2.getA().isJustPressed()) {
                // new:
                robot.actuators.runningRetract = true;
                extendFrom = extendTo;
            }
        } else if ((robot.actuators.runningExtend) && !(robot.actuators.runningRetract)) {
            if (!driver2.getStart().isPressed() && driver2.getA().isJustPressed()) {
                // new:
                robot.actuators.retractQueue = true;
            }
        }

        // macro or manual control
        if (robot.actuators.runningExtend) {
            robot.actuators.runningExtend(getRuntime(), alliance, extendTo);
        } else if (robot.actuators.runningRetract) {
            robot.actuators.runningRetract(getRuntime(), alliance, extendFrom);
        }

//        if (robot.actuators.runningAlliance) {
//            robot.actuators.runningAlliance(getRuntime(), alliance, BarcodeLocation.RIGHT);
//        } else if (robot.actuators.runningShared) {
//            robot.actuators.runningShared(getRuntime(), alliance, BarcodeLocation.RIGHT);
//        } else if (robot.actuators.runningDeposit) {
//            robot.actuators.runningDeposit(getRuntime(), alliance, BarcodeLocation.RIGHT);
        else {
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

            turretPosition = clamp(turretPosition, TURRET_MIN, TURRET_MAX);
            slidesPosition = clamp(slidesPosition, SLIDES_MIN, SLIDES_MAX);
            armHopperPosition = clamp(armHopperPosition, ARM_HOPPER_MIN, ARM_HOPPER_MAX);
            armPivotPosition = clamp(armPivotPosition, ARM_PIVOT_MIN, ARM_PIVOT_MAX);

            robot.actuators.setTurret(turretPosition);
            robot.actuators.setSlides(slidesPosition);
            robot.actuators.setArmHopper(armHopperPosition);
            robot.actuators.setArmPivot(armPivotPosition);

            // intake
            if (driver2.getRightBumper().isJustPressed()) {
                int newPos = (int) (robot.actuators.getIntakePosition() - (robot.actuators.getIntakePosition() % (145.1)));
                robot.actuators.setIntakePosition(newPos);
            } else if (driver2.getRightBumper().isPressed()) {
                robot.actuators.resetIntake();
            } else {
                if (driver2.getRightTrigger().getValue() > 0.1) {
                    robot.actuators.setIntakePower(-driver2.getRightTrigger().getValue() * INTAKE_SPEED);
                } else if (driver2.getLeftTrigger().getValue() > 0.1) {
                    robot.actuators.setIntakePower(driver2.getLeftTrigger().getValue() * INTAKE_SPEED);
                } else {
                    robot.actuators.setIntakePower(0);
                }
            }
        }

        // reset memory
        if (driver2.getLeftBumper().isJustPressed()) {
            robot.actuators.clearMemory();
        }

        // cancel macro button
        if(driver2.getLeftStickButton().isJustPressed() || driver2.getRightStickButton().isJustPressed() || driver1.getLeftStickButton().isJustPressed() || driver1.getRightStickButton().isJustPressed()) {
            // old macro variables
//            robot.actuators.runningAlliance = false;
//            robot.actuators.runningDeposit = false;
//            robot.actuators.runningShared = false;
//            robot.actuators.sharedQueue = false;
//            robot.actuators.allianceQueue = false;
//            robot.actuators.justFinishedAllianceMacro = false;
//            robot.actuators.runningArm = false;
//            robot.actuators.pickingUpFreight = false;
//            robot.actuators.justFinishedSharedMacro = false;
//            robot.actuators.depositQueue = false;
            robot.actuators.justCancledMacro = true; //used to deactivate memory
            robot.actuators.justFinishedAMacro = true; // used in both
            robot.actuators.setState(0); // used in both

            // new macro variables
            robot.actuators.runningRetract = false;
            robot.actuators.runningExtend = false;
            robot.actuators.retractQueue = false;
        }

        // duckies
        if (!driver1.getBack().isPressed()) {
            robot.actuators.setDuckies(driver1.getY().isPressed() ? DUCKY_SPEED : 0, alliance);
        } else {
            robot.actuators.setDuckies(0, alliance);
        }

        // retractables
        if (!driver1.getBack().isPressed() && driver1.getB().isJustPressed()) {
            intakeRetracted = !intakeRetracted;
        }
        if (!driver1.getBack().isPressed() && driver1.getA().isJustPressed()) {
            odoRetracted = !odoRetracted;
        }

        robot.actuators.setIntakeServo(intakeRetracted ? INTAKE_SERVO_UP : INTAKE_SERVO_DOWN);
        robot.actuators.setOdoServo(odoRetracted ? ODO_SERVO_UP : ODO_SERVO_DOWN);

        // switch alliance button
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
