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
import static org.firstinspires.ftc.teamcode.util.DepositPosition.GENERAL;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.HIGH;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.SHARED;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Alliance;
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

    public static double INTAKE_SPEED = 0.75;
    public static double INTAKE_SLOW_SPEED = 0.15;

    Alliance alliance;

    private Controller driver1;
    private Controller driver2;

    private Robot robot;

    private int turretPosition = 0;
    private int slidesPosition = 0;
    private double armHopperPosition = ARM_HOPPER_POSITION.getInit();
    private double armPivotPosition = ARM_PIVOT_POSITION.getInit();

    public DepositPosition extendTo = HIGH;
    public DepositPosition extendFrom = HIGH;

    public void setAlliance() {
        alliance = RED;
    }

    Trajectory pathToScore;

    @Override
    public void init() {
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        setAlliance();

        driver1 = new Controller(gamepad1);
        driver2 = new Controller(gamepad2);

        robot = new Robot(hardwareMap, alliance);

        // set the pose from auto
        if (PoseStorage.POSE_IS_DEFAULT) {
            PoseStorage.CURRENT_POSE = alliance == RED ? PoseStorage.START_RED : PoseStorage.START_BLUE;
        }
        robot.drive.setPoseEstimate(PoseStorage.CURRENT_POSE);
        PoseStorage.POSE_IS_DEFAULT = true;

        // reset positions every teleop
        robot.actuators.clearMemory();

        robot.actuators.intakeStartPos = 0;

        //robot.actuators.odoRetracted = false;
        robot.actuators.setOdoServo(ODO_SERVO_DOWN);
    }

    @Override
    public void init_loop() {
        robot.updateLights();
        telemetry.addLine(("Initialized: " + alliance + " alliance selected."));
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.updateLights();
        driver1.update();
        driver2.update();

        // check for rumble
        if (!robot.actuators.hasBlock && robot.actuators.hopperIsFull()) {
            driver1.rumble(500);
            driver2.rumble(500);
            robot.actuators.hasBlock = true;
        }

        // drive base
        switch (currentMode) {
            case DRIVER_CONTROL:
                // normal driver stuff
                double x, y, z;

                //new stuff with "exponential" speed

                //get the initial values
                x = driver1.getLeftStick().getY();
                y = -driver1.getLeftStick().getX();
                z = -driver1.getRightStick().getX();

                //transform the linear controller output into the nonlinear curve
                x =  0.152*Math.tan(1.42*x)  ; // blue desmos curve
                //y =  0.2*Math.tan(1.3734*y)  ;
                z =  0.152*Math.tan(1.42*z)  ;

                //old stuff with boost button
//                if (driver1.getLeftBumper().isPressed()) {
//                    x = driver1.getLeftStick().getY();
//                    y = -driver1.getLeftStick().getX();
//                    z = -driver1.getRightStick().getX();
//                } else {
//                    x = driver1.getLeftStick().getY(); //* DRIVE_SPEED;
//                    y = -driver1.getLeftStick().getX(); //* DRIVE_SPEED;
//                    z = -driver1.getRightStick().getX(); //* DRIVE_SPEED;
//                }

                robot.drive.setWeightedDrivePower(new Pose2d(x, y, z));

                // if x is pressed, go into automatic mode
                if (driver1.getX().isJustPressed()) {
                    extendTo = HIGH;
                    pathToScore = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), true)
                            .lineToSplineHeading((alliance == RED ? PoseStorage.SCORE_1_RED : PoseStorage.SCORE_1_BLUE))
                            .addDisplacementMarker(() -> {
                                robot.actuators.setIntakePower(0);
                                robot.actuators.runningExtend = true;
                            })
                            .splineToSplineHeading(alliance == RED ? PoseStorage.SCORE_1_1_RED : PoseStorage.SCORE_1_1_BLUE, Math.toRadians(180))
                            .lineToSplineHeading(alliance == RED ? PoseStorage.SCORE_2_RED : PoseStorage.SCORE_2_BLUE)
                            .build();
                    currentMode = Mode.AUTOMATIC_CONTROL;
                }

                // or if b is pressed, go into automatic mode
                if (driver1.getB().isJustPressed() && !driver1.getStart().isJustPressed()) {
                    extendTo = SHARED;
                    pathToScore = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), true)
                            .lineToSplineHeading((alliance == RED ? PoseStorage.SCORE_1_SHARED_RED : PoseStorage.SCORE_1_SHARED_BLUE))
                            .addDisplacementMarker(() -> {
                                robot.actuators.setIntakePower(0);
                                robot.actuators.runningExtend = true;
                            })
                            .splineToSplineHeading(alliance == RED ? PoseStorage.SCORE_1_1_SHARED_RED : PoseStorage.SCORE_1_1_SHARED_BLUE, Math.toRadians(180))
                            .lineToSplineHeading(alliance == RED ? PoseStorage.SCORE_2_SHARED_RED : PoseStorage.SCORE_2_SHARED_BLUE)
                            .build();
                    currentMode = Mode.AUTOMATIC_CONTROL;
                }
                break;
            case AUTOMATIC_CONTROL:
                switch(state) {
                    case 0:
                        robot.drive.followTrajectoryAsync(pathToScore);
                        robot.actuators.setIntakePosition((int) (robot.actuators.getIntakePosition() + (robot.actuators.getIntakePosition() % 145.1)));
                        state++;
                        break;
                    case 1:
                        if (!robot.drive.isBusy() && !robot.actuators.runningExtend) {
                            state++;
                        }
                        break;
                }

                // if drive finishes its task, cede control to the driver
                if (state == 2) {
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
//        PoseStorage.CURRENT_POSE = robot.drive.getPoseEstimate();

        // automation
        if (!(robot.actuators.runningExtend || robot.actuators.runningRetract)) {
            if (!driver2.getStart().isPressed() && driver2.getX().isJustPressed()) {
                // new:
                extendTo = HIGH;
                robot.actuators.runningExtend = true;
            } else if (!driver2.getStart().isPressed() && driver2.getB().isJustPressed()) {
                // new:
                extendTo = SHARED;
                robot.actuators.runningExtend = true;
            } else if (!driver2.getStart().isPressed() && driver2.getA().isJustPressed()) {
                // new:
                extendFrom = extendTo;
                robot.actuators.runningRetract = true;
            } else if (driver2.getY().isJustPressed()) {
                if (!robot.actuators.capPickedUp) {
                    robot.actuators.runningExtend = true;
                    extendTo = GENERAL;
                    robot.actuators.capPickedUp = true;
                } else {
                    armPivotPosition = ARM_PIVOT_POSITION.getGeneral();
                    armHopperPosition = ARM_HOPPER_POSITION.getGeneral();
                    slidesPosition = 348;
                    robot.actuators.capPickedUp = false;
                }
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


        }

        // intake
        if (driver2.getRightBumper().isJustPressed()) {
            robot.actuators.setIntakePosition((int) (robot.actuators.getIntakePosition() + (robot.actuators.getIntakePosition() % 145.1)));
        } else if (!driver2.getRightBumper().isPressed()) {
            if (driver2.getRightTrigger().getValue() > 0.1) {
                robot.actuators.setIntakePower(-driver2.getRightTrigger().getValue() * INTAKE_SPEED);
            } else if (driver2.getLeftTrigger().getValue() > 0.1) {
                robot.actuators.setIntakePower(driver2.getLeftTrigger().getValue() * INTAKE_SPEED);
            } else {
                robot.actuators.setIntakePower(0);
            }
        }

        // reset memory
        if (driver2.getLeftBumper().isJustPressed()) {
            robot.actuators.clearMemory();
            //reset turret pos so that the current value becomes the zero point
            robot.actuators.resetTurret();
            turretPosition = 0;
            robot.drive.setPoseEstimate(new Pose2d(10.6875, (alliance==RED ? -65.75 : 65.75), Math.toRadians(0)));
        }

        // cancel macro button
        if(driver2.getLeftStickButton().isJustPressed() || driver2.getRightStickButton().isJustPressed() || driver1.getLeftStickButton().isJustPressed() || driver1.getRightStickButton().isJustPressed()) {
            robot.actuators.justCancledMacro = true; //used to deactivate memory
            robot.actuators.justFinishedAMacro = true; // used in both
            robot.actuators.setState(0); // used in both

            // new macro variables
            robot.actuators.runningRetract = false;
            robot.actuators.runningExtend = false;
            robot.actuators.retractQueue = false;

            robot.actuators.clearMemory();
        }

        // duckies
        robot.actuators.setDuckies(driver1.getY().isPressed() ? DUCKY_SPEED : 0 , alliance);

        // retractables
        if (!driver2.getBack().isPressed() && driver2.getY().isJustPressed()) {
            robot.actuators.intakeRetracted = !robot.actuators.intakeRetracted;
        }
        if (!driver1.getBack().isPressed() && driver1.getA().isJustPressed()) {
            robot.actuators.odoRetracted = !robot.actuators.odoRetracted;
        }

        robot.actuators.setIntakeServo(robot.actuators.intakeRetracted ? INTAKE_SERVO_UP : INTAKE_SERVO_DOWN);
        robot.actuators.setOdoServo(robot.actuators.odoRetracted ? ODO_SERVO_UP : ODO_SERVO_DOWN);

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
