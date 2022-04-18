package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_UP;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.SLIDES_AUTO;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.TURRET_ALLIANCE;
import static org.firstinspires.ftc.teamcode.hardware.Lights.BLUEINIT;
import static org.firstinspires.ftc.teamcode.hardware.Lights.REDINIT;
import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.PoseStorage;
import org.firstinspires.ftc.teamcode.opmodes.Step;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BarcodeLocation;
import org.firstinspires.ftc.teamcode.util.CameraPosition;
import org.firstinspires.ftc.teamcode.util.DepositPosition;

import java.util.ArrayList;
import java.util.Locale;

public abstract class AbstractAuto extends LinearOpMode {

    Alliance alliance;
    CameraPosition cameraPosition;
    public Robot robot;

    private BarcodeLocation teamElementLocation = BarcodeLocation.RIGHT;
    private ArrayList<Step> steps;
    private double currentRuntime;

    // Main method to run all the steps for autonomous
    @Override
    public void runOpMode() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // initialize robot
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        setAlliance();
        setCameraPosition();
        PoseStorage.POSE_IS_DEFAULT = false;

        if (useCamera()) {
            robot = new Robot(hardwareMap, cameraPosition, alliance);
            robot.lights.setPattern(alliance == RED ? REDINIT : BLUEINIT);
            while (robot.camera.getFrameCount() < 1) {
                idle();
            }
        } else {
            robot = new Robot(hardwareMap, alliance);
            robot.lights.setPattern(alliance == RED ? REDINIT : BLUEINIT);
        }

        robot.actuators.odoRetracted = false;

        makeTrajectories();

        // set up into box
        robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getAlmostGeneral());
        robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getAlmostGeneral());
        sleep(1000);
        if (alliance == RED) {
            robot.actuators.setSlides(SLIDES_AUTO);
            robot.actuators.setTurret(TURRET_ALLIANCE);
        } else {
            robot.actuators.setSlides(SLIDES_AUTO);
            robot.actuators.setTurret(-TURRET_ALLIANCE);
        }
        double time = getRuntime();
        do {
            robot.actuators.update();
        } while (!(getRuntime() > time + 0.5));
        robot.actuators.setIntakeServo(INTAKE_SERVO_UP);
        robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getGeneral());
        robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getGeneral());

        // wait for start
        while (!(isStarted() || isStopRequested())) {
            robot.actuators.update();
//            robot.updateLights();
            telemetry.addLine("Initialized");
            if (useCamera()) {
                // april tag
                BarcodeLocation newLocation = robot.camera.checkTeamElementLocationFromAprilTag();
                // update location if a location is found
                if (newLocation != BarcodeLocation.UNKNOWN) {
                    teamElementLocation = newLocation;
                }
                telemetry.addLine(String.format(Locale.US, "Location: %s", teamElementLocation));
//                telemetry.addLine(robot.getTelemetryWithCamera());
                // opencv
//                teamElementLocation = robot.camera.checkTeamElementLocation();
//                telemetry.addLine(String.format(Locale.US, "Location: %s", teamElementLocation));
//                telemetry.addLine(String.format(Locale.US, "Size: %.4f", robot.camera.getTeamElement().getArea()));
            }
            telemetry.update();
        }
        resetStartTime();

        // build the first step
        steps = new ArrayList<>();
        if (useCamera()) {
            stopTargetingCamera();
        }
        initializeSteps(teamElementLocation);

        int stepNumber = 0;
        double stepTimeout;
        Step step = steps.get(stepNumber);
        stepTimeout = step.getTimeout() != -1 ? currentRuntime + step.getTimeout() : Double.MAX_VALUE;
        step.start();

        // run the remaining steps
        while (opModeIsActive()) {
            currentRuntime = getRuntime();
            // once a step finishes
            if (step.isFinished() || currentRuntime >= stepTimeout) {
                // do the finishing move
                step.end();
                stepNumber++;
                // if it was the last step break out of the while loop
                if (stepNumber > steps.size() - 1) {
                    break;
                }
                // else continue to the next step
                step = steps.get(stepNumber);
                stepTimeout = step.getTimeout() != -1 ? currentRuntime + step.getTimeout() : Double.MAX_VALUE;
                step.start();
            }

            // while the step is running display telemetry
            step.whileRunning();
            robot.actuators.update();
            robot.updateLights();
            // update PoseStorage for use in teleop
            PoseStorage.CURRENT_POSE = robot.drive.getPoseEstimate();
            telemetry.addLine(String.format(Locale.US, "Runtime: %.0f", currentRuntime));
            telemetry.addLine("Step " + (stepNumber + 1) + " of " + steps.size() + ", " + step.getTelemetry() + "\n");
            telemetry.addLine(robot.getTelemetry());
            telemetry.update();
        }
    }

    // Load up all of the steps for the autonomous: to be overridden with the specific steps in the specific auto
    public void initializeSteps(BarcodeLocation location) {
        addDelay(5);
    }

    //methods to be implemented in the specific autos
    public abstract void setAlliance();

    public abstract void setCameraPosition();

    public abstract boolean useCamera();

    public abstract void makeTrajectories();

    //other methods that do certain tasks

    public void turn(double degrees) {
        steps.add(new Step("Following a trajectory") {
            @Override
            public void start() {
                robot.drive.turn(degrees);
            }

            @Override
            public void whileRunning() {
                robot.drive.update();
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return !robot.drive.isBusy();
            }
        });
    }

    public void followTrajectory(Trajectory trajectory) {
        steps.add(new Step("Following a trajectory") {
            @Override
            public void start() {
                robot.drive.followTrajectoryAsync(trajectory);
            }

            @Override
            public void whileRunning() {
                robot.drive.update();
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return !robot.drive.isBusy();
            }
        });
    }

    public void resetIntake(double timeout) {
        steps.add(new Step("Resetting Intake", timeout) {
            @Override
            public void start() {
                int newPos = (int) (robot.actuators.getIntakePosition() + (145.1 / 8.0) - (robot.actuators.getIntakePosition() % (145.1)));
                robot.actuators.setIntakePosition(newPos);
            }

            @Override
            public void whileRunning() {
            }

            @Override
            public void end() {
                robot.actuators.setIntakePower(0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }

    public void stopTargetingCamera() {
        steps.add(new Step("Stopping Targeting Camera") {
            @Override
            public void start() {
                robot.camera.stopBarcodeWebcam();
            }

            @Override
            public void whileRunning() {
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        });
    }

    // Functions to add steps
    public void addDelay(double timeout) {
        steps.add(new Step("Waiting for " + timeout + " seconds", timeout) {
            @Override
            public void start() {
            }

            @Override
            public void whileRunning() {
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }

    public void addIntake(double timeout, final double intakePower) {
        steps.add(new Step("Setting intake power to " + intakePower, timeout) {
            @Override
            public void start() {
                robot.actuators.setIntakePower(intakePower);
            }

            @Override
            public void whileRunning() {
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }

    public void addArmHopper(double timeout, final double armHopperPosition) {
        steps.add(new Step("Setting hopper to " + armHopperPosition, timeout) {
            @Override
            public void start() {
                robot.actuators.setArmHopper(armHopperPosition);
            }

            @Override
            public void whileRunning() {
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }

    public void addArmPivot(double timeout, final double armPivotPosition) {
        steps.add(new Step("Setting hopper to " + armPivotPosition, timeout) {
            @Override
            public void start() {
                robot.actuators.setArmPivot(armPivotPosition);
            }

            @Override
            public void whileRunning() {
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }

    public void addTurret(double timeout, final int turretPos) {
        steps.add(new Step("Setting turret to " + turretPos, timeout) {
            @Override
            public void start() {
                robot.actuators.setTurret(turretPos);
            }

            @Override
            public void whileRunning() {
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }

    public void addSlides(double timeout, final int slidePos) {
        steps.add(new Step("Setting slide to " + slidePos, timeout) {
            @Override
            public void start() {
                robot.actuators.setSlides(slidePos);
            }

            @Override
            public void whileRunning() {
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }

    public void addDuckSpinner(double timeout, final double duckPower) {
        steps.add(new Step("Setting duck power to " + duckPower, timeout) {
            @Override
            public void start() {
                robot.actuators.setDuckies(duckPower, alliance == RED ? BLUE : RED);
            }

            @Override
            public void whileRunning() {
            }

            @Override
            public void end() {
                robot.actuators.setDuckies(0, alliance);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }

    public void addExtend(double timeout, Alliance alliance, DepositPosition depoPos) {
        steps.add(new Step("Extending", timeout) {
            @Override
            public void start() {
                robot.actuators.runningExtend = true;
            }

            @Override
            public void whileRunning() {
                robot.actuators.runningExtend(getRuntime(), alliance, depoPos);
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return !robot.actuators.runningExtend;
            }
        });
    }

    public void addRetract(double timeout, Alliance alliance, DepositPosition depoPos) {
        steps.add(new Step("Retracting", timeout) {
            @Override
            public void start() {
                robot.actuators.runningRetract = true;
            }

            @Override
            public void whileRunning() {
                robot.actuators.runningRetract(getRuntime(), alliance, depoPos);
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return !robot.actuators.runningRetract;
            }
        });
    }

    public void addIntakeServo(double timeout, double position) {
        steps.add(new Step("Depositing", timeout) {
            @Override
            public void start() {
                robot.actuators.setIntakeServo(position);
            }

            @Override
            public void whileRunning() {
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }

    // scoring stuff
    public void cycleBlockInAuto(double timeout, Trajectory trajectoryIn, Trajectory trajectoryOut, Alliance alliance, BarcodeLocation barcodeLocation) {
        steps.add(new Step("Cycling", timeout) {
            @Override
            public void start() {
                stepStartTime = currentRuntime;
                stepCaseStep = 0;
            }

            @Override
            public void whileRunning() {
                stepTime = currentRuntime - stepStartTime;
                switch (stepCaseStep) {
                    // while retracting, reset intake
                    case 0:
                        robot.actuators.setIntakePosition((int) (robot.actuators.getIntakePosition() + (robot.actuators.getIntakePosition() % 145.1)));
                        stepCaseStep++;
                        break;
                    // continue retracting
                    case 1:
                        // reset intake while retracting, then stop when arm is about to come down
                        if (robot.actuators.getState() >= 6) {
                            robot.actuators.setIntakePower(0);
                        }

                        // continue moving into warehouse
                        if (!robot.drive.isBusy()) {
                            robot.drive.followTrajectoryAsync(trajectoryIn);
                        }
                        // if the retract macro is done, start up intake and
                        if (!robot.actuators.runningRetract) {
                            robot.actuators.setIntakePower(-INTAKE_SPEED);
                            stepCaseStep++;
                        }
                        break;
                    // wait for block and drive out
                    case 2:
                        //if (robot.actuators.hopperIsFull()) {
                        if (robot.actuators.getHopperDistance() < 15) {
                            robot.drive.breakFollowing();
                            stepStartTime = currentRuntime;
                            stepCaseStep++;
                        }
                        break;
                    // after a bit, stop intake and extend while going out
                    case 3:
                        if (stepTime > 0.15) {
                            robot.actuators.setIntakePower(0);
                            robot.drive.followTrajectoryAsync(trajectoryOut);
                            robot.actuators.runningExtend = true;
                            stepStartTime = currentRuntime;
                            stepCaseStep++;
                        }
                        break;
                    // while going out, bring up the intake, and retract when done extending
                    case 4:
                        // reverse intake
                        if (stepTime > 0.9) {
                            robot.actuators.setIntakePower(0);
                        } else if (stepTime > 0.5) {
                            robot.actuators.setIntakePower(0.5);
                        }

                        // start retract when done extending
                        if (!robot.actuators.runningExtend) {
                            robot.actuators.runningRetract = true;
                            stepStartTime = currentRuntime;
                            stepCaseStep++;
                        }
                        break;
                    case 5:
                        robot.actuators.setIntakePower(0);
                        robot.actuators.setIntakePosition((int) (robot.actuators.getIntakePosition() + (robot.actuators.getIntakePosition() % 145.1)));
                        stepCaseStep++;
//                        robot.actuators.setIntakePower(0);
//                        if (stepTime > 0.2) {
//                            stepCaseStep++;
//                        }
                        // if the retract macro is almost done, start another cycle round
                    case 6:
                        if (robot.actuators.getState() >= 8) {
                            stepCaseStep++;
                        }
                        break;
                }

                // update the drive base pid
                robot.drive.update();
                // run the alliance macro if it is set to true
                robot.actuators.runningAlliance(getRuntime(), alliance, barcodeLocation);
                // run the deposit macro if set to true
                robot.actuators.runningDeposit(getRuntime(), alliance, barcodeLocation);
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return stepCaseStep == 7;
            }
        });
    }

    public void scorePreloadInAuto(double timeout, Alliance alliance, BarcodeLocation barcodeLocation, boolean cycling) {
        steps.add(new Step("Cycling", timeout) {
            @Override
            public void start() {
                stepStartTime = currentRuntime;
                stepCaseStep = 0;
            }

            @Override
            public void whileRunning() {
                stepTime = currentRuntime - stepStartTime;
                switch (stepCaseStep) {
                    // extend
                    case 0:
                        robot.actuators.setIntakeServo(INTAKE_SERVO_DOWN);
                        // set hopper in motion because the normal macro waits a bit
                        robot.actuators.setState(4);
                        robot.actuators.runningExtend = true;
                        if (barcodeLocation == BarcodeLocation.LEFT) {
                            robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getAlmostLow());
                        } else if (barcodeLocation == BarcodeLocation.MIDDLE) {
                            robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getAlmostMid());
                        } else if (barcodeLocation == BarcodeLocation.RIGHT || barcodeLocation == BarcodeLocation.UNKNOWN) {
                            robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getAlmostHigh());
                        }
                        stepCaseStep++;
                        stepStartTime = currentRuntime;
                        break;
                    case 1:
                        if (stepTime > 0) {
                            if (barcodeLocation == BarcodeLocation.LEFT) {
                                robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getAlmostLow());
                            } else if (barcodeLocation == BarcodeLocation.MIDDLE) {
                                robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getAlmostMid());
                            } else if (barcodeLocation == BarcodeLocation.RIGHT || barcodeLocation == BarcodeLocation.UNKNOWN) {
                                robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getAlmostHigh());
                            }
                            stepCaseStep++;
                        }
                    case 2:
                        if (!robot.actuators.runningExtend) {
                            stepCaseStep++;
                        }
                        break;
                    // retract
                    case 3:
                        robot.actuators.runningRetract = true;
                        stepCaseStep++;
                        break;
                    case 4:
                        if (cycling && robot.actuators.getState() >= 8) { // if the retract macro is almost done, start cycling
                            stepCaseStep++;
                        } else if (!robot.actuators.runningRetract) {
                            stepCaseStep++;
                        }
                        break;
                }

                //update the drive base pid
                robot.drive.update();
                //run the alliance macro if it is set to true
                robot.actuators.runningAlliance(getRuntime(), alliance, barcodeLocation);
                //run the deposit macro if set to true
                robot.actuators.runningDeposit(getRuntime(), alliance, barcodeLocation);
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return stepCaseStep == 5;
            }
        });
    }

    public void parkInAuto(double timeout, Alliance alliance, Trajectory path, BarcodeLocation barcodeLocation) {
        steps.add(new Step("parking", timeout) {
            @Override
            public void start() {
                robot.drive.followTrajectoryAsync(path);
            }

            @Override
            public void whileRunning() {
                robot.drive.update();
                robot.actuators.runningDeposit(getRuntime(), alliance, barcodeLocation);
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return (!robot.drive.isBusy() && !robot.actuators.runningRetract);
            }
        });
    }
}