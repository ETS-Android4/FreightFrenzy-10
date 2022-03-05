package org.firstinspires.ftc.teamcode.opmodes;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_RESET_TIME;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_UP;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.TURRET_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmodes.AbstractTeleOp.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.opmodes.RedWarehouse.SCORE;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Actuators;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BarcodeLocation;
import org.firstinspires.ftc.teamcode.util.CameraPosition;

import java.util.ArrayList;
import java.util.Locale;

public abstract class AbstractAuto extends LinearOpMode {

    Alliance alliance;
    CameraPosition cameraPosition;
    public Robot robot;

    private BarcodeLocation teamElementLocation = BarcodeLocation.RIGHT;
    private ArrayList<Step> steps;
    private double currentRuntime;
    private boolean stopWasNotRequested;

    public BarcodeLocation getTeamElementLocation() {
        return teamElementLocation;
    }

    // Main method to run all the steps for autonomous
    @Override
    public void runOpMode() {
        // initialize robot
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        setAlliance();
        setCameraPosition();

        if (useCamera()) {
            robot = new Robot(hardwareMap, cameraPosition, alliance);

            robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getDown());
            robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getDown());

            while (robot.camera.getFrameCount() < 1) {
                idle();
            }
        } else {
            robot = new Robot(hardwareMap, alliance);

            robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getDown());
            robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getDown());
        }


        // set up into box
//        robot.actuators.setArmPivot(ARM_PIVOT_POSITION.almostDown);
//        sleep(1000 * (long) DEPOSIT1_ALMOST/2);
//        robot.actuators.setArmHopper(ARM_HOPPER_POSITION.almostDown);
//        sleep(1000 * (long) DEPOSIT1_ALMOST/2);
//        robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getUp());
//        robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getUp());
//        sleep(1000 * (long) DEPOSIT2_ARM);

        robot.actuators.setIntakeServo(INTAKE_SERVO_UP);
//
//        double time = getRuntime();
//        while (getRuntime() < time + 2) {
//            robot.actuators.resetIntake();
//        }

//        robot.actuators.setIntakeVerticalPositionInAuto((int) robot.actuators.getIntakePosition() + 72);

//        makeTrajectories();

        makeTrajectories();

        // wait for start
        while (!(isStarted() || isStopRequested())) {
//            teamElementLocation = robot.camera.checkTeamElementLocationUsingAprilTags();
            robot.updateLights();
            if (useCamera()) {
                teamElementLocation = robot.camera.checkTeamElementLocation();
                telemetry.addLine("Initialized");
                telemetry.addLine(String.format(Locale.US, "Location: %s", teamElementLocation));
                telemetry.addLine(String.format(Locale.US, "Size: %.4f", robot.camera.getTeamElement().getArea()));
            } else {
                telemetry.addLine("Initialized");
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
                robot.actuators.resetIntake();
            }

            @Override
            public void end() {
                robot.actuators.setIntake(0);
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
                    case 0:
                        if (!robot.drive.isBusy()) { // if the robot is not already going into the warehouse, start moving
                            robot.drive.followTrajectoryAsync(trajectoryIn);
                        }
                        if (!robot.actuators.runningRetract) {
                            robot.actuators.setIntake(-INTAKE_SPEED);
                            stepCaseStep++;
                        }
                        break;
                    case 1:
                        if (robot.actuators.hopperIsFull()) {
                            robot.drive.breakFollowing();
                            stepStartTime = currentRuntime;
                            stepCaseStep++;
                        }
                        break;
                    case 2:
                        if (stepTime > 0.3) {
                            robot.drive.followTrajectoryAsync(trajectoryOut);
                            robot.actuators.setIntake(0);
                            stepStartTime = currentRuntime;
                            stepCaseStep++;
                        }
                        break;
                    case 3:
                        if (stepTime > 0.2) {
                            robot.actuators.setIntakePosition((int) (robot.actuators.getIntakePosition() - (robot.actuators.getIntakePosition() % 145.1)));
                            stepCaseStep++;
                        }
                        break;
                    case 4:
                        robot.actuators.resetIntake();
                        if (robot.actuators.intakeIsReset()) {
                            robot.actuators.setIntake(0);
                            robot.actuators.runningExtend = true;
                            stepCaseStep++;
                        }
                        break;
                    case 5:
                        if (!robot.drive.isBusy() && !robot.actuators.runningExtend) {
                            robot.actuators.runningRetract = true;
                            stepCaseStep++;
                        }
                        break;
                    case 6:
                        if (robot.actuators.getState() >= 10) { // if the retract macro is almost done, start cycling again
                            stepCaseStep++;
                        }
//                        if (!robot.actuators.runningRetract) {
//                            stepCaseStep++;
//                        }
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
                return stepCaseStep == 7;
            }
        });
    }
    public void cycleBlockInAuto2(double timeout, Trajectory trajectoryIn, Trajectory trajectoryOut, Trajectory creep, Alliance alliance, BarcodeLocation barcodeLocation) {
        steps.add(new Step("Scoring Alliance Hub ", timeout) {
            @Override
            public void start() {
                stepStartTime = currentRuntime;
                stepCaseStep = 0;
            }

            @Override
            public void whileRunning() {
                stepTime = currentRuntime - stepStartTime;
                switch (stepCaseStep) {
                    case 0: //if the hopper is empty, creep and spin intake
                        if(!robot.actuators.hopperIsFull()) { //if the hopper is empty
                            robot.actuators.setIntake(-INTAKE_SPEED / 2); //run the intake forward slow
                            robot.drive.followTrajectoryAsync(creep); //and move to pickup blocks
                        }
                        stepCaseStep++;
                        break;
                    case 1: //wait until the robot has a block or the creep trajectory is done
                        if(robot.actuators.hopperIsFull()){//if we have a block, leave
                            robot.drive.followTrajectoryAsync(trajectoryOut);
                            robot.actuators.setIntake(0); // and try to prepare to reset the intake
                            stepCaseStep++;
                        }else if(!robot.actuators.hopperIsFull() && !robot.drive.isBusy()){//if we have done the creep and got no block, then:
                            //for now just move on. try a second creep in future iterations
                            stepCaseStep++;
                        }
                        break;
                    case 2: // as we leave the warehouse, reset the intake
                        robot.actuators.setIntakePosition( (int) (robot.actuators.getIntakePosition() - (robot.actuators.getIntakePosition() % 145.1)));
                        stepCaseStep++;
                        break;
                    case 3:
                        robot.actuators.resetIntake();//update the intake PID each loop
                        if(robot.actuators.intakeIsReset()){ //once the intake is reset,
                            robot.actuators.runningAlliance = true; //run the score macro
                            stepCaseStep++; //and move on to the next step
                        }
                        break;
                    case 4:
                        if (!robot.drive.isBusy() && robot.actuators.justFinishedAllianceMacro) { // if we are into the scoring location and the macro is ready:
                            robot.actuators.runningDeposit=true; //run the deposit macro
                            stepCaseStep++; //and move to the next step
                        }
                        break;
                    case 5:
                        if(!robot.actuators.hopperIsFull()){ //if the block has fallen out of the hopper
                            robot.drive.followTrajectoryAsync(trajectoryIn); //go into the warehouse
                            stepCaseStep++; // move to the next step
                        }
                        break;
                    case 6:
                        if (!robot.actuators.runningDeposit) { //if the deposit macro is over
                            stepCaseStep=-1; //end the state machine because we are ready to do another cycle
                        }
                        break;
                }//end of switch

                //at the end of each and every run through the loop:
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
                if (stepCaseStep == -1) {
                    return true;
                } else {
                    return false;
                }
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
                robot.actuators.setIntake(intakePower);
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
                robot.actuators.setDuckies(duckPower, alliance);
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

    public void addTrajectory(double timeout, Alliance alliance, Trajectory path) {
        steps.add(new Step("following trajectory", timeout) {
            @Override
            public void start() {
                robot.drive.followTrajectory(path);
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

    public void addAlliance(double timeout, Alliance alliance, BarcodeLocation barcodeLocation) {
        steps.add(new Step("Scoring Alliance Hub ", timeout) {
            @Override
            public void start() {
                robot.actuators.runningExtend = true;
            }

            @Override
            public void whileRunning() {
                robot.actuators.runningAlliance(getRuntime(), alliance, barcodeLocation);
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

    public void addShared(double timeout, Alliance alliance, BarcodeLocation barcodeLocation) {
        steps.add(new Step("Scoring Shared Hub ", timeout) {
            @Override
            public void start() {
                robot.actuators.runningShared = true;
            }

            @Override
            public void whileRunning() {
                robot.actuators.runningShared(getRuntime(), alliance, barcodeLocation);
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return !robot.actuators.runningShared;
            }
        });
    }

    public void addDeposit(double timeout, Alliance alliance, BarcodeLocation barcodeLocation) {
        steps.add(new Step("Depositing", timeout) {
            @Override
            public void start() {
                robot.actuators.runningRetract = true;
            }

            @Override
            public void whileRunning() {
                robot.actuators.runningDeposit(getRuntime(), alliance, barcodeLocation);
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

    public void addArm(double timeout) {
        steps.add(new Step("Depositing", timeout) {
            @Override
            public void start() {
                robot.actuators.runningArm = true;
            }

            @Override
            public void whileRunning() {
                robot.actuators.runningArm(getRuntime());
            }

            @Override
            public void end() {
            }

            @Override
            public boolean isFinished() {
                return !robot.actuators.runningArm;
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
}