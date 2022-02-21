package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_UP;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
    private BarcodeLocation teamElementLocation;
    private ArrayList<Step> steps;
    private double currentRuntime;
    private boolean stopWasNotRequested;

    public abstract void setAlliance();

    public abstract void setCameraPosition();

    // Main method to run all the steps for autonomous
    @Override
    public void runOpMode() {
        // initialize robot
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        setAlliance();
        setCameraPosition();

        robot = new Robot(hardwareMap, cameraPosition);

        robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getDown());
        robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getDown());

        while (robot.camera.getFrameCount() < 1) {
            idle();
        }

        // set up into box
//        robot.actuators.setArmPivot(ARM_PIVOT_POSITION.almostDown);
//        sleep(1000 * (long) DEPOSIT1_ALMOST/2);
//        robot.actuators.setArmHopper(ARM_HOPPER_POSITION.almostDown);
//        sleep(1000 * (long) DEPOSIT1_ALMOST/2);
//        robot.actuators.setArmPivot(ARM_PIVOT_POSITION.getUp());
//        robot.actuators.setArmHopper(ARM_HOPPER_POSITION.getUp());
//        sleep(1000 * (long) DEPOSIT2_ARM);

//        robot.actuators.setIntakeServo(INTAKE_SERVO_UP);
//
//        double time = getRuntime();
//        while (getRuntime() < time + 2) {
//            robot.actuators.resetIntake();
//        }

        // wait for start
        while (!(isStarted() || isStopRequested())) {
//            teamElementLocation = robot.camera.checkTeamElementLocationUsingAprilTags();
            teamElementLocation = robot.camera.checkTeamElementLocation();
            telemetry.addLine("Initialized");
            telemetry.addLine(String.format(Locale.US, "Location: %s", teamElementLocation));
            telemetry.addLine(String.format(Locale.US, "Size: %.4f", robot.camera.getTeamElement().getArea()));
            telemetry.update();
        }
        resetStartTime();

        // start up the first step
        steps = new ArrayList<>();
        initializeSteps(teamElementLocation);

        int stepNumber = 0;
        double stepTimeout;
        Step step = steps.get(stepNumber);
        stepTimeout = step.getTimeout() != -1 ? currentRuntime + step.getTimeout() : Double.MAX_VALUE;
        step.start();

        // run the remaining steps
        while(opModeIsActive()) {
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
            telemetry.addLine(String.format(Locale.US, "Runtime: %.0f", currentRuntime));
            telemetry.addLine("Step "+(stepNumber+1)+" of "+steps.size()+", "+step.getTelemetry()+"\n");
            telemetry.addLine(robot.getTelemetry());
            telemetry.update();
        }
}

    // Load up all of the steps for the autonomous
    public void initializeSteps(BarcodeLocation location) {
        addDelay(5);
    }

    // Functions to add steps
    public void addDelay(double timeout) {
        steps.add(new Step("Waiting for "+timeout+" seconds", timeout) {
            @Override
            public void start() {}
            @Override
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }
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
            public void end() {}
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
            public void end() {}
            @Override
            public boolean isFinished() {
                return !robot.drive.isBusy();
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
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }
    public void resetIntake(double timeout) {
        steps.add(new Step("Resetting Intake", timeout) {
            @Override
            public void start() {
//                robot.actuators.setIntake(0);
                int newPos = (int) (robot.actuators.getIntakePosition() + (145.1/8.0) - (robot.actuators.getIntakePosition() % (145.1)));
//                int newPos = (int) (robot.actuators.getIntakePosition() + 145.1*5 + (robot.actuators.getIntakePosition()  % (145.1)));
//                int newPos = (int) (robot.actuators.getIntakePosition() - (robot.actuators.getIntakePosition()  % (145.1)));
                robot.actuators.setIntakePosition(newPos);
            }
            @Override
            public void whileRunning() {
                robot.actuators.resetIntake();
            }
            @Override
            public void end() {
                robot.actuators.setIntake(0);
                robot.actuators.setIntakePosition(robot.actuators.getIntakePosition());
                robot.actuators.setIntake(0);
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
            public void whileRunning() {}
            @Override
            public void end() {}
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
            public void whileRunning() {}
            @Override
            public void end() {}
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
            public void whileRunning() {}
            @Override
            public void end() {}
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
            public void whileRunning() {}
            @Override
            public void end() {}
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
            public void whileRunning() {}
            @Override
            public void end() {}
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
                return robot.camera.getFrameCount() == 0;
            }
        });
    }
//    public void addIntakeReset(double timeout) {
//        steps.add(new Step("Resetting Intake", timeout) {
//            @Override
//            public void start() {
//                robot.actuators.pickingUpFreight = true;
//            }
//            @Override
//            public void whileRunning() {
//                robot.actuators.pickingUpFreight(getRuntime());
//            }
//            @Override
//            public void end() {}
//            @Override
//            public boolean isFinished() {
//                return !robot.actuators.pickingUpFreight;
//            }
//        });
//    }
    public void addAlliance(double timeout, Alliance alliance, BarcodeLocation barcodeLocation) {
        steps.add(new Step("Scoring Alliance Hub ", timeout) {
            @Override
            public void start() {
                robot.actuators.runningAlliance = true;
            }
            @Override
            public void whileRunning() {
                robot.actuators.runningAlliance(getRuntime(), alliance, barcodeLocation);
            }
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return !robot.actuators.runningAlliance;
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
            public void end() {}
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
                robot.actuators.runningDeposit = true;
            }
            @Override
            public void whileRunning() {
                robot.actuators.runningDeposit(getRuntime(), alliance, barcodeLocation);
            }
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return !robot.actuators.runningDeposit;
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
            public void end() {}
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
            public void whileRunning() {}
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }
}