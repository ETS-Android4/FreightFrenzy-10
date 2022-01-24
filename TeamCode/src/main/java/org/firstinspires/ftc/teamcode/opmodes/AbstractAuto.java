package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.oldutil.Alliance;
import org.firstinspires.ftc.teamcode.oldutil.BarcodeLocation;

import java.util.ArrayList;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.oldutil.Alliance.NEITHER;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.AUTO_CUTOFF;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.AUTO_MIN;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.AUTO_P;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.HOPPER_INIT;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.HOPPER_MID;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.HOPPER_START;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.SLIDE_CUTOFF;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.SLIDE_SPEED;
import static org.firstinspires.ftc.teamcode.oldutil.MathUtil.about;
import static org.firstinspires.ftc.teamcode.oldutil.MathUtil.clamp;

public abstract class AbstractAuto extends LinearOpMode {
    Alliance alliance;
    public Robot robot;
    private BarcodeLocation teamElementLocation;
    private ArrayList<Step> steps;
    private double currentRuntime;
    private boolean stopWasNotRequested;

    public void setAlliance() {
        this.alliance = NEITHER;
    }

    // Main method to run all the steps for autonomous
    @Override
    public void runOpMode() {
        // initialize robot
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        setAlliance();

        robot = new Robot(hardwareMap);

        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slide.setTargetPosition(0);
        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.setPower(SLIDE_SPEED);

        robot.hopper.setPosition(HOPPER_INIT);

        robot.camera.initBarcodeWebcam();
        while (robot.camera.getFrameCount() < 1) {
            idle();
        }

        // wait for start
        while (!(isStarted() || isStopRequested())) {
            teamElementLocation = robot.camera.checkTeamElementLocation();
//            teamElementLocation = robot.camera.checkTeamElementLocationUsingAprilTags();
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
                robot.intake.setPower(intakePower);
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
    public void addHopper(double timeout, final double hopperPos) {
        steps.add(new Step("Setting hopper to " + hopperPos, timeout) {
            @Override
            public void start() {
                robot.hopper.setPosition(hopperPos);
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
    public void addSlide(final int slidePos) {
        steps.add(new Step("Setting slide to " + slidePos) {
            @Override
            public void start() {
                robot.slide.setTargetPosition(slidePos);
            }
            @Override
            public void whileRunning() {
                if (robot.slide.getCurrentPosition() > SLIDE_CUTOFF) {
                    robot.hopper.setPosition(HOPPER_START.l);
                } else if (robot.hopper.getPosition() == HOPPER_START.l) {
                    robot.hopper.setPosition(HOPPER_MID.l);
                }
            }
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return about(robot.slide.getCurrentPosition(), slidePos);
            }
        });
    }
    public void addDuckSpinner(double timeout, final double duckPower) {
        steps.add(new Step("Setting duck power to " + duckPower, timeout) {
            @Override
            public void start() {
                robot.ducky.setPower(duckPower);
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
}