package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BarcodeLocation;

import java.util.ArrayList;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.util.Alliance.NEITHER;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_INIT;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_SPEED;
import static org.firstinspires.ftc.teamcode.util.MathUtil.about;

public abstract class AbstractAuto extends LinearOpMode {
    Alliance alliance;
    private Robot robot;
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
            telemetry.addLine("Initialized");
            telemetry.addLine(String.format(Locale.US, "Location: %s", robot.camera.checkTeamElementLocation()));
            telemetry.addLine(String.format(Locale.US, "Size: %.4f", robot.camera.getTeamElement().getArea()));
            telemetry.update();
        }
        resetStartTime();

        // switch from stack camera to targeting camera
        robot.camera.stopBarcodeWebcam();


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
    public void addMovement(final double xMovement, final double yMovement, final double speed) {
        String status = "Moving ";
        if (Math.abs(xMovement) > 0) {
            status += xMovement > 0 ? xMovement + " right" : Math.abs(xMovement) + " left";
            if (Math.abs(yMovement) > 0) {
                status += " and ";
            }
        }
        if (Math.abs(yMovement) > 0) {
            status += yMovement > 0 ? yMovement + " forward" : Math.abs(yMovement) + " back";
        }
        steps.add(new Step(status) {
            @Override
            public void start() {
                this.x = xMovement;
                this.y = yMovement;
                this.power = speed;
                robot.drive.setTargetPositionRelative(x, y, power);
            }
            @Override
            public void whileRunning() {
//                if ((robot.drive.getTargetDistance() - robot.drive.getTargetDistanceRemaining() < 500 ||
//                        robot.drive.getTargetDistanceRemaining() < 1250 ) && this.power > 0.5) {
//                    robot.drive.setPower(0.5);
//                } else if (robot.drive.getTargetDistanceRemaining() > 500 && this.power > 0.5) {
//                    robot.drive.setPower(1);
//                }
            }
            @Override
            public void end() {}
            @Override
            public boolean isFinished() {
                return !robot.drive.isBusy() || robot.drive.getTargetDistanceRemaining() < 15;
            }
        });
    }
    public void addTurnAbsolute(final double degrees) {
        steps.add(new Step("Turning "+degrees+" degrees") {
            @Override
            public void start() {
                robot.sensors.resetGyroHeadingToInitial();
                destinationHeading = degrees;
                zRuntime = -1;
            }
            @Override
            public void whileRunning() {
                currentHeading = robot.sensors.getGyroHeading360();
                // determine the error (special case because the heading resets to 0 instead of 360)
                // the logic works, but in the future making it clearer and more efficient would be nice
                if (currentHeading > 180 && currentHeading > destinationHeading + 180) {
                    zErr = 360 - currentHeading + destinationHeading;
                } else {
                    zErr = (destinationHeading - currentHeading + 180) % 360 - 180;
                }

                // determine whether to turn or not
                if (Math.abs(zErr) <= 1) {
                    z = 0;
                    if (zRuntime == -1) {
                        zRuntime = currentRuntime;
                    }
                } else {
                    zRuntime = -1;
                    // set the speed proportionally to the error the robot is off by, with a minimum speed of 0.15
                    z = Math.copySign(Math.abs(zErr) > 45 ? 0.7 : 0.15, -zErr);
                }
                robot.drive.setWheels(0, 0, z);
            }
            @Override
            public void end() {
                robot.drive.setWheels(0, 0, 0);
            }
            @Override
            public boolean isFinished() {
                // if the robot is within a degree of the target position for more than 1 second
                return currentRuntime > zRuntime + 1 && zRuntime != -1;
            }
        });
    }
    public void addTurnAbsoluteFast(final double degrees) {
        steps.add(new Step("Turning "+degrees+" degrees") {
            @Override
            public void start() {
                robot.sensors.resetGyroHeadingToInitial();
                destinationHeading = degrees;
                zRuntime = -1;
            }
            @Override
            public void whileRunning() {
                currentHeading = robot.sensors.getGyroHeading360();
                // determine the error (special case because the heading resets to 0 instead of 360)
                // the logic works, but in the future making it clearer and more efficient would be nice
                if (currentHeading > 180 && currentHeading > destinationHeading + 180) {
                    zErr = 360 - currentHeading + destinationHeading;
                } else {
                    zErr = (destinationHeading - currentHeading + 180) % 360 - 180;
                }

                // determine whether to turn or not
                if (Math.abs(zErr) <= 3) {
                    z = 0;
                    if (zRuntime == -1) {
                        zRuntime = currentRuntime;
                    }
                } else {
                    zRuntime = -1;
                    // set the speed proportionally to the error the robot is off by, with a minimum speed of 0.15
                    z = Math.copySign(Math.abs(zErr) > 45 ? 0.7 : 0.15, -zErr);
                }
                robot.drive.setWheels(0, 0, z);
            }
            @Override
            public void end() {
                robot.drive.setWheels(0, 0, 0);
            }
            @Override
            public boolean isFinished() {
                // if the robot is within a degree of the target position for more than 1 second
                return z == 0;
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
            public void whileRunning() {}
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
}