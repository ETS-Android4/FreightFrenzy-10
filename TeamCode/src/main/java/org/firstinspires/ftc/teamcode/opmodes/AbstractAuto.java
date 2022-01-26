//package org.firstinspires.ftc.teamcode.opmodes;
//
//import static org.firstinspires.ftc.teamcode.util.Alliance.NEITHER;
//import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
//
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.hardware.Robot;
//import org.firstinspires.ftc.teamcode.util.Alliance;
//import org.firstinspires.ftc.teamcode.util.BarcodeLocation;
//
//import java.util.ArrayList;
//import java.util.Locale;
//
//public abstract class AbstractAuto extends LinearOpMode {
//    Alliance alliance;
//    public Robot robot;
//    private BarcodeLocation teamElementLocation;
//    private ArrayList<Step> steps;
//    private double currentRuntime;
//    private boolean stopWasNotRequested;
//
//    public void setAlliance() {
//        this.alliance = RED;
//    }
//
//    // Main method to run all the steps for autonomous
//    @Override
//    public void runOpMode() {
//        // initialize robot
//        telemetry.addLine("Initializing Robot...");
//        telemetry.update();
//
//        setAlliance();
//
//        robot = new Robot(hardwareMap);
//
//        // wait for start
//        while (!(isStarted() || isStopRequested())) {
////            teamElementLocation = robot.camera.checkTeamElementLocationUsingAprilTags();
//            telemetry.addLine("Initialized");
////            telemetry.addLine(String.format(Locale.US, "Location: %s", teamElementLocation));
////            telemetry.addLine(String.format(Locale.US, "Size: %.4f", robot.camera.getTeamElement().getArea()));
//            telemetry.update();
//        }
//        resetStartTime();
//
//        // start up the first step
//        steps = new ArrayList<>();
//        initializeSteps(teamElementLocation);
//
//        int stepNumber = 0;
//        double stepTimeout;
//        Step step = steps.get(stepNumber);
//        stepTimeout = step.getTimeout() != -1 ? currentRuntime + step.getTimeout() : Double.MAX_VALUE;
//        step.start();
//
//        // run the remaining steps
//        while(opModeIsActive()) {
//            currentRuntime = getRuntime();
//            // once a step finishes
//            if (step.isFinished() || currentRuntime >= stepTimeout) {
//                // do the finishing move
//                step.end();
//                stepNumber++;
//                // if it was the last step break out of the while loop
//                if (stepNumber > steps.size() - 1) {
//                    break;
//                }
//                // else continue to the next step
//                step = steps.get(stepNumber);
//                stepTimeout = step.getTimeout() != -1 ? currentRuntime + step.getTimeout() : Double.MAX_VALUE;
//                step.start();
//            }
//
//            // while the step is running display telemetry
//            step.whileRunning();
//            telemetry.addLine(String.format(Locale.US, "Runtime: %.0f", currentRuntime));
//            telemetry.addLine("Step "+(stepNumber+1)+" of "+steps.size()+", "+step.getTelemetry()+"\n");
//            telemetry.addLine(robot.getTelemetry());
//            telemetry.update();
//        }
//}
//
//    // Load up all of the steps for the autonomous
//    public void initializeSteps(BarcodeLocation location) {
//        addDelay(5);
//    }
//
//    // Functions to add steps
//    public void addDelay(double timeout) {
//        steps.add(new Step("Waiting for "+timeout+" seconds", timeout) {
//            @Override
//            public void start() {}
//            @Override
//            public void whileRunning() {}
//            @Override
//            public void end() {}
//            @Override
//            public boolean isFinished() {
//                return false;
//            }
//        });
//    }
//    public void turn(double degrees) {
//        steps.add(new Step("Following a trajectory") {
//            @Override
//            public void start() {
//                robot.drive.turn(degrees);
//            }
//            @Override
//            public void whileRunning() {
//                robot.drive.update();
//            }
//            @Override
//            public void end() {}
//            @Override
//            public boolean isFinished() {
//                return !robot.drive.isBusy();
//            }
//        });
//    }
//
//    public void followTrajectory(Trajectory trajectory) {
//        steps.add(new Step("Following a trajectory") {
//            @Override
//            public void start() {
//                robot.drive.followTrajectoryAsync(trajectory);
//            }
//            @Override
//            public void whileRunning() {
//                robot.drive.update();
//            }
//            @Override
//            public void end() {}
//            @Override
//            public boolean isFinished() {
//                return !robot.drive.isBusy();
//            }
//        });
//    }
//
//    public void addIntake(double timeout, final double intakePower) {
//        steps.add(new Step("Setting intake power to " + intakePower, timeout) {
//            @Override
//            public void start() {
//                robot.actuators.setIntake(intakePower);
//            }
//            @Override
//            public void whileRunning() {}
//            @Override
//            public void end() {
//                robot.actuators.resetIntake();
//            }
//            @Override
//            public boolean isFinished() {
//                return false;
//            }
//        });
//    }
//    public void addHopperServo(double timeout, final double hopperServoPos) {
//        steps.add(new Step("Setting hopper to " + hopperServoPos, timeout) {
//            @Override
//            public void start() {
//                robot.actuators.setHopperServo(hopperServoPos);
//            }
//            @Override
//            public void whileRunning() {}
//            @Override
//            public void end() {}
//            @Override
//            public boolean isFinished() {
//                return false;
//            }
//        });
//    }
//    public void addSlidesServo(double timeout, final double slidesServoPos) {
//        steps.add(new Step("Setting hopper to " + slidesServoPos, timeout) {
//            @Override
//            public void start() {
//                robot.actuators.setHopperServo(slidesServoPos);
//            }
//            @Override
//            public void whileRunning() {}
//            @Override
//            public void end() {}
//            @Override
//            public boolean isFinished() {
//                return false;
//            }
//        });
//    }
//    public void addTurret(double timeout, final int turretPos) {
//        steps.add(new Step("Setting turret to " + turretPos, timeout) {
//            @Override
//            public void start() {
//                robot.actuators.setTurret(turretPos);
//            }
//            @Override
//            public void whileRunning() {}
//            @Override
//            public void end() {}
//            @Override
//            public boolean isFinished() {
//                return false;
//            }
//        });
//    }
//
//    public void addSlides(double timeout, final int slidePos) {
//        steps.add(new Step("Setting slide to " + slidePos, timeout) {
//            @Override
//            public void start() {
//                robot.actuators.setSlides(slidePos);
//            }
//            @Override
//            public void whileRunning() {}
//            @Override
//            public void end() {}
//            @Override
//            public boolean isFinished() {
//                return false;
//            }
//        });
//    }
//    public void addDuckSpinner(double timeout, final double duckPower) {
//        steps.add(new Step("Setting duck power to " + duckPower, timeout) {
//            @Override
//            public void start() {
//                robot.actuators.setDuckies(duckPower, alliance);
//            }
//            @Override
//            public void whileRunning() {}
//            @Override
//            public void end() {}
//            @Override
//            public boolean isFinished() {
//                return false;
//            }
//        });
//    }
//    public void stopTargetingCamera() {
//        steps.add(new Step("Stopping Targeting Camera") {
//            @Override
//            public void start() {
//                robot.camera.stopBarcodeWebcam();
//            }
//
//            @Override
//            public void whileRunning() {
//            }
//
//            @Override
//            public void end() {
//            }
//
//            @Override
//            public boolean isFinished() {
//                return robot.camera.getFrameCount() == 0;
//            }
//        });
//    }
//    public void addAlliance(double timeout, Alliance alliance) {
//        steps.add(new Step("Scoring Alliance Hub ", timeout) {
//            @Override
//            public void start() {
//                robot.actuators.setDuckies(duckPower, alliance);
//            }
//            @Override
//            public void whileRunning() {}
//            @Override
//            public void end() {}
//            @Override
//            public boolean isFinished() {
//                return false;
//            }
//        });
//    }
//
//
//}