package org.firstinspires.ftc.teamcode.opmodes.teleop;


import static org.firstinspires.ftc.teamcode.hardware.Actuators.SLIDES_SPEED;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.HIGH;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.DepositPosition;
import org.firstinspires.ftc.teamcode.util.controller.Controller;

@Config
public class AbstractTeleOp extends OpMode {


    public int state = 0;

    Alliance alliance;

    private Controller driver1;
    private Controller driver2;

    private Robot robot;

    private int turretPosition = 0;
    private int slidesPosition = 0;

    public DepositPosition extendTo = HIGH;
    public DepositPosition extendFrom = HIGH;

    public void setAlliance() {
        alliance = RED;
    }

    Trajectory pathToScore;

    @Override
    public void init() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        setAlliance();

        driver1 = new Controller(gamepad1);
        driver2 = new Controller(gamepad2);

        robot = new Robot(hardwareMap, alliance);
        //robot.lights.setPattern(alliance == RED ? REDINIT : BLUEINIT);

        // set the pose from auto
        if (PoseStorage.POSE_IS_DEFAULT) {
            PoseStorage.intakeOffset = 0;
            PoseStorage.slidesOffset = 0;
            PoseStorage.turretOffset = 0;
            PoseStorage.CURRENT_POSE = alliance == RED ? PoseStorage.START_RED : PoseStorage.START_BLUE;
        }
        robot.drive.setPoseEstimate(PoseStorage.CURRENT_POSE);
        PoseStorage.POSE_IS_DEFAULT = true;



        robot.actuators.intakeStartPos = 0;

        //robot.actuators.odoRetracted = false;
        //robot.actuators.setOdoServo(0.01);

//        turretPosition = robot.actuators.getTurret();
//        slidesPosition = robot.actuators.getSlides();
//        armHopperPosition = robot.actuators.getArmHopper();
//        armPivotPosition = robot.actuators.getArmPivot();
//        turretPosition = 0;
//        slidesPosition = 0;
//        armHopperPosition = ARM_HOPPER_POSITION.getDown();
//        armPivotPosition = ARM_PIVOT_POSITION.getDown();
    }

    @Override
    public void init_loop() {
//        robot.updateLights();
        telemetry.addLine(("Initialized: " + alliance + " alliance selected."));
        telemetry.update();
    }

    @Override
    public void loop() {
        //robot.updateLights();
        driver1.update();
        driver2.update();



        // drive base

        // normal driver stuff
        double x, y, z;

        //new stuff with "exponential" speed

        //get the initial values
        x = driver1.getLeftStick().getY();
        y = -driver1.getLeftStick().getX();
        z = -driver1.getRightStick().getX();

        //transform the linear controller output into the nonlinear curve
        x = 0.152 * Math.tan(1.42 * x); // blue desmos curve
        //y =  0.2*Math.tan(1.3734*y)  ;
        z = 0.152 * Math.tan(1.42 * z);

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

        robot.drive.update();

            if (slidesPosition > 500) {
                turretPosition += driver2.getLeftStick().getX() * 5;
            } else {
                turretPosition += driver2.getLeftStick().getX() * 15;
            }
            //move the slides
            slidesPosition += driver2.getRightStick().getY() * SLIDES_SPEED;

        // retractables
        if (driver1.getY().isJustPressed()) {
            robot.actuators.intakeRetracted = !robot.actuators.intakeRetracted;
        }
        if (!driver1.getStart().isPressed() && driver1.getA().isJustPressed()) {
            robot.actuators.odoRetracted = !robot.actuators.odoRetracted;
        }

//        robot.actuators.setIntakeServo(robot.actuators.intakeRetracted ? INTAKE_SERVO_UP : INTAKE_SERVO_DOWN);
//        robot.actuators.setOdoServo(robot.actuators.odoRetracted ? ODO_SERVO_UP : ODO_SERVO_DOWN);


        //robot.actuators.update();

        // telemetry
        telemetry.addLine("Alliance: " + alliance);
        telemetry.addLine(robot.getTelemetry());
        telemetry.update();
    }
}


