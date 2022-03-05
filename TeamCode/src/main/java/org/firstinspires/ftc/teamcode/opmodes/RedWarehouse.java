package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_RESET_TIME;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_SPEED;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.LEFT;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.MIDDLE;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.BarcodeLocation;
import org.firstinspires.ftc.teamcode.util.CameraPosition;

import java.util.Arrays;

@Autonomous(name = "Red Warehouse", group = "Competition", preselectTeleOp = "Red TeleOp")
public class RedWarehouse extends AbstractAuto {

    //define the waypoints in this auto
    public static Pose2d START_POSE = new Pose2d(12, -63, Math.toRadians(0));
    public static Pose2d INTAKE = new Pose2d(36, -64, Math.toRadians(0));
    public static Pose2d CREEP = new Pose2d(56, -64, Math.toRadians(0));
    public static Pose2d SCORE = new Pose2d(12, -64, Math.toRadians(0));
    public static Pose2d PARK = new Pose2d(40, -64, Math.toRadians(0));

    Trajectory intake;
    Trajectory score;
    Trajectory park;

    @Override
    public void setAlliance() {
        this.alliance = RED;
    }

    @Override
    public void setCameraPosition() {
        this.cameraPosition = CameraPosition.LEFT;
    }

    @Override
    public boolean useCamera() {
        return true;
    }

    @Override //build the trajectories for this auto
    public void makeTrajectories() {
        robot.drive.setPoseEstimate(START_POSE);

        intake = robot.drive.trajectoryBuilder(START_POSE)
                .splineToConstantHeading(new Vector2d(INTAKE.getX(), INTAKE.getY()), INTAKE.getHeading())
                .splineToConstantHeading(new Vector2d(CREEP.getX(), CREEP.getY()), CREEP.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        score = robot.drive.trajectoryBuilder(new Pose2d(42, -64, Math.toRadians(0)))
                .lineToLinearHeading(SCORE)
                .build();
        park = robot.drive.trajectoryBuilder(score.end())
                .lineToLinearHeading(PARK)
                .build();
    }

    @Override //setup the specific actions in order for this auto
    public void initializeSteps(BarcodeLocation location) {
        // set arm
        addArmPivot(0, ARM_PIVOT_POSITION.getDown());
        addArmHopper(0, ARM_HOPPER_POSITION.getDown());
        addIntakeServo(0.25, INTAKE_SERVO_DOWN);
        resetIntake(INTAKE_RESET_TIME);

        // score preloaded
        addAlliance(10000, alliance, location);
        addDeposit(10000, alliance, location);

//        testing
//        addDelay(3);
//
//        addAlliance(10000, alliance, MIDDLE);
//        addDeposit(10000, alliance, MIDDLE);
//
//        addDelay(3);
//
//        addAlliance(10000, alliance, MIDDLE);
//        addDeposit(10000, alliance, MIDDLE);

//         auto cycle
        cycleBlockInAuto(1000, intake, score, alliance, RIGHT);
        cycleBlockInAuto(1000, intake, score, alliance, RIGHT);
        cycleBlockInAuto(1000, intake, score, alliance, RIGHT);

        //park
        addTrajectory(1000, alliance, park);
    }
}
