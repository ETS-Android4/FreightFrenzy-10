package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.BarcodeLocation;
import org.firstinspires.ftc.teamcode.util.CameraPosition;

@Autonomous(name = "Blue Warehouse", group = "Competition", preselectTeleOp = "Blue TeleOp")
public class BlueWarehouse extends AbstractAuto {

    //define the waypoints in this auto
    public static Pose2d START_POSE = new Pose2d(10.6875, 65.75, Math.toRadians(0));
    public static Pose2d INTAKE1 = new Pose2d(36, 66.25, Math.toRadians(0));
    public static Pose2d INTAKE2 = new Pose2d(38, 66.25, Math.toRadians(0));
    public static Pose2d INTAKE3 = new Pose2d(40, 66.25, Math.toRadians(0));
    public static Pose2d INTAKE4 = new Pose2d(42, 66.25, Math.toRadians(0));
    public static Pose2d INTAKE5 = new Pose2d(44, 66.25, Math.toRadians(0));
    public static Pose2d CREEP = new Pose2d(52, 66.25, Math.toRadians(0));
    public static Pose2d SCORE = new Pose2d(10.6875, 66.25, Math.toRadians(0));
    public static Pose2d PARK = new Pose2d(41, 66.25, Math.toRadians(0));

    public int intake_speed = 35;
    public int creep_speed = 5;

    Trajectory intake1;
    Trajectory intake2;
    Trajectory intake3;
    Trajectory intake4;
    Trajectory intake5;
    Trajectory score1;
    Trajectory score2;
    Trajectory score3;
    Trajectory score4;
    Trajectory score5;
    Trajectory park;

    @Override
    public void setAlliance() {
        this.alliance = BLUE;
    }

    @Override
    public void setCameraPosition() {
        this.cameraPosition = CameraPosition.RIGHT;
    }

    @Override
    public boolean useCamera() {
        return true;
    }

    @Override //build the trajectories for this auto
    public void makeTrajectories() {
        robot.drive.setPoseEstimate(START_POSE);

        intake1 = robot.drive.trajectoryBuilder(START_POSE)
                .splineToConstantHeading(new Vector2d(INTAKE1.getX(), INTAKE1.getY()), INTAKE1.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(intake_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(CREEP.getX(), CREEP.getY()), CREEP.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(creep_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        intake2 = robot.drive.trajectoryBuilder(START_POSE)
                .splineToConstantHeading(new Vector2d(INTAKE2.getX(), INTAKE2.getY()), INTAKE2.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(intake_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(CREEP.getX(), CREEP.getY()), CREEP.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(creep_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        intake3 = robot.drive.trajectoryBuilder(START_POSE)
                .splineToConstantHeading(new Vector2d(INTAKE3.getX(), INTAKE3.getY()), INTAKE3.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(intake_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(CREEP.getX(), CREEP.getY()), CREEP.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(creep_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        intake3 = robot.drive.trajectoryBuilder(START_POSE)
                .splineToConstantHeading(new Vector2d(INTAKE3.getX(), INTAKE3.getY()), INTAKE3.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(intake_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(CREEP.getX(), CREEP.getY()), CREEP.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(creep_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        intake4 = robot.drive.trajectoryBuilder(START_POSE)
                .splineToConstantHeading(new Vector2d(INTAKE4.getX(), INTAKE4.getY()), INTAKE4.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(intake_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(CREEP.getX(), CREEP.getY()), CREEP.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(creep_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        intake5 = robot.drive.trajectoryBuilder(START_POSE)
                .splineToConstantHeading(new Vector2d(INTAKE5.getX(), INTAKE5.getY()), INTAKE5.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(intake_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(CREEP.getX(), CREEP.getY()), CREEP.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(creep_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        score1 = robot.drive.trajectoryBuilder(INTAKE1)
                .lineToLinearHeading(SCORE)
                .build();
        score2 = robot.drive.trajectoryBuilder(INTAKE2)
                .lineToLinearHeading(SCORE)
                .build();
        score3 = robot.drive.trajectoryBuilder(INTAKE3)
                .lineToLinearHeading(SCORE)
                .build();
        score4 = robot.drive.trajectoryBuilder(INTAKE4)
                .lineToLinearHeading(SCORE)
                .build();
        score5 = robot.drive.trajectoryBuilder(INTAKE5)
                .lineToLinearHeading(SCORE)
                .build();

        park = robot.drive.trajectoryBuilder(score5.end())
                .lineToLinearHeading(PARK)
                .build();
    }

    @Override //setup the specific actions in order for this auto
    public void initializeSteps(BarcodeLocation location) {
        scorePreloadInAuto(1000, alliance, location, true);
        cycleBlockInAuto(1000, intake1, score1, alliance, RIGHT);
        cycleBlockInAuto(1000, intake2, score2, alliance, RIGHT);
        cycleBlockInAuto(1000, intake3, score3, alliance, RIGHT);
        cycleBlockInAuto(1000, intake4, score4, alliance, RIGHT);
//        cycleBlockInAuto(1000, intake5, score5, alliance, RIGHT);
        parkInAuto(1000, alliance, park, RIGHT);
    }
}
