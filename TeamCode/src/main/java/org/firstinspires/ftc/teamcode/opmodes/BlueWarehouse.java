package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.LEFT;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.MIDDLE;
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
    public static Pose2d START_POSE = new Pose2d(12, 63.5, Math.toRadians(0));
    public static Pose2d INTAKE = new Pose2d(36, 63.5, Math.toRadians(0));
    public static Pose2d CREEP = new Pose2d(56, 63.5, Math.toRadians(0));
    public static Pose2d SCORE = new Pose2d(12, 63.5, Math.toRadians(0));
    public static Pose2d PARK = new Pose2d(40, 63.5, Math.toRadians(0));

    Trajectory intake;
    Trajectory score;
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

        intake = robot.drive.trajectoryBuilder(START_POSE)
                .splineToConstantHeading(new Vector2d(INTAKE.getX(), INTAKE.getY()), INTAKE.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(CREEP.getX(), CREEP.getY()), CREEP.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        score = robot.drive.trajectoryBuilder(new Pose2d(42, 63.5, Math.toRadians(0)))
                .lineToLinearHeading(SCORE,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        park = robot.drive.trajectoryBuilder(score.end())
                .lineToLinearHeading(PARK,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
    }

    @Override //setup the specific actions in order for this auto
    public void initializeSteps(BarcodeLocation location) {
        scorePreloadInAuto(1000, alliance, location);
        cycleBlockInAuto(1000, intake, score, alliance, RIGHT);
        cycleBlockInAuto(1000, intake, score, alliance, RIGHT);
        cycleBlockInAuto(1000, intake, score, alliance, RIGHT);
        parkInAuto(1000, alliance, park, RIGHT);
    }
}
