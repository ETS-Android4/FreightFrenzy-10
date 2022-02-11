package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.BarcodeLocation;

@Autonomous(name = "Red Warehouse", group = "Competition", preselectTeleOp = "Red TeleOp")
public class RedWarehouse extends AbstractAuto {
    public static Pose2d START_POSE = new Pose2d(12, -63, Math.toRadians(0));
    public static Pose2d INTAKE1 = new Pose2d(40, -65, Math.toRadians(0));
    public static Pose2d SCORE1 = new Pose2d(12, -67, Math.toRadians(0));
    public static Pose2d INTAKE2 = new Pose2d(40, -69, Math.toRadians(0));
    public static Pose2d SCORE2 = new Pose2d(12, -71, Math.toRadians(0));
//    public static Pose2d INTAKE3 = new Pose2d(40, -69, Math.toRadians(0));
//    public static Pose2d SCORE3 = new Pose2d(40, -69, Math.toRadians(0));
    public static Pose2d PARK = new Pose2d(40, -73, Math.toRadians(0));

    @Override
    public void setAlliance() {
        this.alliance = RED;
    }

    @Override
    public void initializeSteps(BarcodeLocation location) {
        robot.drive.setPoseEstimate(START_POSE);

        Trajectory intake1 = robot.drive.trajectoryBuilder(START_POSE)
                .lineToLinearHeading(INTAKE1)
                .build();
        Trajectory score1 = robot.drive.trajectoryBuilder(intake1.end())
                .lineToLinearHeading(SCORE1)
                .build();

        Trajectory intake2 = robot.drive.trajectoryBuilder(score1.end())
                .lineToLinearHeading(INTAKE2)
                .build();
        Trajectory score2 = robot.drive.trajectoryBuilder(intake2.end())
                .lineToLinearHeading(SCORE2)
                .build();

//        Trajectory intake3 = robot.drive.trajectoryBuilder(score2.end())
//                .lineToLinearHeading(INTAKE3)
//                .build();
//        Trajectory score3 = robot.drive.trajectoryBuilder(intake3.end())
//                .lineToLinearHeading(SCORE3)
//                .build();

        Trajectory park = robot.drive.trajectoryBuilder(score2.end())
                .lineToLinearHeading(PARK)
                .build();

        // score preloaded
        addAlliance(10000, alliance, RIGHT);
        addDeposit(10000, alliance, RIGHT);

        // 1 block
        addIntake(0, -1);
        followTrajectory(intake1);
        followTrajectory(score1);
        resetIntake(2);
        addAlliance(10000, alliance, RIGHT);
        addDeposit(10000, alliance, RIGHT);

        // 2 block
        addIntake(0, -1);
        followTrajectory(intake2);
        followTrajectory(score2);
        resetIntake(2);
        addAlliance(10000, alliance, RIGHT);
        addDeposit(10000, alliance, RIGHT);

        // park
        followTrajectory(park);
    }
}