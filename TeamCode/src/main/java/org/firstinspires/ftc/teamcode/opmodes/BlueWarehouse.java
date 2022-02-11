package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.BarcodeLocation;

@Autonomous(name = "Blue Warehouse", group = "Competition", preselectTeleOp = "Blue TeleOp")
public class BlueWarehouse extends AbstractAuto {
    public static Pose2d START_POSE = new Pose2d(12, 63, Math.toRadians(0));

    @Override
    public void setAlliance() {
        this.alliance = BLUE;
    }

    @Override
    public void initializeSteps(BarcodeLocation location) {
        robot.drive.setPoseEstimate(START_POSE);

        Trajectory forward = robot.drive.trajectoryBuilder(START_POSE)
                .forward(28)
                .build();
        Trajectory back = robot.drive.trajectoryBuilder(forward.end())
                .back(28)
                .build();

        // score preloaded
        addAlliance(10000, alliance, RIGHT);
        addDeposit(10000, alliance, RIGHT);

        // 1 block
        addIntake(0, -1);
        followTrajectory(forward);
        followTrajectory(back);
        resetIntake(2);
        addAlliance(10000, alliance, RIGHT);
        addDeposit(10000, alliance, RIGHT);

        // 2 block
        addIntake(0, -1);
        followTrajectory(forward);
        followTrajectory(back);
        resetIntake(2);
        addAlliance(10000, alliance, RIGHT);
        addDeposit(10000, alliance, RIGHT);

        // park
        followTrajectory(forward);
    }
}