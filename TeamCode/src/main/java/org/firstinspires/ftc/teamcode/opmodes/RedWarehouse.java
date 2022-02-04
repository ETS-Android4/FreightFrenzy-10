package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.BarcodeLocation;

@Autonomous(name = "Red Warehouse", group = "Competition", preselectTeleOp = "Red TeleOp")
public class RedWarehouse extends AbstractAuto {
    public static Pose2d START_POSE = new Pose2d(12, -63, Math.toRadians(0));

    @Override
    public void setAlliance() {
        this.alliance = RED;
    }

    @Override
    public void initializeSteps(BarcodeLocation location) {
        robot.drive.setPoseEstimate(START_POSE);

        Trajectory forward = robot.drive.trajectoryBuilder(START_POSE)
                .forward(36)
                .build();
        Trajectory back = robot.drive.trajectoryBuilder(forward.end())
                .back(36)
                .build();

        // score preloaded
        addAlliance(10, alliance);
//        addDelay(3);
        addDeposit(10, alliance);
        addDelay(2);

        // 1 block
        addIntake(0, -1);
        followTrajectory(forward);
//        addDelay(3);
        followTrajectory(back);
        resetIntake(3);
        addAlliance(10, alliance);
        addDeposit(10, alliance);

        // park
        followTrajectory(forward);
    }
}