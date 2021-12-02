package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Actuators;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous(name = "Auto Blank", group = "Linear Opmode")
public class TestAuto extends LinearOpMode {
    Robot robot = new Robot(hardwareMap);

    public static Pose2d START_POSE = new Pose2d(-36, -54, Math.toRadians(90));

    @Override
    public void runOpMode() {
        robot.drive2.setPoseEstimate(START_POSE);

        Trajectory forward1 = robot.drive2.trajectoryBuilder(START_POSE)
//                .lineToLinearHeading(FORWARD)
                .forward(24)
                .build();
        Trajectory back = robot.drive2.trajectoryBuilder(forward1.end().plus(new Pose2d(0, 0, Math.toRadians(-45))))
//                .lineToLinearHeading(FORWARD)
                .back(48)
                .build();
        Trajectory forward2 = robot.drive2.trajectoryBuilder(back.end().plus(new Pose2d(0, 0, Math.toRadians(45))))
                .forward(12)
                .build();



        //wait for start

        robot.drive2.followTrajectory(forward1);
        robot.drive2.turn(Math.toRadians(-45);
        robot.getActuators().setLiftPosition(Actuators.Constants.LINEAR_SLIDE_TOP_POSITION);
        robot.drive2.followTrajectory(back);
        robot.drive2.turn(Math.toRadians(45);
        robot.drive2.followTrajectory(forward2);

    }
}
