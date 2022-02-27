package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_RESET_TIME;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.BarcodeLocation;
import org.firstinspires.ftc.teamcode.util.CameraPosition;

@Autonomous(name = "Red Warehouse", group = "Competition", preselectTeleOp = "Red TeleOp")
public class RedWarehouse extends AbstractAuto {
    public static Pose2d START_POSE = new Pose2d(12, -63, Math.toRadians(0));
    public static Pose2d INTAKE = new Pose2d(38, -63, Math.toRadians(0));
    public static Pose2d CREEP = new Pose2d(50, -63, Math.toRadians(0));
    public static Pose2d SCORE = new Pose2d(12, -63, Math.toRadians(0));
//    public static Pose2d INTAKE2 = new Pose2d(34, -63.5, Math.toRadians(0));
//    public static Pose2d SCORE2 = new Pose2d(12, -63.5, Math.toRadians(0));
//    public static Pose2d INTAKE3 = new Pose2d(34, -63.5, Math.toRadians(0));
//    public static Pose2d SCORE3 = new Pose2d(12, -63.5, Math.toRadians(0));
    public static Pose2d PARK = new Pose2d(42, -63, Math.toRadians(0));

    @Override
    public void setAlliance() {
        this.alliance = RED;
    }

    @Override
    public void setCameraPosition() {
        this.cameraPosition = CameraPosition.LEFT;
    }

    @Override
    public void initializeSteps(BarcodeLocation location) {
        robot.drive.setPoseEstimate(START_POSE);

        Trajectory intake = robot.drive.trajectoryBuilder(START_POSE)
                .lineToLinearHeading(INTAKE)
                .build();

        Trajectory creep = robot.drive.trajectoryBuilder(intake.end())
                .lineToLinearHeading(CREEP,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory score = robot.drive.trajectoryBuilder(creep.end())
                .lineToLinearHeading(SCORE)
                .build();


//        Trajectory intake2 = robot.drive.trajectoryBuilder(score1.end())
//                .lineToLinearHeading(INTAKE2)
//                .build();
//        Trajectory score2 = robot.drive.trajectoryBuilder(intake2.end())
//                .lineToLinearHeading(SCORE2)
//                .build();
//
//        Trajectory intake3 = robot.drive.trajectoryBuilder(score2.end())
//                .lineToLinearHeading(INTAKE3)
//                .build();
//        Trajectory score3 = robot.drive.trajectoryBuilder(intake3.end())
//                .lineToLinearHeading(SCORE3)
//                .build();

        Trajectory park = robot.drive.trajectoryBuilder(score.end())
                .lineToLinearHeading(PARK)
                .build();

        stopTargetingCamera();

        // set arm
        addArmPivot(0, ARM_PIVOT_POSITION.getDown());
        addArmHopper(0, ARM_HOPPER_POSITION.getDown());
        addIntakeServo(0.5, INTAKE_SERVO_DOWN);
        resetIntake(INTAKE_RESET_TIME);

        // score preloaded
//        addAlliance(10000, alliance, RIGHT);
//        addDeposit(10000, alliance, RIGHT);

        // cycle
        for (int i = 0; i < 10; i++) {
            cycleBlockInAuto(1000, intake, score, creep, alliance, RIGHT);
        }

//        // 1 block
//        addIntake(0, -INTAKE_SPEED);
//        followTrajectory(intake1);
//        addIntake(1, INTAKE_SPEED);
//
//        followTrajectory(score1);
//        addIntake(STOP_TIME, 0);
//        resetIntake(RESET_TIME);
//        addAlliance(10000, alliance, RIGHT);
//        addDeposit(10000, alliance, RIGHT);
//
//        // 2 block
//        addIntake(0, -INTAKE_SPEED);
//        followTrajectory(intake2);
//        addIntake(0, INTAKE_SPEED);
//        followTrajectory(score2);
//        addIntake(STOP_TIME, 0);
//        resetIntake(RESET_TIME);
//        addAlliance(10000, alliance, RIGHT);
//        addDeposit(10000, alliance, RIGHT);
//
//        // 3 block
//        addIntake(0, -INTAKE_SPEED);
//        followTrajectory(intake3);
//        addIntake(0, INTAKE_SPEED);
//        followTrajectory(score3);
//        addIntake(STOP_TIME, 0);
//        resetIntake(RESET_TIME);
//        addAlliance(10000, alliance, RIGHT);
//        addDeposit(10000, alliance, RIGHT);

//        for (int i = 0; i < 10; i++) {
//            addIntake(0, -INTAKE_SPEED);
//            followTrajectory(intake1);
//            addIntake(0, INTAKE_SPEED);
//            followTrajectory(score1);
//            addIntake(STOP_TIME, 0);
//            resetIntake(RESET_TIME);
//            addAlliance(10000, alliance, RIGHT);
//            addDeposit(10000, alliance, RIGHT);
//        }


        // park
        followTrajectory(park);
    }
}