package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.DUCKY_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.opmodes.AbstractTeleOp.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.opmodes.TestIntake.RESET_TIME;
import static org.firstinspires.ftc.teamcode.opmodes.TestIntake.STOP_TIME;
import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.LEFT;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.MIDDLE;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.BarcodeLocation;
import org.firstinspires.ftc.teamcode.util.CameraPosition;

@Autonomous(name = "Blue Duck", group = "Competition", preselectTeleOp = "Blue TeleOp")
public class BlueDuck extends AbstractAuto {
    public static Pose2d START_POSE = new Pose2d(-36, 63, Math.toRadians(180));
    public static Pose2d DUCK = new Pose2d(-60, 55, Math.toRadians(180));
    public static Pose2d SCORE1 = new Pose2d(-67, 29, Math.toRadians(0));
    public static Pose2d INTAKE2 = new Pose2d(44, 69, Math.toRadians(0));
    public static Pose2d SCORE2 = new Pose2d(12, 71, Math.toRadians(0));
    public static Pose2d INTAKE3 = new Pose2d(48, 73, Math.toRadians(0));
    public static Pose2d SCORE3 = new Pose2d(12, 75, Math.toRadians(0));
    public static Pose2d PARK = new Pose2d(52, 77, Math.toRadians(0));

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

        Trajectory intake1 = robot.drive.trajectoryBuilder(START_POSE)
                .lineToLinearHeading(DUCK)
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

        Trajectory intake3 = robot.drive.trajectoryBuilder(score2.end())
                .lineToLinearHeading(INTAKE3)
                .build();
        Trajectory score3 = robot.drive.trajectoryBuilder(intake3.end())
                .lineToLinearHeading(SCORE3)
                .build();

        Trajectory park = robot.drive.trajectoryBuilder(score3.end())
                .lineToLinearHeading(PARK)
                .build();

        // score preloaded

        addArmPivot(0.1, ARM_PIVOT_POSITION.getDown());
        addArmHopper(0.1, ARM_HOPPER_POSITION.getDown());

        addIntakeServo(1, INTAKE_SERVO_DOWN);

        // 1 block
        addIntake(STOP_TIME, 0);
        resetIntake(RESET_TIME);

        addDelay(0.1);//wait for alliance partner's auto to move

        if (location == LEFT) {
            addAlliance(10000, alliance, LEFT);
            addDeposit(10000, alliance, LEFT);
        } else if (location == MIDDLE) {
            addAlliance(10000, alliance, MIDDLE);
            addDeposit(10000, alliance, MIDDLE);
        } else {
            addAlliance(10000, alliance, RIGHT);
            addDeposit(10000, alliance, RIGHT);
        }


        followTrajectory(intake1);
        addDuckSpinner(7, -DUCKY_SPEED);
        addDuckSpinner(0, 0);
        followTrajectory(score1);
//        resetIntake(2);
//        addDelay(5);
//        addAlliance(10000, alliance, RIGHT);
//        addDeposit(10000, alliance, RIGHT);

        // 2 block
//        addIntake(0, -INTAKE_SPEED);
//        followTrajectory(intake2);
//        followTrajectory(score2);
//        resetIntake(2);
////        addDelay(5);
////        addAlliance(10000, alliance, RIGHT);
////        addDeposit(10000, alliance, RIGHT);
//
//        // 3 block
//        addIntake(0, -INTAKE_SPEED);
//        followTrajectory(intake3);
//        followTrajectory(score3);
//        resetIntake(2);
//        addDelay(5);
//        addAlliance(10000, alliance, RIGHT);
//        addDeposit(10000, alliance, RIGHT);

        // park
//        followTrajectory(park);
    }
}