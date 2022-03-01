package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_RESET_TIME;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_SPEED;
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
    public static Pose2d INTAKE = new Pose2d(36, -63, Math.toRadians(0));
    public static Pose2d CREEP = new Pose2d(40, -59, Math.toRadians(0));
    public static Pose2d SCORE = new Pose2d(12, -63, Math.toRadians(0));
    public static Pose2d PARK = new Pose2d(36, -63, Math.toRadians(0));

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
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory score = robot.drive.trajectoryBuilder(intake.end())
                .lineToLinearHeading(SCORE)
                .build();

        Trajectory park = robot.drive.trajectoryBuilder(intake.end())
                .lineToLinearHeading(PARK)
                .build();

        stopTargetingCamera();

        // set arm
        addArmPivot(0, ARM_PIVOT_POSITION.getDown());
        addArmHopper(0, ARM_HOPPER_POSITION.getDown());
        addIntakeServo(0.5, INTAKE_SERVO_DOWN);
        resetIntake(INTAKE_RESET_TIME);

        // score preloaded
        addAlliance(10000, alliance, getTeamElementLocation());
        addDeposit(10000, alliance, getTeamElementLocation());

        // cycle
        /*for (int i = 0; i < 3; i++) {
            cycleBlockInAuto(1000, intake, score, creep, alliance, RIGHT);
        }*/

        //cycle faster
        for (int i = 0; i < 3; i++) {
            cycleBlockInAuto2(1000, intake, score, creep, alliance, RIGHT);
        }


        /*while(getRuntime()<20){
            cycleBlockInAuto2(1000, intake, score, creep, alliance, RIGHT);
        }
        while(getRuntime()<24){
            if(robot.actuators.hopperIsFull()){
                cycleBlockInAuto2(1000, intake, score, creep, alliance, RIGHT);
            }
        }
        robot.actuators.setIntake(-INTAKE_SERVO_SPEED/2);
        robot.drive.followTrajectory(creep);
        while(getRuntime()<29.8){
            robot.drive.update();
        }
        robot.actuators.setIntake(0);
        robot.drive.stopthetrajectory*/


        // park
        followTrajectory(park);

    }
}







//OLD CODE
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
