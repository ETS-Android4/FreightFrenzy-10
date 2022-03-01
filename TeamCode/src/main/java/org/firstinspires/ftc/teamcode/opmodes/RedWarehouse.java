package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_RESET_TIME;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.HIGH;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.LOW;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.MID;

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
    public static Pose2d CREEP = new Pose2d(40, -63, Math.toRadians(0));
    public static Pose2d SCORE = new Pose2d(12, -63, Math.toRadians(0));
    public static Pose2d PARK = new Pose2d(36, -63, Math.toRadians(0));
    Trajectory intake;
    Trajectory creep;
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
    public void makeTrajectories() {
        robot.drive.setPoseEstimate(START_POSE);

        intake = robot.drive.trajectoryBuilder(START_POSE)
                .lineToLinearHeading(INTAKE)
                .build();

        creep = robot.drive.trajectoryBuilder(intake.end())
                .lineToLinearHeading(CREEP,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        score = robot.drive.trajectoryBuilder(creep.end())
                .lineToLinearHeading(SCORE)
                .build();

        park = robot.drive.trajectoryBuilder(intake.end())
                .lineToLinearHeading(PARK)
                .build();

    }

    @Override
    public void initializeSteps(BarcodeLocation location) {

        stopTargetingCamera();

        // reset things
        addArmPivot(0, ARM_PIVOT_POSITION.getDown());
        addArmHopper(0, ARM_HOPPER_POSITION.getDown());
        addIntakeServo(0.5, INTAKE_SERVO_DOWN);
        resetIntake(INTAKE_RESET_TIME);

        // score preloaded
        switch(getTeamElementLocation()) {
            case LEFT:
                addExtend(10000, alliance, LOW);
                addRetract(10000, alliance, LOW);
                break;
            case MIDDLE:
                addExtend(10000, alliance, MID);
                addRetract(10000, alliance, MID);
                break;
            case RIGHT:
            case UNKNOWN:
                addExtend(10000, alliance, HIGH);
                addRetract(10000, alliance, HIGH);
                break;
        }

        robot.drive.followTrajectory(intake); // go into the warehouse
        // cycle
        int cycles = 3;
        for (int i = 0; i < cycles; i++) {
            cycleBlockInAuto(1000, intake, score, creep, alliance, HIGH);
        }

        // park
        followTrajectory(park);
    }
}