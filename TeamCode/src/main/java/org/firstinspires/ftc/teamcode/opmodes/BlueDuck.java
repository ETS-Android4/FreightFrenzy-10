package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_RESET_TIME;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.opmodes.AbstractTeleOp.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.BarcodeLocation;
import org.firstinspires.ftc.teamcode.util.CameraPosition;

@Autonomous(name = "Blue Duck", group = "Competition", preselectTeleOp = "Blue TeleOp")
public class BlueDuck extends AbstractAuto {

    //define the waypoints in this auto
    public static Pose2d START_POSE = new Pose2d(-36, 63, Math.toRadians(180));
    public static Pose2d DUCK_SPIN = new Pose2d(-60, 55, Math.toRadians(180));
    public static Pose2d DUCK_TRANSITION = new Pose2d(-55, 57, Math.toRadians(45));
    public static Pose2d DUCK_PICKUP = new Pose2d(-40, 57, Math.toRadians(45));
    public static Pose2d DUCK_SCORE = new Pose2d(-36, 63, Math.toRadians(180));
    public static Pose2d PARK = new Pose2d(-60, 36, Math.toRadians(180));

    Trajectory spin;
    Trajectory transition;
    Trajectory pickup;
    Trajectory score;
    Trajectory park;

    @Override
    public void setAlliance() {
        this.alliance = BLUE;
    }

    @Override
    public void setCameraPosition() {
        this.cameraPosition = CameraPosition.LEFT;
    }

    @Override
    public boolean useCamera() {
        return true;
    }

    @Override //build the trajectories for this auto
    public void makeTrajectories() {
        robot.drive.setPoseEstimate(START_POSE);

        spin = robot.drive.trajectoryBuilder(START_POSE)
                .lineToLinearHeading(DUCK_SPIN)
                .build();
        transition = robot.drive.trajectoryBuilder(spin.end())
                .lineToLinearHeading(DUCK_TRANSITION
//                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        pickup = robot.drive.trajectoryBuilder(transition.end())
                .lineToLinearHeading(DUCK_PICKUP,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
//                .lineToLinearHeading(new Pose2d(-63, -48, Math.toRadians(-135)),
//                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                )
//                .lineToLinearHeading(new Pose2d(-40, -48, Math.toRadians(-135)),
//                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                )
//                .lineToLinearHeading(new Pose2d(-59, -59, Math.toRadians(-135)),
//                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                )
                .build();
        score = robot.drive.trajectoryBuilder(pickup.end())
                .lineToLinearHeading(DUCK_SCORE)
                .build();
        park = robot.drive.trajectoryBuilder(score.end())
                .lineToLinearHeading(PARK)
                .build();
    }

    @Override //setup the specific actions in order for this auto
    public void initializeSteps(BarcodeLocation location) {
        // set arm
        addArmPivot(0, ARM_PIVOT_POSITION.getDown());
        addArmHopper(0, ARM_HOPPER_POSITION.getDown());
        addIntakeServo(0.25, INTAKE_SERVO_DOWN);
        resetIntake(INTAKE_RESET_TIME);

        // score preloaded
        addAlliance(10000, RED, location);
        addDeposit(10000, RED, location);

        // move
        followTrajectory(spin);
        addDuckSpinner(4, 1);
        addIntake(0, -INTAKE_SPEED);
        followTrajectory(transition);
        followTrajectory(pickup);
        followTrajectory(score);
        addIntake(0, 0);
        addAlliance(10000, RED, RIGHT);
        addDeposit(10000, RED, RIGHT);
        followTrajectory(park);
    }
}
