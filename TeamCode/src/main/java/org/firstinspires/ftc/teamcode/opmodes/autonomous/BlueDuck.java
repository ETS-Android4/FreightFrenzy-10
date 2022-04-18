package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.GENERAL;

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
    public static Pose2d START_POSE = new Pose2d(-34.6875, 65.75, Math.toRadians(-180));
    public static Pose2d DUCK_SPIN = new Pose2d(-55, 60, Math.toRadians(-180));
    public static Pose2d DUCK_TRANSITION = new Pose2d(-54.5, 58.5, Math.toRadians(-220));
    public static Pose2d DUCK_PICKUP = new Pose2d(-54.5, 58, Math.toRadians(-300));
    public static Pose2d DUCK_SCORE = new Pose2d(-59, 44, Math.toRadians(-180));
    public static Pose2d PARK = new Pose2d(-60, 36, Math.toRadians(-180));

    Trajectory spin;
    Trajectory transition;
    Trajectory pickup;
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
    public boolean useCamera() {
        return true;
    }

    @Override //build the trajectories for this auto
    public void makeTrajectories() {
        robot.drive.setPoseEstimate(START_POSE);

        spin = robot.drive.trajectoryBuilder(START_POSE)
                .lineToLinearHeading(DUCK_SPIN,
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        transition = robot.drive.trajectoryBuilder(spin.end())
                .lineToLinearHeading(DUCK_TRANSITION,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pickup = robot.drive.trajectoryBuilder(transition.end())
                .lineToLinearHeading(DUCK_PICKUP,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
        scorePreloadInAuto(1000, alliance, location, false);
        followTrajectory(spin);
        addDuckSpinner(4, 0.8);
        addIntake(0, -INTAKE_SPEED);
        followTrajectory(transition);
        followTrajectory(pickup);
        addIntake(0, 0);
        followTrajectory(score);
        addExtend(10000, BLUE, GENERAL);
        addRetract(10000, BLUE, GENERAL);
//        followTrajectory(park);
    }
}
