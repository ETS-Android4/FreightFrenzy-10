//package org.firstinspires.ftc.teamcode.opmodes;
//
//import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_POSITION;
//import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_POSITION;
//import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_DOWN;
//import static org.firstinspires.ftc.teamcode.hardware.Actuators.SLIDES_ALLIANCE_LOW;
//import static org.firstinspires.ftc.teamcode.opmodes.AbstractTeleOp.INTAKE_SLOW_SPEED;
//import static org.firstinspires.ftc.teamcode.opmodes.AbstractTeleOp.INTAKE_SPEED;
//import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.getAccelerationConstraint;
//import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.getVelocityConstraint;
//import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
//import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
//import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.LEFT;
//import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.MIDDLE;
//import static org.firstinspires.ftc.teamcode.util.BarcodeLocation.RIGHT;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.util.BarcodeLocation;
//import org.firstinspires.ftc.teamcode.util.CameraPosition;
//
//
//@Disabled
//@Config
//@Autonomous(name = "Test Intake", group = "Development")
//public class TestIntake extends AbstractAuto {
//    public static double STOP_TIME = 0.2;
//    public static double RESET_TIME = 2;
//
//    @Override
//    public void setAlliance() {
//        this.alliance = BLUE;
//    }
//
//    @Override
//    public void setCameraPosition() {
//        this.cameraPosition = CameraPosition.RIGHT;
//    }
//
//    @Override
//    public void initializeSteps(BarcodeLocation location) {
////        for(int i = 0 ; i<100; i++){
////            addArm(10);
////        }
//
//        Pose2d START = new Pose2d(-36, 36, Math.toRadians(90));
//        robot.drive.setPoseEstimate(START);
//        Trajectory forward = robot.drive.trajectoryBuilder(START)
//                .back(96,
//                        getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                )
//                .build();
//
//        followTrajectory(forward);
//
////        addArmPivot(0.1, ARM_PIVOT_POSITION.getDown());
////        addArmHopper(0.1, ARM_HOPPER_POSITION.getDown());
////
////        addIntakeServo(1, INTAKE_SERVO_DOWN);
////
////        resetIntake(RESET_TIME);
////
////        addAlliance(10000, alliance, RIGHT);
////        addDeposit(10000, alliance, RIGHT);
////
////        addIntake(STOP_TIME, 0);
////        resetIntake(RESET_TIME);
////
////        addDelay(5);
////
////        addAlliance(10000, alliance, MIDDLE);
////        addDeposit(10000, alliance, MIDDLE);
////
////        addIntake(STOP_TIME, 0);
////        resetIntake(RESET_TIME);
////
////        addDelay(5);
////
////        addAlliance(10000, alliance, LEFT);
////        addDeposit(10000, alliance, LEFT);
//
//
////        addIntake(0, -INTAKE_SPEED);
////        addDelay(1);
////        addIntake(STOP_TIME, 0);
////        resetIntake(RESET_TIME);
////
////        addIntake(0, -INTAKE_SPEED);
////        addDelay(1);
////        addIntake(STOP_TIME, 0);
////        resetIntake(RESET_TIME);
////
////        addIntake(0, -INTAKE_SPEED);
////        addDelay(1);
////        addIntake(STOP_TIME, 0);
////        resetIntake(RESET_TIME);
////
////        addIntake(0, -INTAKE_SPEED);
////        addDelay(1);
////        addIntake(STOP_TIME, 0);
////        resetIntake(RESET_TIME);
////
////        addIntake(0, -INTAKE_SPEED);
////        addDelay(1);
////        addIntake(STOP_TIME, 0);
////        resetIntake(RESET_TIME);
//        stopTargetingCamera();
//    }
//}