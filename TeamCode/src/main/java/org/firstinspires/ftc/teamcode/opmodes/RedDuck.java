//package org.firstinspires.ftc.teamcode.opmodes;
//
//import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_POSITION;
//import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_POSITION;
//import static org.firstinspires.ftc.teamcode.hardware.Actuators.DUCKY_SPEED;
//import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_RESET_TIME;
//import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_DOWN;
//import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
//import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
//import static org.firstinspires.ftc.teamcode.util.DepositPosition.HIGH;
//import static org.firstinspires.ftc.teamcode.util.DepositPosition.LOW;
//import static org.firstinspires.ftc.teamcode.util.DepositPosition.MID;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.util.BarcodeLocation;
//import org.firstinspires.ftc.teamcode.util.CameraPosition;
//
//@Autonomous(name = "Red Duck", group = "Competition", preselectTeleOp = "Red TeleOp")
//public class RedDuck extends AbstractAuto {
//    public static Pose2d START_POSE = new Pose2d(-36, -63, Math.toRadians(180));
//    public static Pose2d DUCK = new Pose2d(-60, -55, Math.toRadians(180));
//    public static Pose2d PARK = new Pose2d(-69, -29, Math.toRadians(0));
//
//    @Override
//    public void setAlliance() {
//        this.alliance = RED;
//    }
//
//    @Override
//    public void setCameraPosition() {
//        this.cameraPosition = CameraPosition.RIGHT;
//    }
//
//    @Override
//    public void initializeSteps(BarcodeLocation location) {
//        robot.drive.setPoseEstimate(START_POSE);
//
//        Trajectory duck = robot.drive.trajectoryBuilder(START_POSE)
//                .lineToLinearHeading(DUCK)
//                .build();
//        Trajectory park = robot.drive.trajectoryBuilder(duck.end())
//                .lineToLinearHeading(PARK)
//                .build();
//
//        stopTargetingCamera();
//
//        // reset things
//        addArmPivot(0, ARM_PIVOT_POSITION.getDown());
//        addArmHopper(0, ARM_HOPPER_POSITION.getDown());
//        addIntakeServo(0.5, INTAKE_SERVO_DOWN);
//        resetIntake(INTAKE_RESET_TIME);
//
//        // score preloaded
//        switch(location) {
//            case LEFT:
//                addExtend(10000, BLUE, LOW);
//                addRetract(10000, BLUE, LOW);
//                break;
//            case MIDDLE:
//                addExtend(10000, BLUE, MID);
//                addRetract(10000, BLUE, MID);
//                break;
//            case RIGHT:
//            case UNKNOWN:
//                addExtend(10000, BLUE, HIGH);
//                addRetract(10000, BLUE, HIGH);
//                break;
//        }
//
//        followTrajectory(duck);
//        addDuckSpinner(7, -DUCKY_SPEED);
//        addDuckSpinner(0, 0);
//        followTrajectory(park);
//    }
//}