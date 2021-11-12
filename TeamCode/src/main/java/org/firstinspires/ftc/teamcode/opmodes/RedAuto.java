//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.hardware.Robot;
//import org.firstinspires.ftc.teamcode.util.BarcodeLocation;
//
//import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_DROP_HIGH;
//import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_DROP_LOW;
//import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_DROP_LOW_POS1;
//import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_DROP_MIDDLE;
//import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_MID;
//import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_DROP_HIGH;
//import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_DROP_LOW;
//import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_DROP_MIDDLE;
//
//@Autonomous(name = "RedAuto")
//public class RedAuto extends LinearOpMode {
//    private Robot robot;
//
//    private BarcodeLocation teamElementLocation;
//
//    public void move(int inches, double power) {
//        robot.drive.setTargetForwardPositionRelative(inches, power);
//        while(robot.drive.isBusy() && opModeIsActive()) {
//            sleep(1);
//        }
//    }
//
//    public void strafe(int inches, double power) {
//        robot.drive.setTargetStrafePositionRelative(inches, power);
//        while(robot.drive.isBusy() && opModeIsActive()) {
//            sleep(1);
//        }
//    }
//
//    public void turn(int degrees, double power) {
//        final float fudge = 7;
//        if (degrees > 0) {
//            while (robot.getGyroHeading360() < degrees-fudge) {
//                robot.drive.setWheels(0, 0, -power);
//            }
//        } else {
//            while (robot.getGyroHeading360() > 360-degrees+fudge) {
//                robot.drive.setWheels(0, 0, power);
//            }
//        }
//        robot.drive.setPower(0);
//        this.sleep(2000);
//        telemetry.addData("", robot.getGyroHeading360());
//        telemetry.update();
//        this.sleep(10000);
//    }
//
//    @Override
//    public void runOpMode() {
//        robot = new Robot(hardwareMap);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//
//        while (!(isStarted() || isStopRequested())) {
//            telemetry.addData("Status", "waiting to start");
//            telemetry.update();
//            idle();
//        }
//
//        robot.slide.setPower(1);
//        teamElementLocation = robot.camera.checkTeamElementLocation();
//
//        move(6,0.5);
//        strafe(10,0.5);
//
//        switch(teamElementLocation) {
//            case LEFT:
//                //dropoff at low position
//                robot.slide.setTargetPosition(SLIDE_DROP_MIDDLE);
//                robot.hopper.setPosition(HOPPER_DROP_LOW_POS1);
//                robot.slide.setTargetPosition(SLIDE_DROP_LOW);
//                robot.hopper.setPosition(HOPPER_DROP_LOW);
//                robot.slide.setTargetPosition(SLIDE_DROP_MIDDLE);
//                robot.hopper.setPosition(HOPPER_MID.l);
//
//                break;
//            case MIDDLE:
//                robot.slide.setTargetPosition(SLIDE_DROP_MIDDLE);
//                robot.hopper.setPosition(HOPPER_DROP_MIDDLE);
//                break;
//            case RIGHT:
//                robot.slide.setTargetPosition(SLIDE_DROP_HIGH);
//                robot.hopper.setPosition(HOPPER_DROP_HIGH);
//                break;
//            case UNKNOWN:
//
//        }
//
//        telemetry.addData("Status", "finished");
//        telemetry.update();
//    }
//}