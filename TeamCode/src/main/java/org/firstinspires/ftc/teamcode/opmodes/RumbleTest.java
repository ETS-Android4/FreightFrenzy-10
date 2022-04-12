package org.firstinspires.ftc.teamcode.opmodes;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_MAX;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_MIN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_HOPPER_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_MAX;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_MIN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ARM_PIVOT_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.DUCKY_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.INTAKE_SERVO_UP;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ODO_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ODO_SERVO_UP;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.SLIDES_MAX;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.SLIDES_MIN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.SLIDES_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.TURRET_MAX;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.TURRET_MIN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.TURRET_SPEED;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.Constants.LEFT_DUCKY;
import static org.firstinspires.ftc.teamcode.util.Constants.RIGHT_DUCKY;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.GENERAL;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.HIGH;
import static org.firstinspires.ftc.teamcode.util.DepositPosition.SHARED;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.DepositPosition;
import org.firstinspires.ftc.teamcode.util.controller.Controller;


@Config
@TeleOp(name = "Rumble Test", group = "Competition")
public class RumbleTest extends OpMode {

    Alliance alliance;

    private Controller driver1;
    private Controller driver2;

    private CRServo leftDucky;
//    private CRServo rightDucky;


    //private Robot robot;

    @Override
    public void init() {
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        driver1 = new Controller(gamepad1);
        driver2 = new Controller(gamepad2);

        this.leftDucky = hardwareMap.get(CRServo.class, LEFT_DUCKY);
//        this.rightDucky = hardwareMap.get(CRServo.class, RIGHT_DUCKY);

        //robot.actuators.odoRetracted = false;

        //robot.lights.setPattern(98);
    }

    @Override
    public void init_loop() {
        //robot.updateLights();
        telemetry.addLine(("Initialized: " + alliance + " alliance selected."));
        telemetry.update();
        //robot.actuators.odoRetracted=false;
        //robot.actuators.setOdoServo(0.01);
    }

    boolean flag = true;

    @Override
    public void loop() {

        driver1.update();
        driver2.update();

        if(flag){
        driver1.rumble();
        driver2.rumble();
        flag = false;
        }

        if (driver1.getA().isPressed()) {
            leftDucky.setPower(0.9);
//            rightDucky.setPower(0.9);
        } else if (leftDucky.getPower() > 0) {
            leftDucky.setPower(0);
//            rightDucky.setPower(0);
        }

//        if(driver1.getA().isJustPressed()){
//            driver1.rumble(1000);
//        }else{driver1.stopRumble();}
//
//        if(driver2.getA().isPressed()){
//        driver2.rumble(1000);
//        }else{driver2.stopRumble();}
//
//        if(driver1.getB().isPressed()){
//            driver1.rumbleBlips(1000);
//        }else{driver1.stopRumble();}
//
//        if(driver2.getB().isPressed()){
//            driver1.rumbleBlips(1000);
//        }else{driver2.stopRumble();}
//
//




        //robot.actuators.update();

        // telemetry
        telemetry.addLine("Rumble mode: " + driver1.isRumbling());
        //telemetry.addLine(robot.getTelemetry());
        telemetry.update();
    }
}
