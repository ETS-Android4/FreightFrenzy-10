package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.hardware.Actuators.ODO_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.hardware.Actuators.ODO_SERVO_UP;
import static org.firstinspires.ftc.teamcode.util.Constants.LEFT_DUCKY;
import static org.firstinspires.ftc.teamcode.util.Constants.ODO_SERVO;
import static org.firstinspires.ftc.teamcode.util.Constants.RIGHT_DUCKY;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.controller.Controller;


@Config
@TeleOp(name = "Rumble Test", group = "Competition")
public class RumbleTest extends OpMode {

    Alliance alliance;

    private Controller driver1;
    private Controller driver2;

    private CRServo leftDucky;
    private CRServo rightDucky;

    private Servo odoServo;

    boolean odoRetracted = false;

    @Override
    public void init() {
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        driver1 = new Controller(gamepad1);
        driver2 = new Controller(gamepad2);

        this.leftDucky = hardwareMap.get(CRServo.class, LEFT_DUCKY);
        this.rightDucky = hardwareMap.get(CRServo.class, RIGHT_DUCKY);
        this.odoServo = hardwareMap.get(Servo.class, ODO_SERVO);
    }

    @Override
    public void init_loop() {
        telemetry.addLine(("Initialized: " + alliance + " alliance selected."));
        telemetry.update();
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
            rightDucky.setPower(0.9);
        } else if (leftDucky.getPower() > 0) {
            leftDucky.setPower(0);
            rightDucky.setPower(0);
        }

        if (driver1.getB().isJustPressed()) {
            odoRetracted = !odoRetracted;
        }

        odoServo.setPosition(odoRetracted ? ODO_SERVO_UP : ODO_SERVO_DOWN);

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
        telemetry.addLine("Odometry: "+odoServo.getPosition());
        telemetry.addLine("Odo retracted: "+odoRetracted);
        telemetry.update();
    }
}
