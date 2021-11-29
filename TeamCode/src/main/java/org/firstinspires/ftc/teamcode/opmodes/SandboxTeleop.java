package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name="Teleop", group="Sandbox")
public class SandboxTeleop extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        this.robot = new Robot(this.hardwareMap);
    }

    @Override
    public void loop() {
        this.robot.setInput(this.gamepad1, this.gamepad2);
    }
}
