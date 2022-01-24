package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
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
        telemetry.addLine(robot.getTelemetry());
        //New Code 3/5
        telemetry.addLine(robot.getLSTelemetry());
        telemetry.update();
    }
}