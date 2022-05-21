package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;

@Config
@TeleOp(name = "Teleop", group = "Competition")
public class Teleop extends OpMode {
    public static double DRIVE_SPEED = 0.5;
    public static double INTAKE_SPEED = 0.75;
    public static double SLIDE_SPEED = 10;
    public static double HOPPER_SPEED = 0.05;

    public static int SLIDE_MIN = 0;
    public static int SLIDE_MAX = 3000;
    public static double HOPPER_MIN = 0.1;
    public static double HOPPER_MAX = 0.9;
    public static double LEFT_SWEEPER_UP = 0.9;
    public static double LEFT_SWEEPER_DOWN = 0.1;
    public static double RIGHT_SWEEPER_UP = 0.1;
    public static double RIGHT_SWEEPER_DOWN = 0.9;
    public static double FLIPPER_SERVO_OPEN = 0.9;
    public static double FLIPPER_SERVO_CLOSED = 0.1;

    private Controller driver1;
    private Controller driver2;

    private Robot robot;

    private int slidePos = 0;
    private double hopperPos = 0;
    private boolean sweepersReleased = false;
    private boolean flipperReleased = false;
    private boolean flipperUp = false;

    @Override
    public void init() {
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        driver1 = new Controller(gamepad1);
        driver2 = new Controller(gamepad2);

        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        driver1.update();
        driver2.update();

        // drive base
        double x, y, z;
        if (driver1.getLeftBumper().isPressed()) {
            x = driver1.getLeftStick().getX();
            y = driver1.getLeftStick().getY();
            z = driver1.getRightStick().getX();
        } else {
            x = driver1.getLeftStick().getX() * DRIVE_SPEED;
            y = driver1.getLeftStick().getY() * DRIVE_SPEED;
            z = driver1.getRightStick().getX() * DRIVE_SPEED;
        }
        robot.drive.setWheels(x, y, z);

        // intake
        double leftTrigger = driver2.getLeftTrigger().getValue();
        double rightTrigger = driver2.getRightTrigger().getValue();
        if (leftTrigger > rightTrigger) {
            robot.intake.setPower(-leftTrigger * INTAKE_SPEED);
        } else {
            robot.intake.setPower(rightTrigger * INTAKE_SPEED);
        }

        // slides
        slidePos += driver2.getLeftStick().getY() * SLIDE_SPEED;
        slidePos = Math.max(Math.min(slidePos, SLIDE_MAX), SLIDE_MIN);
        hopperPos += driver2.getRightStick().getY() * HOPPER_SPEED;
        hopperPos = Math.max(Math.min(slidePos, HOPPER_MAX), HOPPER_MIN);

        // sweepers
        if (driver2.getA().isJustPressed()) {
            sweepersReleased = !sweepersReleased;
        }
        if (sweepersReleased) {
            robot.leftSweeper.setPosition(LEFT_SWEEPER_DOWN);
            robot.rightSweeper.setPosition(RIGHT_SWEEPER_DOWN);
        } else {
            robot.leftSweeper.setPosition(LEFT_SWEEPER_UP);
            robot.rightSweeper.setPosition(RIGHT_SWEEPER_UP);
        }

        // flipper
        if (driver2.getY().isJustPressed()) {
            flipperReleased = !flipperReleased;
        }
        if (flipperReleased) {
            robot.flipperServo.setPosition(FLIPPER_SERVO_OPEN);
        } else {
            robot.flipperServo.setPosition(FLIPPER_SERVO_CLOSED);
        }

        // telemetry
        telemetry.addLine(robot.getTelemetry());
        telemetry.update();
    }
}
