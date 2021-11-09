package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
    private final Gamepad gamepad;

    private final org.firstinspires.ftc.teamcode.controller.Joystick leftStick;
    private final org.firstinspires.ftc.teamcode.controller.Joystick rightStick;

    private final org.firstinspires.ftc.teamcode.controller.Button dLeft;
    private final org.firstinspires.ftc.teamcode.controller.Button dRight;
    private final org.firstinspires.ftc.teamcode.controller.Button dUp;
    private final org.firstinspires.ftc.teamcode.controller.Button dDown;

    private final org.firstinspires.ftc.teamcode.controller.Button a;
    private final org.firstinspires.ftc.teamcode.controller.Button b;
    private final org.firstinspires.ftc.teamcode.controller.Button x;
    private final org.firstinspires.ftc.teamcode.controller.Button y;

    private final org.firstinspires.ftc.teamcode.controller.Button leftBumper;
    private final org.firstinspires.ftc.teamcode.controller.Button rightBumper;

    private final org.firstinspires.ftc.teamcode.controller.Button back;
    private final org.firstinspires.ftc.teamcode.controller.Button start;

    private final Trigger leftTrigger;
    private final Trigger rightTrigger;

    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;

        leftStick = new org.firstinspires.ftc.teamcode.controller.Joystick();
        rightStick = new org.firstinspires.ftc.teamcode.controller.Joystick();

        dLeft = new org.firstinspires.ftc.teamcode.controller.Button();
        dRight = new org.firstinspires.ftc.teamcode.controller.Button();
        dUp = new org.firstinspires.ftc.teamcode.controller.Button();
        dDown = new org.firstinspires.ftc.teamcode.controller.Button();

        a = new org.firstinspires.ftc.teamcode.controller.Button();
        b = new org.firstinspires.ftc.teamcode.controller.Button();
        x = new org.firstinspires.ftc.teamcode.controller.Button();
        y = new org.firstinspires.ftc.teamcode.controller.Button();

        leftBumper = new org.firstinspires.ftc.teamcode.controller.Button();
        rightBumper = new org.firstinspires.ftc.teamcode.controller.Button();

        back = new org.firstinspires.ftc.teamcode.controller.Button();
        start = new org.firstinspires.ftc.teamcode.controller.Button();

        leftTrigger = new Trigger();
        rightTrigger = new Trigger();
    }

    public void update() {
        leftStick.update(gamepad.left_stick_x, gamepad.left_stick_y);
        rightStick.update(gamepad.right_stick_x, gamepad.right_stick_y);

        dLeft.update(gamepad.dpad_left);
        dRight.update(gamepad.dpad_right);
        dUp.update(gamepad.dpad_up);
        dDown.update(gamepad.dpad_down);

        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);

        leftBumper.update(gamepad.left_bumper);
        rightBumper.update(gamepad.right_bumper);

        back.update(gamepad.back);
        start.update(gamepad.start);

        leftTrigger.update(gamepad.left_trigger);
        rightTrigger.update(gamepad.right_trigger);
    }

    public org.firstinspires.ftc.teamcode.controller.Joystick getLeftStick() {
        return leftStick;
    }
    public org.firstinspires.ftc.teamcode.controller.Joystick getRightStick() {
        return rightStick;
    }

    public org.firstinspires.ftc.teamcode.controller.Button getDLeft() {
        return dLeft;
    }
    public org.firstinspires.ftc.teamcode.controller.Button getDRight() {
        return dRight;
    }
    public org.firstinspires.ftc.teamcode.controller.Button getDUp() {
        return dUp;
    }
    public org.firstinspires.ftc.teamcode.controller.Button getDDown() {
        return dDown;
    }

    public org.firstinspires.ftc.teamcode.controller.Button getA() {
        return a;
    }
    public org.firstinspires.ftc.teamcode.controller.Button getB() {
        return b;
    }
    public org.firstinspires.ftc.teamcode.controller.Button getX() {
        return x;
    }
    public org.firstinspires.ftc.teamcode.controller.Button getY() {
        return y;
    }

    public org.firstinspires.ftc.teamcode.controller.Button getLeftBumper() {
        return leftBumper;
    }
    public org.firstinspires.ftc.teamcode.controller.Button getRightBumper() {
        return rightBumper;
    }

    public org.firstinspires.ftc.teamcode.controller.Button getBack() {
        return back;
    }
    public org.firstinspires.ftc.teamcode.controller.Button getStart() {
        return start;
    }

    public Trigger getLeftTrigger() {
        return leftTrigger;
    }
    public Trigger getRightTrigger() {
        return rightTrigger;
    }
}
