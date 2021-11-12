package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;

import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.Alliance.NEITHER;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.Configurables.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_DELAY;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_DROP_HIGH;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_DROP_LOW;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_DROP_LOW_POS1;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_DROP_MIDDLE;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_INIT;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_MID;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_START;
import static org.firstinspires.ftc.teamcode.util.Configurables.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.util.Configurables.SERVO_MOVEMENT;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_CUTOFF;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_DROP_HIGH;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_DROP_LOW;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_DROP_MIDDLE;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_TICKS_PER_CYCLE;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_SPEED;
import static org.firstinspires.ftc.teamcode.util.MathUtil.about;
import static org.firstinspires.ftc.teamcode.util.MathUtil.clamp;

@Config
public class AbstractTeleOp extends OpMode {
    private Controller driver1;
    private Controller driver2;

    private Robot robot;
    Alliance alliance;

    private int targetPos;
    private double servoPos;

    private boolean scoringLow = false;
    private int scoringLowPos = 0;

    private double currentTime;
    private double startMacro;
    private boolean macroing = false;

    public void setAlliance() {
        this.alliance = NEITHER;
    }

    @Override
    public void init() {
        telemetry.addLine("Initializing Robot...");
        telemetry.update();

        setAlliance();

        driver1 = new Controller(gamepad1);
        driver2 = new Controller(gamepad2);

        robot = new Robot(hardwareMap);
        robot.camera.initBarcodeWebcam();

        targetPos = 0;
        servoPos = HOPPER_INIT;

        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slide.setTargetPosition(targetPos);
        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.setPower(SLIDE_SPEED);

        robot.hopper.setPosition(servoPos);
    }

    @Override
    public void init_loop() {
        if (robot.camera.getFrameCount() > 0) {
            telemetry.addLine("Initialized");
            telemetry.addLine("Alliance: "+alliance);
            telemetry.update();
        }
    }

    @Override
    public void loop() {
        currentTime = getRuntime();

        driver1.update();
        driver2.update();

        // wheels
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
        if (driver2.getRightTrigger().getValue() > 0) {
            robot.intake.setPower(driver2.getRightTrigger().getValue()*INTAKE_SPEED);
        } else if (driver2.getLeftTrigger().getValue() > 0) {
            robot.intake.setPower(driver2.getLeftTrigger().getValue()*INTAKE_SPEED);
        }

        // transfer
        targetPos -= driver2.getLeftStick().getY()* SLIDE_TICKS_PER_CYCLE;
        servoPos += driver2.getRightStick().getY()/SERVO_MOVEMENT;

        if (driver2.getDUp().isJustPressed()) {
            targetPos = SLIDE_DROP_HIGH;
        } else if (driver2.getDLeft().isJustPressed()) {
            targetPos = SLIDE_DROP_MIDDLE;
        } else if (driver2.getDDown().isJustPressed()) {
            scoringLow = true;
            scoringLowPos = 0;
        } else if (driver2.getDRight().isJustPressed()) {
            targetPos = 0;
            servoPos = HOPPER_INIT;
        } else if (driver2.getY().isJustPressed()) {
            if (targetPos == SLIDE_DROP_HIGH) {
                servoPos = HOPPER_DROP_HIGH;
            } else if (targetPos == SLIDE_DROP_MIDDLE) {
                servoPos = HOPPER_DROP_MIDDLE;
            } else if (targetPos == SLIDE_DROP_LOW) {
                servoPos = HOPPER_DROP_LOW;
            } else {
                servoPos = HOPPER_DROP_MIDDLE;
            }
        } else if (driver2.getX().isJustPressed()) {
            servoPos = HOPPER_MID.l;
        }

        if (scoringLow) {
            switch (scoringLowPos) {
                case 0:
                    targetPos = SLIDE_DROP_MIDDLE;
                    servoPos = HOPPER_MID.l;
                    if (about(robot.slide.getCurrentPosition(), SLIDE_DROP_MIDDLE)) {
                        scoringLowPos++;
                    }
                    break;
                case 1:
                    servoPos = HOPPER_DROP_LOW_POS1;
                    scoringLowPos++;
                    break;
                case 2:
                    targetPos = SLIDE_DROP_LOW;
                    if (about(robot.slide.getCurrentPosition(), SLIDE_DROP_LOW)) {
                        scoringLowPos++;
                    }
                    break;
                case 3:
                    servoPos = HOPPER_DROP_LOW;
                    if (!macroing) {
                        macroing = true;
                        startMacro = currentTime;
                    }
                    if (currentTime - startMacro > HOPPER_DELAY) {
                        scoringLowPos++;
                        macroing = false;
                    }
                    break;
                case 4:
                    targetPos = SLIDE_DROP_MIDDLE;
                    servoPos = HOPPER_MID.l;
                    if (about(robot.slide.getCurrentPosition(), SLIDE_DROP_MIDDLE)) {
                        scoringLowPos++;
                    }
                    break;
                case 5:
                    targetPos = 0;
                    if (about(robot.slide.getCurrentPosition(), 0)) {
                        scoringLowPos++;
                    }
                    break;
                case 6:
                    scoringLowPos = 0;
                    scoringLow = false;
                    break;
            }
        }

        // clamp
        targetPos = clamp(targetPos, -2400, 0);
        if (targetPos > SLIDE_CUTOFF) {
//            if (robot.intake.getPower() != 0) {
//                servoPos = HOPPER_MID.l;
//            } else {
//                servoPos = HOPPER_INIT;
//            }
            servoPos = clamp(servoPos, HOPPER_START.l, HOPPER_START.u);
        } else {
            servoPos = clamp(servoPos, HOPPER_MID.l, HOPPER_MID.u);
        }

        robot.slide.setPower(SLIDE_SPEED);
        robot.slide.setTargetPosition(targetPos);
        robot.hopper.setPosition(servoPos);

        // ducky
        if (driver2.getA().isPressed()) {
            if (alliance == RED) {
                robot.ducky.setPower(-1);
            } else if (alliance == BLUE) {
                robot.ducky.setPower(1);
            }
        }

        // telemetry
        telemetry.addData("servo pos", servoPos);
        telemetry.addData("slide pos", targetPos);
        telemetry.update();
    }
}
