/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")
public class TestTeleOp extends OpMode{

    DcMotor driveFrontLeft;
    DcMotor driveFrontRight;
    DcMotor driveBackLeft;
    DcMotor driveBackRight;
    DcMotor intakeMotor;
    DcMotor linearSlide;
    CRServo duckWheel;
    Servo elementHolder;
    Servo hopper;
    //OpenCvCamera fred;
    SamplePipeline george;
    double hopperPosition;
    int targetPosition = 0;

    public static final int WEBCAM_WIDTH = 320;
    public static final int WEBCAM_HEIGHT = 240;
    //public static final OpenCvCameraRotation WEBCAM_ROTATION = OpenCvCameraRotation.UPRIGHT;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.*/


        driveFrontLeft = this.hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontRight = this.hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveBackLeft = this.hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveBackRight = this.hardwareMap.get(DcMotor.class, "driveBackRight");

        intakeMotor = this.hardwareMap.get(DcMotor.class, "intakeMotor");

        linearSlide = this.hardwareMap.get(DcMotor.class, "linearSlide");
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        duckWheel = this.hardwareMap.get(CRServo.class, "duckWheel");

        elementHolder = this.hardwareMap.get(Servo.class,"elementHolder");
        elementHolder.setPosition(0.25);

        hopper = this.hardwareMap.get(Servo.class, "hopper");
        hopper.scaleRange(0.25,1.0);

        //fred is stack camera
        //george is the pipeline
//
//        int stackCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        this.fred = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), stackCameraMonitorViewId);
//        this.george = new SamplePipeline();
//        fred.setPipeline(george);
//        fred.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                fred.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hi there hello!");

    }

//linear slide goes to 4.8 ish
    @Override
    public void loop() {
        double left;
        double right;
        double speed = .6;
        boolean turbo = gamepad1.left_bumper;
        boolean slowdown = gamepad1.right_bumper;
        if (turbo) {
            speed = .8;
        }
        if (slowdown) {
            speed = .25;
        }
        //wheel controls
        double x = (gamepad1.left_stick_x)*speed;
        double y = (-gamepad1.right_stick_y)*speed;

        double frontLeftPower = y+x;
        double frontRightPower = y-x;
        double backLeftPower = y+x;
        double backRightPower = y-x;

        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        frontLeftPower /= max;
        frontRightPower /= max;
        backLeftPower /= max;
        backRightPower /= max;

        driveFrontLeft.setPower(frontLeftPower);
        driveBackLeft.setPower(backLeftPower);
        driveFrontRight.setPower(frontRightPower);
        driveBackRight.setPower(backRightPower);

        //intake motor
        boolean intakePress;

        intakePress = gamepad2.right_bumper;

        if(intakePress){
            intakeMotor.setPower(1);
            this.hopperPosition = 0;
        } else {
            intakeMotor.setPower(0);
            this.hopperPosition = 0.5;
        }

        // Send telemetry message to signify robot running;
        telemetry.addLine(String.format(Locale.US, "FL: %.02f FR: %.02f\nBL: %.02f BR: %.02f",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower));

        //linear slide
        double tR = -5.8;  //total rotation
        double rotationScale = 537.7;
        boolean slideUp = gamepad2.y;
        boolean slideDown = gamepad2.x;
        int currentPosition = linearSlide.getCurrentPosition();
        double maxPosition = tR * rotationScale;

        if(slideUp && currentPosition > maxPosition){
            linearSlide.setPower(-1);
        } else if(slideDown && currentPosition < -200){
            linearSlide.setPower(1);
        } else{
            linearSlide.setPower(0);
        }

        telemetry.addData("linearSlide", currentPosition);

        // upper is -1750
        // middle is -750
        // bottom is 0

        //duck wheel

        double duckSpinLeft = gamepad2.left_trigger;
        double duckSpinRight = gamepad2.right_trigger;

        if(duckSpinLeft>0 && duckSpinRight == 0) {
            duckWheel.setPower(duckSpinLeft);
        } else if(duckSpinLeft == 0 && duckSpinRight > 0) {
            duckWheel.setPower(-duckSpinRight);
        } else {
            duckWheel.setPower(0);
        }

        //hopper rotation

        boolean aPress = gamepad2.a;


        if(aPress){
            this.hopperPosition = 1.0;
        }

        this.hopper.setPosition(this.hopperPosition);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
