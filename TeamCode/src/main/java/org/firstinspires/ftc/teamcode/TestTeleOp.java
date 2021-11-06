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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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
    Servo hopper;
    double hopperPosition;
    int targetPosition = 0;

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
        linearSlide.setTargetPosition(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        duckWheel = this.hardwareMap.get(CRServo.class, "duckWheel");

        hopper = this.hardwareMap.get(Servo.class, "hopper");
        hopper.scaleRange(0.185,1.0);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hi there hello!");

    }

//linear slide goes to 4.8 ish
    @Override
    public void loop() {
        double left;
        double right;

        //wheel controls
        left = (gamepad1.left_stick_y)*0.6;
        right = (-gamepad1.right_stick_y)*0.6;



            driveFrontLeft.setPower(left);
            driveBackLeft.setPower(left);
            driveFrontRight.setPower(right);
            driveBackRight.setPower(right);



        //intake motor
        boolean intakePress;

        intakePress = gamepad1.right_bumper;

        if(intakePress){
            intakeMotor.setPower(1);
            this.hopperPosition = 0;
        } else {
            intakeMotor.setPower(0);
            this.hopperPosition = 0.5;
        }


        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);


        //linear slide
        double tR = -5.8;  //total rotation
        double rotationScale = 537.7;
        boolean slideUp = gamepad1.y;
        boolean slideDown = gamepad1.x;

        if(slideUp){
            if(targetPosition > tR * rotationScale){
                targetPosition-=10;
            }
            linearSlide.setTargetPosition(targetPosition);
            linearSlide.setPower(.5);
        }

        if(slideDown){
            if(targetPosition < 0){
                targetPosition+=10;
            }
            linearSlide.setTargetPosition(targetPosition);
            linearSlide.setPower(.5);
        }

        //duck wheel

        double duckSpinLeft = gamepad1.left_trigger;
        double duckSpinRight = gamepad1.right_trigger;

        if(duckSpinLeft>0 && duckSpinRight == 0) {
            duckWheel.setPower(duckSpinLeft);
        } else if(duckSpinLeft == 0 && duckSpinRight > 0) {
            duckWheel.setPower(-duckSpinRight);
        } else {
            duckWheel.setPower(0);
        }

        //hopper rotation

        boolean aPress = gamepad1.a;


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
