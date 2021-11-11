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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Auto v1", group="Linear Opmode")
@Disabled
public class AutoV1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor driveFrontLeft;
    DcMotor driveFrontRight;
    DcMotor driveBackLeft;
    DcMotor driveBackRight;
    DcMotor intakeMotor;
    DcMotor linearSlide;
    CRServo duckWheel;
    Servo elementHolder;
    Servo hopper;

    double TICKS_PER_INCH = 28.53// Ticks per revolution = 537.7;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        driveFrontLeft = this.hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontRight = this.hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveBackLeft = this.hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveBackRight = this.hardwareMap.get(DcMotor.class, "driveBackRight");

        driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeMotor = this.hardwareMap.get(DcMotor.class, "intakeMotor");

        linearSlide = this.hardwareMap.get(DcMotor.class, "linearSlide");
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        duckWheel = this.hardwareMap.get(CRServo.class, "duckWheel");

        elementHolder = this.hardwareMap.get(Servo.class,"elementHolder");
        elementHolder.setPosition(0.25);

        hopper = this.hardwareMap.get(Servo.class, "hopper");
        hopper.scaleRange(0.185,1.0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            int firstMoveDist = 12;
            double driveSpeed = 0.3;

            telemetry.addData("Status", "Dist: " + firstMoveDist);
            telemetry.addData("Status", "Speed: " + driveSpeed);
            telemetry.update();
            driveInches(firstMoveDist, driveSpeed);


            turnDumb(12, driveSpeed);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
    }

    private void driveInches(int distance, double driveSpeed) {
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveFrontLeft.setTargetPosition(distance);
        driveFrontRight.setTargetPosition(distance);
        driveBackLeft.setTargetPosition(distance);
        driveBackRight.setTargetPosition(distance);

        driveBackRight.setPower(driveSpeed);
        driveBackLeft.setPower(driveSpeed);
        driveFrontRight.setPower(driveSpeed);
        driveFrontLeft.setPower(driveSpeed);

        while (driveFrontLeft.isBusy() && driveFrontRight.isBusy() && driveBackRight.isBusy() && driveBackLeft.isBusy()) {
            sleep(5);
        }

        driveBackRight.setPower(0);
        driveBackLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveFrontLeft.setPower(0);
    }

    private void turnDumb(int distance, double driveSpeed) {
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveFrontLeft.setTargetPosition(distance);
        driveBackLeft.setTargetPosition(distance);
        driveBackRight.setTargetPosition(-distance);
        driveFrontRight.setTargetPosition(-distance);

        driveBackLeft.setPower(driveSpeed);
        driveFrontLeft.setPower(driveSpeed);
        driveBackRight.setPower(-driveSpeed);
        driveFrontRight.setPower(-driveSpeed);

        while (driveFrontLeft.isBusy() && driveFrontRight.isBusy() && driveBackRight.isBusy() && driveBackLeft.isBusy()) {
            sleep(5);
        }

        driveBackRight.setPower(0);
        driveBackLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveFrontLeft.setPower(0);
    }
}
