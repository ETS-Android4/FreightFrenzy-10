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
//Right now, Auto Blue Right is testing all new features except pushing element out of way
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.visioncode.BarcodePipeline;
import org.firstinspires.ftc.teamcode.visioncode.Cvhelper;
import org.firstinspires.ftc.teamcode.visioncode.Detection;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "DuckBlue", group = "Linear Opmode")
public class AutoBlueRight extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Cvhelper.BarcodeLocation teamElementLocation;
    double LinearSPos = 0;
    int noLinear = 1;
    DcMotor driveFrontLeft;
    DcMotor driveFrontRight;
    DcMotor driveBackLeft;
    DcMotor driveBackRight;
    DcMotor intakeMotor;
    DcMotor linearSlide;
    CRServo duckWheel;
    Servo elementHolder;
    Servo hopper;
    int CLocation;
    Camera camera;


    double TICKS_PER_INCH = 28.53; // Ticks per revolution = 537.7;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        driveFrontLeft = this.hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveBackLeft = this.hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveFrontRight = this.hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveBackRight = this.hardwareMap.get(DcMotor.class, "driveBackRight");
        camera = new Camera(hardwareMap);
        camera.initBarcodeWebcam();
        intakeMotor = this.hardwareMap.get(DcMotor.class, "intakeMotor");

        linearSlide = this.hardwareMap.get(DcMotor.class, "linearSlide");
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        duckWheel = this.hardwareMap.get(CRServo.class, "duckWheel");

        elementHolder = this.hardwareMap.get(Servo.class, "elementHolder");
        elementHolder.setPosition(0.1);

        hopper = this.hardwareMap.get(Servo.class, "hopper");
        hopper.scaleRange(0.25, 1.0);
        hopper.setPosition(0.5);

        while (camera.getFrameCount() < 1) {
            idle();
        }

        // Wait for the game to start (driver presses PLAY)
        while(!isStarted() || isStopRequested()){
            teamElementLocation = camera.checkTeamElementLocation();
            telemetry.addData("Camera", teamElementLocation);
            telemetry.update();
        }

        runtime.reset();

        double driveSpeed = 0.3;
        int sleeptime = 1000;
        int firstMoveDist = 30;

        //Read Camera (Work-In-Progress)
        if(teamElementLocation == Cvhelper.BarcodeLocation.LEFT){
            LinearSPos = 0;
            noLinear = 0;
        } else if(teamElementLocation == Cvhelper.BarcodeLocation.MIDDLE){
            LinearSPos = 60;
        } else if(teamElementLocation == Cvhelper.BarcodeLocation.RIGHT){
            LinearSPos = 0;
        }

        //Drive forward
        driveInchesEnc(firstMoveDist, driveSpeed);
        sleep(sleeptime);
        driveInchesEnc(-15, -driveSpeed);
        sleep(sleeptime);
        telemetry.addData("Status", "Run beater");
        telemetry.update();
        runBeater(1000, -1.0);
        sleep(sleeptime);

        //Turn left towards score
        turnDumbEnc(8, driveSpeed);
        sleep(sleeptime);

        //Drive slightly forward before score
        driveInchesEnc(5, driveSpeed);
        sleep(sleeptime);

        //Score

        driveLinearSlide((110-LinearSPos)*noLinear, 1);
        intakeMotor.setPower(1.0);
        hopper.setPosition(1.0);
        sleep(1000);
        intakeMotor.setPower(0);
        hopper.setPosition(0.5);
        driveLinearSlide((-109.6+LinearSPos)*noLinear, -1);

        turnDumbEnc(1, driveSpeed);
        sleep(sleeptime);
        //Back up
        duckWheel.setPower(-1.0);
        driveInchesEnc(-35, -driveSpeed/3);
        sleep(sleeptime/2);
        driveInchesEnc(-1, -driveSpeed/10);
        //Spin Duck
        duckWheel.setPower(0);
        //Turn towards storage unit
        turnDumbEnc(-8, -driveSpeed);
        sleep(sleeptime);

        driveInchesEnc(9, driveSpeed);
        sleep(sleeptime);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.update();
    }

    private void runBeater(int ms, double speed) {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double start = time.milliseconds();
        intakeMotor.setPower(speed);
        while (opModeIsActive() && time.milliseconds() - start < ms) {
            sleep(5);
        }
        intakeMotor.setPower(0);
    }

    private void driveInchesEnc(int distance, double driveSpeed) {
        telemetry.addData("Status", "Dist: " + distance);
        telemetry.addData("Status", "Speed: " + driveSpeed);
        telemetry.update();
        distance = (int) (distance * TICKS_PER_INCH);

        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveBackRight.setPower(-1*driveSpeed);
        driveFrontRight.setPower(-1*driveSpeed);
        driveBackLeft.setPower(driveSpeed);
        driveFrontLeft.setPower(driveSpeed);

        while (opModeIsActive() && Math.abs(driveFrontRight.getCurrentPosition()) < Math.abs(distance)) {
            sleep(5);
            telemetry.addData("FR", driveFrontRight.getCurrentPosition());
            telemetry.update();
        }

        driveBackRight.setPower(0);
        driveBackLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveFrontLeft.setPower(0);
    }
    private void driveLinearSlide(double distance, double slideSpeed) {
        telemetry.addData("Status", "Dist: " + distance);
        telemetry.addData("Status", "Speed: " + slideSpeed);
        telemetry.update();
        distance = (int) (distance * TICKS_PER_INCH);

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setPower(-1*slideSpeed);


        while (opModeIsActive() && Math.abs(linearSlide.getCurrentPosition()) < Math.abs(distance)) {
            sleep(5);
            telemetry.addData("FL", driveFrontLeft.getCurrentPosition());
            telemetry.addData("FR", driveFrontRight.getCurrentPosition());
            telemetry.addData("BL", driveBackLeft.getCurrentPosition());
            telemetry.addData("BR", driveBackRight.getCurrentPosition());
            telemetry.update();
        }

        linearSlide.setPower(0);
    }

//    private void driveInches(int distance, double driveSpeed) {
//        telemetry.addData("Status", "Dist: " + distance);
//        telemetry.addData("Status", "Speed: " + driveSpeed);
//        telemetry.update();
//        distance = (int) (distance * TICKS_PER_INCH);
//
//        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        driveFrontLeft.setTargetPosition(distance);
//        driveFrontRight.setTargetPosition(distance);
//        driveBackLeft.setTargetPosition(distance);
//        driveBackRight.setTargetPosition(distance);
//
//        driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        driveBackRight.setPower(driveSpeed);
//        driveBackLeft.setPower(driveSpeed);
//        driveFrontRight.setPower(driveSpeed);
//        driveFrontLeft.setPower(driveSpeed);
//
//        while (opModeIsActive() && driveFrontLeft.isBusy() && driveFrontRight.isBusy() && driveBackRight.isBusy() && driveBackLeft.isBusy()) {
//            sleep(5);
//        }
//
//        driveBackRight.setPower(0);
//        driveBackLeft.setPower(0);
//        driveFrontRight.setPower(0);
//        driveFrontLeft.setPower(0);
//    }
//
//
//
//    private void turnDumb(int distance, double driveSpeed) {
//        telemetry.addData("Status", "Dist: " + distance);
//        telemetry.addData("Status", "Speed: " + driveSpeed);
//        telemetry.update();
//        distance = (int) (distance * TICKS_PER_INCH);
//        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        driveFrontLeft.setTargetPosition(distance);
//        driveBackLeft.setTargetPosition(distance);
//        driveBackRight.setTargetPosition(-distance);
//        driveFrontRight.setTargetPosition(-distance);
//
//        driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        driveBackLeft.setPower(driveSpeed);
//        driveFrontLeft.setPower(driveSpeed);
//        driveBackRight.setPower(-driveSpeed);
//        driveFrontRight.setPower(-driveSpeed);
//
//        while (opModeIsActive() && driveFrontLeft.isBusy() && driveFrontRight.isBusy() && driveBackRight.isBusy() && driveBackLeft.isBusy()) {
//            sleep(5);
//        }
//
//        driveBackRight.setPower(0);
//        driveBackLeft.setPower(0);
//        driveFrontRight.setPower(0);
//        driveFrontLeft.setPower(0);
//    }

    private void turnDumbEnc(int distance, double driveSpeed) {
        telemetry.addData("Status", "Dist: " + distance);
        telemetry.addData("Status", "Speed: " + driveSpeed);
        telemetry.update();
        distance = (int) (distance * TICKS_PER_INCH);
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveBackLeft.setPower(driveSpeed);
        driveFrontLeft.setPower(driveSpeed);
        driveBackRight.setPower(driveSpeed);
        driveFrontRight.setPower(driveSpeed);

        while (opModeIsActive() && Math.abs(driveFrontRight.getCurrentPosition()) < Math.abs(distance)) {
            sleep(5);
        }

        driveBackRight.setPower(0);
        driveBackLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveFrontLeft.setPower(0);
    }
/*
    private void turnTime(double time, double driveSpeed) {
        telemetry.addData("Status", "Dist: " + distance);
        telemetry.addData("Status", "Speed: " + driveSpeed);
        telemetry.update();
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveBackLeft.setPower(driveSpeed);
        driveFrontLeft.setPower(driveSpeed);
        driveBackRight.setPower(driveSpeed);
        driveFrontRight.setPower(driveSpeed);

        while (opModeIsActive() && Math.abs(driveFrontLeft.getCurrentPosition()) < Math.abs(distance)) {
            sleep(5);
        }

        driveBackRight.setPower(0);
        driveBackLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveFrontLeft.setPower(0);
    }

 */
}
