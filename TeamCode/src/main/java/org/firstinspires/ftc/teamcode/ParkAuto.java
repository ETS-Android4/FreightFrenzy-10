package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Park Auto")

public class ParkAuto extends LinearOpMode {

    DcMotor driveFrontLeft;
    DcMotor driveFrontRight;
    DcMotor driveBackLeft;
    DcMotor driveBackRight;

    public void move(int inches, double power){
        int ticks = (int)((inches/(6*3.14))*537.7);
        driveFrontLeft.setTargetPosition(ticks);
        driveFrontRight.setTargetPosition(ticks);
        driveBackLeft.setTargetPosition(ticks);
        driveBackRight.setTargetPosition(ticks);
        driveFrontLeft.setPower(power);
        driveFrontRight.setPower(power);
        driveBackLeft.setPower(power);
        driveBackRight.setPower(power);
    }

    @Override
    public void runOpMode(){
        driveFrontLeft = this.hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontRight = this.hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveBackLeft = this.hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveBackRight = this.hardwareMap.get(DcMotor.class, "driveBackRight");

        driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while(!isStarted() || isStopRequested()) {
            telemetry.addLine("Ready to launch at your command Houston");
            telemetry.update();
            idle();
        }
        move(40, 0.5);
    }
}
