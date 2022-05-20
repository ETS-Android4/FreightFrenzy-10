package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.controller.Controller;

@Config
@TeleOp(name = "SpeedBump", group = "Competition")
public class SpeedBump extends OpMode{


    Alliance alliance;

    private Controller driver1;
    private Controller driver2;

    private Robot robot;


        @Override
        public void init() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            telemetry.addLine("Initializing Robot...");
            telemetry.update();


            driver1 = new Controller(gamepad1);
            driver2 = new Controller(gamepad2);

            robot = new Robot(hardwareMap, alliance);

            //robot.actuators.setOdoServo(0.01);

            robot.actuators.startup();


        }

        @Override
        public void init_loop() {
//        robot.updateLights();
            telemetry.addLine(("Initialized"));
            telemetry.update();
        }

        @Override
        public void loop() {
//            robot.updateLights();
            driver1.update();
            driver2.update();

            double x, y, z, temp;

            //new stuff with "exponential" speed

            //get the initial values
            x = driver1.getLeftStick().getY();
            y = -driver1.getLeftStick().getX();
            z = -driver1.getRightStick().getX();

            //transform the linear controller output into the nonlinear curve
            x = 0.152 * Math.tan(1.42 * x); // blue desmos curve
            //y =  0.2*Math.tan(1.3734*y)  ;
            z = 0.152 * Math.tan(1.42 * z);

                    //old stuff with boost button
//                if (driver1.getLeftBumper().isPressed()) {
//                    x = driver1.getLeftStick().getY();
//                    y = -driver1.getLeftStick().getX();
//                    z = -driver1.getRightStick().getX();
//                } else {
//                    x = driver1.getLeftStick().getY(); //* DRIVE_SPEED;
//                    y = -driver1.getLeftStick().getX(); //* DRIVE_SPEED;
//                    z = -driver1.getRightStick().getX(); //* DRIVE_SPEED;
//                }

            temp = Math.max(x,Math.max(y,z));
            if(temp > 1){
                x = x/temp;
                y = y/temp;
                z = z/temp;
            }

            robot.drive.setWeightedDrivePower(new Pose2d(x, -y, z));
            robot.actuators.update(x,y,z);


            robot.drive.update();

//            robot.actuators.setDuckies(driver2.getY().isPressed() ? DUCKY_SPEED : 0, alliance);

            // retractables
            if (driver1.getY().isJustPressed()) {
                robot.actuators.intakeRetracted = !robot.actuators.intakeRetracted;
            }
            if (!driver1.getStart().isPressed() && driver1.getA().isJustPressed()) {
                robot.actuators.odoRetracted = !robot.actuators.odoRetracted;
            }

            //robot.actuators.setIntakeServo(robot.actuators.intakeRetracted ? 0.01 : 0.99);
            //robot.actuators.setOdoServo(robot.actuators.odoRetracted ? 0.01 : 0.99);


            //robot.actuators.update();

            // telemetry
            telemetry.addLine("Alliance: " + alliance);
            telemetry.addLine(robot.getTelemetry());
            telemetry.update();
        }
}


