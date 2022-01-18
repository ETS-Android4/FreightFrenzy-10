package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.oldutil.BarcodeLocation;

import static org.firstinspires.ftc.teamcode.oldutil.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.oldutil.Alliance.RED;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.HOPPER_DROP_HIGH;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.HOPPER_DROP_LOW;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.HOPPER_DROP_LOW_POS1;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.HOPPER_DROP_MIDDLE;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.HOPPER_MID;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.SLIDE_DROP_HIGH;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.SLIDE_DROP_LOW;
import static org.firstinspires.ftc.teamcode.oldutil.Configurables.SLIDE_DROP_MIDDLE;

@Autonomous(name = "Blue Duck", group = "Competition", preselectTeleOp = "Blue TeleOp")
public class BlueDuck extends AbstractAuto {
    public static Pose2d START_POSE = new Pose2d(-36, 63, Math.toRadians(90));

    public static Pose2d FORWARD  = new Pose2d(-36, 30, Math.toRadians(90));
    public static Pose2d BACK  = new Pose2d(-36, 55, Math.toRadians(90));
    public static Pose2d DEPOSIT  = new Pose2d(-28, 36, Math.toRadians(135));
    public static Pose2d READY_TO_SPIN  = new Pose2d(-61.5, 59, Math.toRadians(135));
    public static Pose2d PARK  = new Pose2d(-63, 33.5, Math.toRadians(180));

    @Override
    public void setAlliance() {
        this.alliance = BLUE;
    }

    @Override
    public void initializeSteps(BarcodeLocation location) {
        robot.drive.setPoseEstimate(START_POSE);

        Trajectory forward = robot.drive.trajectoryBuilder(START_POSE)
                .lineToLinearHeading(FORWARD)
                .build();
        Trajectory back = robot.drive.trajectoryBuilder(forward.end())
                .lineToLinearHeading(BACK)
                .build();
        Trajectory deposit = robot.drive.trajectoryBuilder(back.end())
                .lineToLinearHeading(DEPOSIT)
                .build();
        Trajectory readyToSpin = robot.drive.trajectoryBuilder(deposit.end())
                .lineToLinearHeading(READY_TO_SPIN)
                .build();
        Trajectory park = robot.drive.trajectoryBuilder(readyToSpin.end())
                .lineToLinearHeading(PARK)
                .build();

        followTrajectory(forward);
        followTrajectory(back);
        followTrajectory(deposit);
        switch(location) {
            case LEFT:
                addSlide(SLIDE_DROP_HIGH);
                addHopper(0, HOPPER_DROP_LOW_POS1);
                addSlide(SLIDE_DROP_LOW);
                addHopper(1, HOPPER_DROP_LOW);
                addSlide(SLIDE_DROP_HIGH);
                addHopper(0, HOPPER_MID.l);
                addSlide(0);
                break;
            case MIDDLE:
                addSlide(SLIDE_DROP_MIDDLE);
                addHopper(1, HOPPER_DROP_MIDDLE);
                addHopper(0.2, HOPPER_MID.l);
                addSlide(0);
                break;
            case RIGHT:
            case UNKNOWN:
                addSlide(SLIDE_DROP_HIGH);
                addHopper(1, HOPPER_DROP_HIGH);
                addHopper(0.2, HOPPER_MID.l);
                addSlide(0);
                break;
        }
        addHopper(0, 0.2);
        followTrajectory(readyToSpin);
        addDuckSpinner(5, -0.3);
        addDuckSpinner(0, 0);
        followTrajectory(park);
        stopTargetingCamera();
    }
}