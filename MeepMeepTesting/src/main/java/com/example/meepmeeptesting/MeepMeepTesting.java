package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepTesting {
    public static final Pose2d SCORE_1_RED = new Pose2d(37, -66.25, Math.toRadians(0));
    public static final Pose2d SCORE_1_1_RED = new Pose2d(SCORE_1_RED.getX()+0.0001, SCORE_1_RED.getY(), SCORE_1_RED.getHeading());
    public static final Pose2d SCORE_2_RED = new Pose2d(10.6587, -66.25, Math.toRadians(0));
    public static final Pose2d SCORE_1_BLUE = new Pose2d(37, 66.25, Math.toRadians(0));
    public static final Pose2d SCORE_1_1_BLUE = new Pose2d(SCORE_1_BLUE.getX()+0.0001, SCORE_1_BLUE.getY(), SCORE_1_BLUE.getHeading());
    public static final Pose2d SCORE_2_BLUE = new Pose2d(10.6587, 66.25, Math.toRadians(0));

    public static final Pose2d SCORE_1_SHARED_RED = new Pose2d(66.75, -37, Math.toRadians(-90));
    public static final Pose2d SCORE_1_1_SHARED_RED = new Pose2d(SCORE_1_SHARED_RED.getX()+0.0001, SCORE_1_SHARED_RED.getY()+0.0001, SCORE_1_SHARED_RED.getHeading());
    public static final Pose2d SCORE_2_SHARED_RED = new Pose2d(66.75, -12, Math.toRadians(-90));
    public static final Pose2d SCORE_1_SHARED_BLUE = new Pose2d(66.75, 37, Math.toRadians(90));
    public static final Pose2d SCORE_1_1_SHARED_BLUE = new Pose2d(SCORE_1_SHARED_BLUE.getX()+0.0001, SCORE_1_SHARED_BLUE.getY()+0.0001, SCORE_1_SHARED_BLUE.getHeading());
    public static final Pose2d SCORE_2_SHARED_BLUE = new Pose2d(66.75, 12, Math.toRadians(90));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        // Declare our first bot
        RoadRunnerBotEntity redWarehouseBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(30, 30, Math.toRadians(60), Math.toRadians(60), 11)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(45, -56, Math.toRadians(45)))
                                        .setReversed(true)
                                        .lineToSplineHeading(SCORE_1_RED)
                                        .splineToSplineHeading(SCORE_1_1_RED, Math.toRadians(180))
                                        .lineToSplineHeading(SCORE_2_RED)
                                        .build()
                );

        RoadRunnerBotEntity redSharedBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(30, 30, Math.toRadians(60), Math.toRadians(60), 11)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(50, -40, Math.toRadians(-110)))
                                .setReversed(true)
                                .lineToSplineHeading(SCORE_1_SHARED_RED)
                                .splineToSplineHeading(SCORE_1_1_SHARED_RED, Math.toRadians(90))
                                .lineToSplineHeading(SCORE_2_SHARED_RED)
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add bot entities
                .addEntity(redWarehouseBot)
                .addEntity(redSharedBot)
                .start();
    }
}