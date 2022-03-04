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
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Declare our first bot
        RoadRunnerBotEntity redWarehouseBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(30, 30, Math.toRadians(60), Math.toRadians(60), 11)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(12, -63, 0))
                                        .splineToConstantHeading(new Vector2d(36, -63), Math.toRadians(0))
                                        .splineToConstantHeading(new Vector2d(56, -63), Math.toRadians(0),
                                                new MinVelocityConstraint(Arrays.asList(
                                                        new AngularVelocityConstraint(30),
                                                        new MecanumVelocityConstraint(10, 11)
                                                )),
                                                new ProfileAccelerationConstraint(30)
                                        )
//                                        .lineToSplineHeading(new Pose2d(36, -63, Math.toRadians(0)))
//                                        .lineToSplineHeading(new Pose2d(50, -63, Math.toRadians(0)),
//                                                new MinVelocityConstraint(Arrays.asList(
//                                                        new AngularVelocityConstraint(30),
//                                                        new MecanumVelocityConstraint(10, 11)
//                                                )),
//                                                new ProfileAccelerationConstraint(30)
//                                        )
//                                        .lineToSplineHeading(new Pose2d(12, -63, Math.toRadians(0)))
                                        .build()
                );
        RoadRunnerBotEntity blueWarehouseBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(30, 30, Math.toRadians(60), Math.toRadians(60), 11)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(50, 45, 0))
                                        .lineToLinearHeading(new Pose2d(40, 63, Math.toRadians(0)))
                                        .lineToLinearHeading(new Pose2d(12, 63, Math.toRadians(0)))
                                        .build()
                );

        RoadRunnerBotEntity redDuckBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(30, 30, Math.toRadians(60), Math.toRadians(60), 11)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-36, -63, -180))
                                        .setReversed(true)
//                                        .splineToConstantHeading(new Vector2d(40, 63), Math.toRadians(0))
                                        .lineToLinearHeading(new Pose2d(-60, -60, Math.toRadians(-135)))
                                        .lineToLinearHeading(new Pose2d(-36, -60, Math.toRadians(-135)))//intake duck
                                        .lineToLinearHeading(new Pose2d(-36, -63, Math.toRadians(-180)))//score duck
//                                        .splineToConstantHeading(new Vector2d(12, 63), Math.toRadians(0))
                                        .lineToLinearHeading(new Pose2d(-63, -36, Math.toRadians(-180)))
                                        .build()
                );

        RoadRunnerBotEntity blueDuckBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(30, 30, Math.toRadians(60), Math.toRadians(60), 11)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-36, 63, 180))
                                        .setReversed(true)
//                                        .splineToConstantHeading(new Vector2d(40, 63), Math.toRadians(0))
                                        .lineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(135)))
                                        .lineToLinearHeading(new Pose2d(-36, 60, Math.toRadians(135)))//intake duck
                                        .lineToLinearHeading(new Pose2d(-36, 63, Math.toRadians(180)))//score duck
//                                        .splineToConstantHeading(new Vector2d(12, 63), Math.toRadians(0))
                                        .lineToLinearHeading(new Pose2d(-63, 36, Math.toRadians(180)))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add bot entities
                .addEntity(redWarehouseBot)
//                .addEntity(blueWarehouseBot)
//                .addEntity(redDuckBot)
//                .addEntity(blueDuckBot)
                .start();
    }
}