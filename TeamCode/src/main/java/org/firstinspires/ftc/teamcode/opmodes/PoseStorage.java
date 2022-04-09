package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class PoseStorage {
    public static Pose2d CURRENT_POSE = new Pose2d(0, 0, Math.toRadians(0));
    public static boolean POSE_IS_DEFAULT = true;

    public static final Pose2d START_RED = new Pose2d(10.6875, -65.75, Math.toRadians(0));
    public static final Pose2d START_BLUE = new Pose2d(10.6587, 65.75, Math.toRadians(0));

    public static final Pose2d SCORE_1_RED = new Pose2d(37, -66.25, Math.toRadians(0));
    public static final Pose2d SCORE_2_RED = new Pose2d(10.6587, -66.25, Math.toRadians(0));
    public static final Pose2d SCORE_1_BLUE = new Pose2d(37, 66.25, Math.toRadians(0));
    public static final Pose2d SCORE_2_BLUE = new Pose2d(10.6587, 66.25, Math.toRadians(0));

    public static final Pose2d SCORE_1_SHARED_RED = new Pose2d(66.75, -37, Math.toRadians(-90));
    public static final Pose2d SCORE_2_SHARED_RED = new Pose2d(66.75, -12, Math.toRadians(-90));
    public static final Pose2d SCORE_1_SHARED_BLUE = new Pose2d(66.75, 37, Math.toRadians(90));
    public static final Pose2d SCORE_2_SHARED_BLUE = new Pose2d(66.75, 12, Math.toRadians(90));
}
