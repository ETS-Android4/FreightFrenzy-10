package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class PoseStorage {

    public static Pose2d currentPose = new Pose2d(-1000.0, -1000.0, Math.toRadians(0));
    public static boolean currentPoseIsDefault = true;

    public static final Pose2d TELEOP_RED_START_POSE = new Pose2d(12,-63,Math.toRadians(0));
    public static final Pose2d TELEOP_BLUE_START_POSE = new Pose2d(12,63,Math.toRadians(0));

    public static final Vector2d SCORE_1_POS_BLUE = new Vector2d(37, 63.5);
    public static final double SCORE_1_HEADING_BLUE = Math.toRadians(0);
    public static final Vector2d SCORE_2_POS_BLUE = new Vector2d(12, 63.5);
    public static final double SCORE_2_HEADING_BLUE = Math.toRadians(0);

    public static final Vector2d SCORE_1_POS_RED = new Vector2d(37, -63.5);
    public static final double SCORE_1_HEADING_RED = Math.toRadians(0);
    public static final Vector2d SCORE_2_POS_RED = new Vector2d(12, -63.5);
    public static final double SCORE_2_HEADING_RED = Math.toRadians(0);

    public static Pose2d currentPose(){ return currentPose; }
    public static boolean currentPoseIsDefault(){ return currentPoseIsDefault; }

    public static final Pose2d TELEOP_RED_START_POSE(){ return TELEOP_RED_START_POSE; }
    public static final Pose2d TELEOP_BLUE_START_POSE(){ return TELEOP_BLUE_START_POSE; }

    public static final Vector2d SCORE_1_POS_BLUE(){ return SCORE_1_POS_BLUE; }
    public static final double SCORE_1_HEADING_BLUE(){ return SCORE_1_HEADING_BLUE; }
    public static final Vector2d SCORE_2_POS_BLUE(){ return SCORE_2_POS_BLUE; }
    public static final double SCORE_2_HEADING_BLUE(){ return SCORE_2_HEADING_BLUE; }

    public static final Vector2d SCORE_1_POS_RED(){ return SCORE_1_POS_RED; }
    public static final double SCORE_1_HEADING_RED(){ return SCORE_1_HEADING_RED; }
    public static final Vector2d SCORE_2_POS_RED(){ return SCORE_2_POS_RED; }
    public static final double SCORE_2_HEADING_RED(){ return SCORE_2_HEADING_RED; }



}
