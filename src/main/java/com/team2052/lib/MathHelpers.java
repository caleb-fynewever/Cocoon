package com.team2052.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class MathHelpers {
    public static final Pose2d POSE_2D_ZERO = new Pose2d();

    public static final Pose2d pose2dFromRotation(Rotation2d rotation) {
        return new Pose2d(TRANSLATION_2D_ZERO, rotation);
    }

    public static final Pose2d pose2dFromTranslation(Translation2d translation) {
        return new Pose2d(translation, ROTATION_2D_ZERO);
    }

    public static final Rotation2d ROTATION_2D_ZERO = new Rotation2d();
    public static final Rotation2d ROTATION_2D_PI = Rotation2d.fromDegrees(180.0);

    public static final Translation2d TRANSLATION_2D_ZERO = new Translation2d();

    public static double deadband(double value, double deadband) {
        deadband = Math.abs(deadband);
        if (deadband == 1) {
            return 0;
        }
        double scaledValue = (value + (value < 0 ? deadband : -deadband)) / (1 - deadband);
        return (Math.abs(value) > Math.abs(deadband)) ? scaledValue : 0;
    }
}
